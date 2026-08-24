// goldsrc.cpp - GoldSrc (.mdl v10) decompile. The format shares nothing with
// v44+ past the "IDST" magic, so it gets its own reader and writes the sources
// GoldSrc authored from: SMD meshes, SMD animations and 8-bit .bmp textures.

#include "goldsrc.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "mdlfile.h"

namespace fs = std::filesystem;

namespace mdldecompiler {
namespace {

// --- v10 layout (re-declared from the public Half-Life SDK's studio.h) -------

constexpr int32_t kIdStudioHeader = 0x54534449;   // "IDST"
constexpr int32_t kIdSequenceHeader = 0x51534449; // "IDSQ"

#pragma pack(push, 4)
struct hdr_t {
    int32_t id, version;
    char name[64];
    int32_t length;
    pm::Vector3 eyeposition, min, max, bbmin, bbmax;
    int32_t flags;
    int32_t numbones, boneindex;
    int32_t numbonecontrollers, bonecontrollerindex;
    int32_t numhitboxes, hitboxindex;
    int32_t numseq, seqindex;
    int32_t numseqgroups, seqgroupindex;
    int32_t numtextures, textureindex, texturedataindex;
    int32_t numskinref, numskinfamilies, skinindex;
    int32_t numbodyparts, bodypartindex;
    int32_t numattachments, attachmentindex;
    int32_t soundtable, soundindex, soundgroups, soundgroupindex;
    int32_t numtransitions, transitionindex;
};

struct bone_t {
    char name[32];
    int32_t parent, flags;
    int32_t bonecontroller[6];
    float value[6]; // x y z, then the three euler angles in radians
    float scale[6];
};

struct bonecontroller_t {
    int32_t bone, type;
    float start, end;
    int32_t rest, index;
};

struct bbox_t {
    int32_t bone, group;
    pm::Vector3 bbmin, bbmax;
};

struct seqgroup_t {
    char label[32];
    char name[64];
    int32_t cache, data; // group 0 keeps its anim base in `data`
};

struct seqdesc_t {
    char label[32];
    float fps;
    int32_t flags, activity, actweight;
    int32_t numevents, eventindex;
    int32_t numframes;
    int32_t numpivots, pivotindex;
    int32_t motiontype, motionbone;
    pm::Vector3 linearmovement;
    int32_t automoveposindex, automoveangleindex;
    pm::Vector3 bbmin, bbmax;
    int32_t numblends, animindex;
    int32_t blendtype[2];
    float blendstart[2], blendend[2];
    int32_t blendparent;
    int32_t seqgroup;
    int32_t entrynode, exitnode, nodeflags, nextseq;
};

struct event_t {
    int32_t frame, event, type;
    char options[64];
};

struct attachment_t {
    char name[32];
    int32_t type, bone;
    pm::Vector3 org, vectors[3];
};

struct bodypart_t {
    char name[64];
    int32_t nummodels, base, modelindex;
};

struct model_t {
    char name[64];
    int32_t type;
    float boundingradius;
    int32_t nummesh, meshindex;
    int32_t numverts, vertinfoindex, vertindex;
    int32_t numnorms, norminfoindex, normindex;
    int32_t numgroups, groupindex;
};

struct mesh_t {
    int32_t numtris, triindex, skinref, numnorms, normindex;
};

struct texture_t {
    char name[64];
    int32_t flags, width, height, index;
};

// One bone channel, RLE-compressed. offset[0..2] is position, [3..5] rotation;
// zero means the bone holds its default value for the whole clip.
struct anim_t {
    uint16_t offset[6];
};
#pragma pack(pop)

constexpr int32_t kStudioLooping = 0x0001;

// --- bounds-checked file access ---------------------------------------------

// Every offset in a .mdl is file-absolute and attacker-controlled, so nothing
// is dereferenced without a range check.
struct Buf {
    std::vector<char> data;

    template <typename T>
    const T* At(int32_t off, int count = 1) const {
        if (off <= 0 || count <= 0)
            return nullptr;
        const size_t start = static_cast<size_t>(off);
        if (start > data.size() ||
            sizeof(T) * static_cast<size_t>(count) > data.size() - start)
            return nullptr;
        return reinterpret_cast<const T*>(data.data() + start);
    }
    // offset 0 means "absent" everywhere except the header itself
    const struct hdr_t* Header() const {
        return data.size() >= sizeof(hdr_t) ? reinterpret_cast<const hdr_t*>(data.data())
                                            : nullptr;
    }
};

// char[N] that a hostile file need not terminate.
template <size_t N>
std::string Fixed(const char (&s)[N]) {
    return std::string(s, std::find(s, s + N, '\0'));
}

// Usable as a filename and as a token in a script.
std::string Sanitize(const std::string& s) {
    std::string out;
    for (unsigned char c : s)
        out += (c > 0x20 && c < 0x7f && c != '"' && c != '/' && c != '\\' && c != ':')
                   ? static_cast<char>(c)
                   : '_';
    while (!out.empty() && out.back() == '_')
        out.pop_back();
    return out;
}

std::FILE* Create(const std::string& path) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        std::printf("  cannot write %s\n", path.c_str());
    return f;
}

// --- the model, plus the files it splits itself across ----------------------

struct Model {
    Buf main;
    Buf textures;              // <base>T.mdl, when the main file carries none
    std::vector<Buf> seqFiles; // <base>%02d.mdl, indexed by sequence group
    const hdr_t* h = nullptr;
    const Buf* tex = nullptr; // where the texture table actually lives
    const hdr_t* texHdr = nullptr;

    std::vector<pm::matrix3x4> bind; // bind-pose world matrix per bone
    std::vector<std::string> boneNames;
};

bool Read(const std::string& path, Buf& b) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        std::fclose(f);
        return false;
    }
    b.data.resize(static_cast<size_t>(size));
    const size_t got = std::fread(b.data.data(), 1, b.data.size(), f);
    std::fclose(f);
    return got == b.data.size();
}

// The sibling files GoldSrc splits a model across: "<base>T.mdl" holds the
// textures and "<base>%02d.mdl" the sequence groups past group 0.
void LoadSiblings(Model& m, const std::string& in) {
    const std::string base = StripExt(in);
    m.tex = &m.main;
    m.texHdr = m.h;
    if (m.h->numtextures == 0) {
        for (const char* suffix : {"T.mdl", "t.mdl"}) {
            if (!Read(base + suffix, m.textures))
                continue;
            const hdr_t* th = m.textures.Header();
            if (th && th->id == kIdStudioHeader && th->numtextures > 0) {
                m.tex = &m.textures;
                m.texHdr = th;
                std::printf("textures:    %s\n", (base + suffix).c_str());
                break;
            }
            m.textures.data.clear();
        }
    }

    m.seqFiles.resize(static_cast<size_t>(std::max(m.h->numseqgroups, 1)));
    for (size_t g = 1; g < m.seqFiles.size(); ++g) {
        char suffix[16];
        std::snprintf(suffix, sizeof suffix, "%02zu.mdl", g);
        if (!Read(base + suffix, m.seqFiles[g])) {
            std::printf("missing sequence group file %s\n", (base + suffix).c_str());
            continue;
        }
        const hdr_t* sh = m.seqFiles[g].Header();
        if (!sh || (sh->id != kIdSequenceHeader && sh->id != kIdStudioHeader)) {
            std::printf("%s is not a sequence group file\n", (base + suffix).c_str());
            m.seqFiles[g].data.clear();
        }
    }
}

// Bind pose: bone->value holds the parent-relative rest pose that every SMD
// skeleton block and every vertex is expressed against.
void BuildBindPose(Model& m) {
    const bone_t* bones = m.main.At<bone_t>(m.h->boneindex, m.h->numbones);
    m.bind.resize(static_cast<size_t>(std::max(m.h->numbones, 0)));
    m.boneNames.resize(m.bind.size());
    for (int i = 0; bones && i < m.h->numbones; ++i) {
        const bone_t& b = bones[i];
        m.boneNames[i] = Sanitize(Fixed(b.name));
        if (m.boneNames[i].empty())
            m.boneNames[i] = "bone" + std::to_string(i);
        pm::matrix3x4 local;
        pm::AngleMatrix({b.value[3], b.value[4], b.value[5]},
                        {b.value[0], b.value[1], b.value[2]}, local);
        m.bind[i] = (b.parent >= 0 && b.parent < i)
                        ? pm::ConcatTransforms(m.bind[b.parent], local)
                        : local;
    }
}

// --- SMD ---------------------------------------------------------------------

void WriteNodes(std::FILE* f, const Model& m) {
    const bone_t* bones = m.main.At<bone_t>(m.h->boneindex, m.h->numbones);
    std::fprintf(f, "version 1\nnodes\n");
    for (int i = 0; bones && i < m.h->numbones; ++i)
        std::fprintf(f, "%d \"%s\" %d\n", i, m.boneNames[i].c_str(), bones[i].parent);
    std::fprintf(f, "end\n");
}

// The rest pose, which is both a mesh SMD's skeleton block and frame 0 of any
// clip that leaves a bone alone.
void WriteBindSkeleton(std::FILE* f, const Model& m) {
    const bone_t* bones = m.main.At<bone_t>(m.h->boneindex, m.h->numbones);
    std::fprintf(f, "skeleton\ntime 0\n");
    for (int i = 0; bones && i < m.h->numbones; ++i) {
        const float* v = bones[i].value;
        std::fprintf(f, "%d %s %s %s %s %s %s\n", i, F(v[0]).c_str(), F(v[1]).c_str(),
                     F(v[2]).c_str(), F(v[3]).c_str(), F(v[4]).c_str(), F(v[5]).c_str());
    }
    std::fprintf(f, "end\n");
}

// Materials are matched by name, so a mesh SMD names the texture bare - the
// .bmp beside it is only there for the material author.
std::vector<std::string> MaterialNames(const Model& m) {
    const texture_t* tex = m.tex->At<texture_t>(m.texHdr->textureindex, m.texHdr->numtextures);
    std::vector<std::string> names;
    for (int i = 0; tex && i < m.texHdr->numtextures; ++i) {
        std::string n = Sanitize(StripExt(Fixed(tex[i].name)));
        names.push_back(n.empty() ? "material" + std::to_string(i) : n);
    }
    return names;
}

// skinref -> texture, through skin family `family`. The table lives beside the
// textures, so a T.mdl carries its own.
int SkinTexture(const Model& m, int family, int skinref) {
    const int16_t* skins = m.tex->At<int16_t>(m.texHdr->skinindex,
                                              m.texHdr->numskinref * m.texHdr->numskinfamilies);
    if (!skins || family < 0 || family >= m.texHdr->numskinfamilies || skinref < 0 ||
        skinref >= m.texHdr->numskinref)
        return skinref;
    const int16_t t = skins[family * m.texHdr->numskinref + skinref];
    return (t >= 0 && t < m.texHdr->numtextures) ? t : skinref;
}

struct Tri {
    int16_t vert, norm, s, t;
};

// One model's triangles as SMD. Vertices are stored in their own bone's space,
// so the bind pose is what puts them back in model space.
void WriteMeshSmd(const Model& m, const model_t& mo, const std::string& path,
                  const std::vector<std::string>& materials) {
    std::FILE* f = Create(path);
    if (!f)
        return;
    WriteNodes(f, m);
    WriteBindSkeleton(f, m);
    std::fprintf(f, "triangles\n");

    const pm::Vector3* verts = m.main.At<pm::Vector3>(mo.vertindex, mo.numverts);
    const pm::Vector3* norms = m.main.At<pm::Vector3>(mo.normindex, mo.numnorms);
    const uint8_t* vbone = m.main.At<uint8_t>(mo.vertinfoindex, mo.numverts);
    const uint8_t* nbone = m.main.At<uint8_t>(mo.norminfoindex, mo.numnorms);
    const mesh_t* meshes = m.main.At<mesh_t>(mo.meshindex, mo.nummesh);
    const texture_t* tex = m.tex->At<texture_t>(m.texHdr->textureindex, m.texHdr->numtextures);
    const int nbones = static_cast<int>(m.bind.size());

    auto emit = [&](const Tri& t, int texIndex) {
        const int vi = t.vert, ni = t.norm;
        const int vb = (verts && vbone && vi >= 0 && vi < mo.numverts) ? vbone[vi] : 0;
        const int nb = (norms && nbone && ni >= 0 && ni < mo.numnorms) ? nbone[ni] : 0;
        pm::Vector3 p{}, n{0, 0, 1};
        if (verts && vi >= 0 && vi < mo.numverts && vb < nbones)
            p = pm::VectorTransform(verts[vi], m.bind[vb]);
        if (norms && ni >= 0 && ni < mo.numnorms && nb < nbones)
            n = pm::Normalize(pm::VectorRotate(norms[ni], m.bind[nb]));
        const float w = (tex && texIndex < m.texHdr->numtextures && tex[texIndex].width > 0)
                            ? static_cast<float>(tex[texIndex].width)
                            : 1.0f;
        const float hgt = (tex && texIndex < m.texHdr->numtextures && tex[texIndex].height > 0)
                              ? static_cast<float>(tex[texIndex].height)
                              : 1.0f;
        // studiomdl stored s = u * width and t = (1 - v) * height
        std::fprintf(f, "%d %s %s %s %s\n", (vb < nbones ? vb : 0), V3(p).c_str(),
                     V3(n).c_str(), F(t.s / w).c_str(), F(1.0f - t.t / hgt).c_str());
    };

    for (int i = 0; meshes && i < mo.nummesh; ++i) {
        const int texIndex = SkinTexture(m, 0, meshes[i].skinref);
        const std::string mat = (texIndex >= 0 && texIndex < static_cast<int>(materials.size()))
                                    ? materials[texIndex]
                                    : std::string("material");
        // command list: a positive count starts a strip, a negative one a fan,
        // and zero ends the mesh
        int32_t off = meshes[i].triindex;
        for (;;) {
            const int16_t* cmd = m.main.At<int16_t>(off);
            if (!cmd || *cmd == 0)
                break;
            const bool fan = *cmd < 0;
            const int count = fan ? -*cmd : *cmd;
            const Tri* run = m.main.At<Tri>(off + 2, count);
            if (!run)
                break;
            for (int k = 2; k < count; ++k) {
                std::fprintf(f, "%s\n", mat.c_str());
                if (fan) {
                    emit(run[0], texIndex);
                    emit(run[k - 1], texIndex);
                    emit(run[k], texIndex);
                } else if (k & 1) {
                    emit(run[k - 1], texIndex);
                    emit(run[k - 2], texIndex);
                    emit(run[k], texIndex);
                } else {
                    emit(run[k - 2], texIndex);
                    emit(run[k - 1], texIndex);
                    emit(run[k], texIndex);
                }
            }
            off += 2 + count * static_cast<int32_t>(sizeof(Tri));
        }
    }
    std::fprintf(f, "end\n");
    std::fclose(f);
}

// One channel of one bone at one frame. The RLE runs hold `total` frames of
// which the first `valid` are stored; the last stored value carries the rest.
float AnimValue(const Buf& b, const anim_t& a, const bone_t& bone, int channel, int frame) {
    const float base = bone.value[channel];
    if (a.offset[channel] == 0)
        return base;
    const ptrdiff_t d = reinterpret_cast<const char*>(&a) - b.data.data();
    int32_t off = static_cast<int32_t>(d) + a.offset[channel];
    int k = frame;
    for (;;) {
        const int16_t* run = b.At<int16_t>(off, 1);
        if (!run)
            return base;
        const uint8_t valid = static_cast<uint8_t>(*run & 0xff);
        const uint8_t total = static_cast<uint8_t>((*run >> 8) & 0xff);
        if (valid == 0 || total == 0)
            return base;
        if (total > k) {
            const int pick = std::min<int>(k, valid - 1);
            const int16_t* v = b.At<int16_t>(off + 2 * (1 + pick), 1);
            return v ? base + *v * bone.scale[channel] : base;
        }
        k -= total;
        off += 2 * (1 + valid);
    }
}

// One blend of one sequence as an SMD clip.
void WriteAnimSmd(const Model& m, const seqdesc_t& s, int blend, const std::string& path) {
    const Buf& src = (s.seqgroup > 0 && static_cast<size_t>(s.seqgroup) < m.seqFiles.size())
                         ? m.seqFiles[s.seqgroup]
                         : m.main;
    int32_t animBase = s.animindex;
    if (s.seqgroup == 0) {
        const seqgroup_t* g = m.main.At<seqgroup_t>(m.h->seqgroupindex, 1);
        if (g)
            animBase += g->data;
    }
    const bone_t* bones = m.main.At<bone_t>(m.h->boneindex, m.h->numbones);
    const anim_t* anim = src.At<anim_t>(
        animBase + blend * m.h->numbones * static_cast<int32_t>(sizeof(anim_t)), m.h->numbones);
    if (!bones || src.data.empty()) {
        std::printf("  \"%s\": animation data is missing\n", Fixed(s.label).c_str());
        return;
    }

    std::FILE* f = Create(path);
    if (!f)
        return;
    WriteNodes(f, m);
    std::fprintf(f, "skeleton\n");
    const int frames = std::max(s.numframes, 1);
    for (int t = 0; t < frames; ++t) {
        std::fprintf(f, "time %d\n", t);
        for (int i = 0; i < m.h->numbones; ++i) {
            float v[6];
            for (int c = 0; c < 6; ++c)
                v[c] = anim ? AnimValue(src, anim[i], bones[i], c, t) : bones[i].value[c];
            std::fprintf(f, "%d %s %s %s %s %s %s\n", i, F(v[0]).c_str(), F(v[1]).c_str(),
                         F(v[2]).c_str(), F(v[3]).c_str(), F(v[4]).c_str(), F(v[5]).c_str());
        }
    }
    std::fprintf(f, "end\n");
    std::fclose(f);
}

// --- textures ----------------------------------------------------------------

void Put32(std::FILE* f, uint32_t v) {
    std::fputc(v & 0xff, f);
    std::fputc((v >> 8) & 0xff, f);
    std::fputc((v >> 16) & 0xff, f);
    std::fputc((v >> 24) & 0xff, f);
}

// 8-bit .bmp, palette and all - the same thing GoldSrc studiomdl read back in.
void WriteBmp(const Model& m, const texture_t& t, const std::string& path) {
    if (t.width <= 0 || t.height <= 0)
        return;
    const size_t pixels = static_cast<size_t>(t.width) * static_cast<size_t>(t.height);
    const uint8_t* data = m.tex->At<uint8_t>(t.index, static_cast<int>(pixels) + 768);
    if (!data) {
        std::printf("  %s: texture data is out of range\n", Fixed(t.name).c_str());
        return;
    }
    const uint8_t* pal = data + pixels;

    std::FILE* f = Create(path);
    if (!f)
        return;
    const uint32_t stride = (static_cast<uint32_t>(t.width) + 3) & ~3u;
    const uint32_t bits = 14 + 40 + 256 * 4;
    std::fputc('B', f);
    std::fputc('M', f);
    Put32(f, bits + stride * static_cast<uint32_t>(t.height));
    Put32(f, 0);
    Put32(f, bits);
    Put32(f, 40);
    Put32(f, static_cast<uint32_t>(t.width));
    Put32(f, static_cast<uint32_t>(t.height));
    Put32(f, 1 | (8 << 16)); // 1 plane, 8 bits per pixel
    Put32(f, 0);             // BI_RGB
    Put32(f, stride * static_cast<uint32_t>(t.height));
    Put32(f, 0);
    Put32(f, 0);
    Put32(f, 256);
    Put32(f, 256);
    for (int i = 0; i < 256; ++i) {
        std::fputc(pal[i * 3 + 2], f);
        std::fputc(pal[i * 3 + 1], f);
        std::fputc(pal[i * 3 + 0], f);
        std::fputc(0, f);
    }
    // the texture is stored top row first, a .bmp bottom row first
    for (int y = t.height - 1; y >= 0; --y) {
        std::fwrite(data + static_cast<size_t>(y) * static_cast<size_t>(t.width), 1,
                    static_cast<size_t>(t.width), f);
        for (uint32_t p = static_cast<uint32_t>(t.width); p < stride; ++p)
            std::fputc(0, f);
    }
    std::fclose(f);
}

// --- the script --------------------------------------------------------------

struct Qc {
    std::FILE* f;
    void Line(const std::string& s) { std::fprintf(f, "%s\n", s.c_str()); }
    void Blank() { std::fprintf(f, "\n"); }
};

// The compiler swizzles a script-space point into model space as (-y, x, z).
pm::Vector3 Unswizzle(const pm::Vector3& v) {
    return {v.y, -v.x, v.z};
}

std::string Deg(float radians) {
    return F(static_cast<float>(radians * pm::kRad2Deg));
}

// Which motion axis a blend controller drives, used to name its pose parameter.
const char* BlendAxis(int32_t type) {
    static const struct {
        int32_t bit;
        const char* name;
    } kAxes[] = {{0x0001, "x"},  {0x0002, "y"},  {0x0004, "z"},   {0x0008, "xr"},
                 {0x0010, "yr"}, {0x0020, "zr"}, {0x0040, "lx"},  {0x0080, "ly"},
                 {0x0100, "lz"}, {0x0200, "ax"}, {0x0400, "ay"},  {0x0800, "az"},
                 {0x1000, "axr"}, {0x2000, "ayr"}, {0x4000, "azr"}};
    for (const auto& a : kAxes)
        if (type & a.bit)
            return a.name;
    return "blend";
}

// A model's mesh name has to be unique across the whole file - a bodypart with
// two "body.smd" models is normal.
std::string UniqueName(std::set<std::string>& used, std::string name) {
    if (name.empty())
        name = "model";
    std::string out = name;
    for (int n = 2; !used.insert(out).second; ++n)
        out = name + std::to_string(n);
    return out;
}

int Decompile(const std::string& in, const std::string& dir, const std::string& outPath) {
    Model m;
    if (!Read(in, m.main))
        return std::printf("read error: cannot read \"%s\"\n", in.c_str()), 1;
    m.h = m.main.Header();
    if (!m.h || m.h->id != kIdStudioHeader)
        return std::printf("read error: \"%s\" is not a studio model\n", in.c_str()), 1;
    if (m.h->version != 10)
        std::printf("version %d is read with the version 10 layout\n", m.h->version);

    std::printf("model:       \"%s\"\n", Fixed(m.h->name).c_str());
    std::printf("contents:    %d bones, %d bodyparts, %d sequences (%d groups),\n"
                "             %d attachments, %d hitboxes, %d bone controllers\n",
                m.h->numbones, m.h->numbodyparts, m.h->numseq, m.h->numseqgroups,
                m.h->numattachments, m.h->numhitboxes, m.h->numbonecontrollers);

    LoadSiblings(m, in);
    BuildBindPose(m);
    const std::vector<std::string> materials = MaterialNames(m);

    std::FILE* out = std::fopen(outPath.c_str(), "wb");
    if (!out)
        return std::printf("write error: cannot write \"%s\"\n", outPath.c_str()), 1;
    Qc q{out};
    q.Line("// mdldecompiler - GoldSrc model version " + std::to_string(m.h->version));
    q.Line("// " + in);
    q.Blank();

    const std::string base = BaseName(StripExt(in));
    std::string name = Fixed(m.h->name);
    if (name.empty())
        name = base + ".mdl";
    q.Line("$modelname \"" + name + "\"");
    q.Line("$cdmaterials \"models/" + base + "\"");
    if (m.h->min.x || m.h->min.y || m.h->min.z || m.h->max.x || m.h->max.y || m.h->max.z)
        q.Line("$bbox " + V3(m.h->min) + "  " + V3(m.h->max));
    if (m.h->bbmin.x || m.h->bbmin.y || m.h->bbmin.z || m.h->bbmax.x || m.h->bbmax.y ||
        m.h->bbmax.z)
        q.Line("$cbox " + V3(m.h->bbmin) + "  " + V3(m.h->bbmax));
    q.Line("$eyeposition " + V3(Unswizzle(m.h->eyeposition)));
    if (m.h->flags) {
        char hex[16];
        std::snprintf(hex, sizeof hex, "0x%X", static_cast<unsigned>(m.h->flags));
        q.Line(std::string("$flags ") + hex);
    }

    // --- meshes -------------------------------------------------------------
    std::printf("\nmeshes:\n");
    const bodypart_t* parts = m.main.At<bodypart_t>(m.h->bodypartindex, m.h->numbodyparts);
    std::vector<std::vector<std::string>> meshNames(static_cast<size_t>(
        std::max(m.h->numbodyparts, 0)));
    std::set<std::string> used;
    for (int i = 0; parts && i < m.h->numbodyparts; ++i) {
        const model_t* models = m.main.At<model_t>(parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            const std::string raw = Sanitize(StripExt(Fixed(models[j].name)));
            // "blank" is how a bodygroup spells an empty choice
            if (models[j].nummesh <= 0 || _stricmp(raw.c_str(), "blank") == 0) {
                meshNames[i].push_back(std::string());
                continue;
            }
            const std::string mesh = UniqueName(used, raw);
            meshNames[i].push_back(mesh);
            WriteMeshSmd(m, models[j], (fs::path(dir) / "meshes" / (mesh + ".smd")).string(),
                         materials);
            std::printf("  %s.smd (%d verts, %d meshes)\n", mesh.c_str(), models[j].numverts,
                        models[j].nummesh);
        }
    }

    q.Blank();
    for (const std::vector<std::string>& part : meshNames)
        for (const std::string& n : part)
            if (!n.empty())
                q.Line("$rendermesh \"" + n + "\" \"meshes/" + n + ".smd\"");
    for (int i = 0; parts && i < m.h->numbodyparts; ++i) {
        q.Blank();
        q.Line("$modelgroup \"" + Sanitize(Fixed(parts[i].name)) + "\" {");
        for (const std::string& n : meshNames[i])
            q.Line(n.empty() ? "    blank" : "    mesh name \"" + n + "\" \"" + n + "\"");
        q.Line("}");
    }

    // --- skins --------------------------------------------------------------
    // one $set per family past family 0, listing only the slots that differ
    if (m.texHdr->numskinfamilies > 1) {
        auto mat = [&](int t) {
            return (t >= 0 && t < static_cast<int>(materials.size())) ? materials[t]
                                                                     : std::string();
        };
        q.Blank();
        q.Line("$texturegroup {");
        for (int fam = 1; fam < m.texHdr->numskinfamilies; ++fam) {
            q.Line("    $set {");
            for (int r = 0; r < m.texHdr->numskinref; ++r) {
                const int base = SkinTexture(m, 0, r), cur = SkinTexture(m, fam, r);
                if (base != cur)
                    q.Line("        material \"" + mat(base) + "\" \"" + mat(cur) + "\"");
            }
            q.Line("    }");
        }
        q.Line("}");
    }

    // --- attachments / hitboxes --------------------------------------------
    const attachment_t* atts = m.main.At<attachment_t>(m.h->attachmentindex, m.h->numattachments);
    if (atts && m.h->numattachments > 0)
        q.Blank();
    for (int i = 0; atts && i < m.h->numattachments; ++i) {
        std::string an = Sanitize(Fixed(atts[i].name));
        if (an.empty())
            an = std::to_string(i); // GoldSrc code addresses attachments by index
        const size_t b = static_cast<size_t>(atts[i].bone);
        q.Line("$attachment \"" + an + "\" \"" +
               (b < m.boneNames.size() ? m.boneNames[b] : std::string()) + "\" origin " +
               V3(atts[i].org));
    }

    const bbox_t* boxes = m.main.At<bbox_t>(m.h->hitboxindex, m.h->numhitboxes);
    if (boxes && m.h->numhitboxes > 0) {
        q.Blank();
        q.Line("$hboxset \"default\" {");
        for (int i = 0; i < m.h->numhitboxes; ++i) {
            const size_t b = static_cast<size_t>(boxes[i].bone);
            q.Line("    $hbox " + std::to_string(boxes[i].group) + " \"" +
                   (b < m.boneNames.size() ? m.boneNames[b] : std::string()) + "\" " +
                   V3(boxes[i].bbmin) + "  " + V3(boxes[i].bbmax));
        }
        q.Line("}");
    }

    const bonecontroller_t* ctrls =
        m.main.At<bonecontroller_t>(m.h->bonecontrollerindex, m.h->numbonecontrollers);
    for (int i = 0; ctrls && i < m.h->numbonecontrollers; ++i) {
        const size_t b = static_cast<size_t>(ctrls[i].bone);
        q.Line("$controller " + std::to_string(ctrls[i].index) + " \"" +
               (b < m.boneNames.size() ? m.boneNames[b] : std::string()) + "\" " +
               BlendAxis(ctrls[i].type) + " " + F(ctrls[i].start) + " " + F(ctrls[i].end));
    }

    // --- sequences ----------------------------------------------------------
    const seqdesc_t* seqs = m.main.At<seqdesc_t>(m.h->seqindex, m.h->numseq);
    if (seqs && m.h->numseq > 0)
        std::printf("\nanimations:\n");

    // one pose parameter per blend axis, spanning every range that uses it
    std::map<std::string, std::pair<float, float>> poses;
    for (int i = 0; seqs && i < m.h->numseq; ++i)
        for (int k = 0; k < 2 && k < seqs[i].numblends; ++k) {
            if (!seqs[i].blendtype[k])
                continue;
            auto& r = poses[BlendAxis(seqs[i].blendtype[k])];
            r.first = std::min(r.first, seqs[i].blendstart[k]);
            r.second = std::max(r.second, seqs[i].blendend[k]);
        }
    if (!poses.empty())
        q.Blank();
    for (const auto& p : poses)
        q.Line("$poseparameter \"" + p.first + "\" " + F(p.second.first) + " " +
               F(p.second.second));

    std::set<std::string> seqUsed;
    for (int i = 0; seqs && i < m.h->numseq; ++i) {
        const seqdesc_t& s = seqs[i];
        std::string label = UniqueName(seqUsed, Sanitize(Fixed(s.label)));
        const int blends = std::max(s.numblends, 1);
        for (int b = 0; b < blends; ++b) {
            const std::string clip = blends > 1 ? label + "_blend" + std::to_string(b) : label;
            WriteAnimSmd(m, s, b, (fs::path(dir) / "anims" / (clip + ".smd")).string());
        }
        std::printf("  %s (%d frames%s)\n", label.c_str(), s.numframes,
                    blends > 1 ? ", blended" : "");

        q.Blank();
        q.Line("$sequence \"" + label + "\" {");
        for (int b = 0; b < blends; ++b)
            q.Line("    \"anims/" + (blends > 1 ? label + "_blend" + std::to_string(b) : label) +
                   ".smd\"");
        q.Line("    fps " + F(s.fps));
        if (s.flags & kStudioLooping)
            q.Line("    loop");
        if (blends > 1) {
            q.Line("    blendwidth " + std::to_string(blends));
            for (int k = 0; k < 2 && k < s.numblends; ++k)
                if (s.blendtype[k])
                    q.Line("    blend \"" + std::string(BlendAxis(s.blendtype[k])) + "\" " +
                           F(s.blendstart[k]) + " " + F(s.blendend[k]));
        }
        // the activity stays a number: GoldSrc's table is not Source's, so the
        // name it was written with cannot be recovered
        if (s.activity)
            q.Line("    activity " + std::to_string(s.activity) + " " +
                   std::to_string(s.actweight));
        if (s.entrynode || s.exitnode) {
            if (s.entrynode == s.exitnode)
                q.Line("    node \"node" + std::to_string(s.entrynode) + "\"");
            else
                q.Line(std::string("    ") + (s.nodeflags & 1 ? "rtransition" : "transition") +
                       " \"node" + std::to_string(s.entrynode) + "\" \"node" +
                       std::to_string(s.exitnode) + "\"");
        }
        const event_t* events = m.main.At<event_t>(s.eventindex, s.numevents);
        for (int e = 0; events && e < s.numevents; ++e) {
            std::string line = "    event " + std::to_string(events[e].event) + " " +
                               std::to_string(events[e].frame);
            const std::string opt = Fixed(events[e].options);
            if (!opt.empty() && CleanName(opt))
                line += " \"" + opt + "\"";
            q.Line(line);
        }
        if (s.motiontype)
            q.Line("    motion " + std::string(BlendAxis(s.motiontype)) + " " +
                   V3(s.linearmovement));
        q.Line("}");
    }
    std::fclose(out);

    // --- textures -----------------------------------------------------------
    const texture_t* tex = m.tex->At<texture_t>(m.texHdr->textureindex, m.texHdr->numtextures);
    if (tex && m.texHdr->numtextures > 0)
        std::printf("\ntextures:\n");
    for (int i = 0; tex && i < m.texHdr->numtextures; ++i) {
        WriteBmp(m, tex[i], (fs::path(dir) / "materials" / (materials[i] + ".bmp")).string());
        std::printf("  %s.bmp (%dx%d)\n", materials[i].c_str(), tex[i].width, tex[i].height);
    }

    std::printf("\nwrote %s\n", outPath.c_str());
    return 0;
}

} // namespace

bool IsGoldSrcMdl(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    int32_t head[2] = {0, 0};
    const size_t got = std::fread(head, sizeof(int32_t), 2, f);
    std::fclose(f);
    return got == 2 && head[0] == kIdStudioHeader && head[1] < 44;
}

int DecompileGoldSrc(const std::string& in, const std::string& dir, const std::string& outPath) {
    return Decompile(in, dir, outPath);
}

} // namespace mdldecompiler
