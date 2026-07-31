// smdloader.cpp - PulseMDL
//
// See smdloader.h. Behavior port of the reference studiomdl SMD reader
// (Load_SMD/Grab_Triangles, Grab_Nodes/Grab_Animation/Build_Reference).
// Studied for behavior, re-typed clean.

#include "smdloader.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/math.h"
#include "pulselimits.h"

namespace pm = pulse::math;
namespace lim = pulse::limits;

namespace pulse::source {

// reference normal_blend default = cos(DEG2RAD(2.0)); studiomdl's -a flag can
// change it, but the default is what decompiled assets are built with.
static const float kNormalBlend = std::cos(2.0f * pm::kDeg2Rad);

namespace {

// ---------------------------------------------------------------------------
// Line reader - the reference reads with fgets + GetLineInput, which skips only
// `//` comment lines and counts every physical line. We mirror both: lineNo
// advances on skipped comment lines too, so error line numbers match.
// ---------------------------------------------------------------------------
struct LineReader {
    std::vector<std::string> lines;
    size_t idx = 0;
    int lineNo = 0;

    // Next non-comment line, or nullptr at EOF. Returns a pointer stable until
    // the next call is irrelevant - we return by value-ish const ref into the
    // vector, which never reallocates after load.
    const std::string* Next() {
        while (idx < lines.size()) {
            const std::string& l = lines[idx++];
            ++lineNo;
            if (l.size() >= 2 && l[0] == '/' && l[1] == '/')
                continue; // reference GetLineInput skips "//" lines
            return &l;
        }
        return nullptr;
    }
};

bool ReadAllLines(const std::string& path, LineReader& r, std::string* err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (err) *err = "cannot open \"" + path + "\"";
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string all = ss.str();

    // strip a leading UTF-8 BOM if present
    if (all.size() >= 3 && static_cast<unsigned char>(all[0]) == 0xEF &&
        static_cast<unsigned char>(all[1]) == 0xBB && static_cast<unsigned char>(all[2]) == 0xBF)
        all.erase(0, 3);

    std::string line;
    std::istringstream ls(all);
    while (std::getline(ls, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        r.lines.push_back(std::move(line));
        line.clear();
    }
    return true;
}

// ---------------------------------------------------------------------------
// nodes (reference Grab_Nodes): lines `index "name" parent`. numbones is the
// highest index + 1; a gap keeps parent -1 (never created as a joint). No
// trailing "defaultRoot" - that is a DMX-loader autoskin artifact the SMD path
// does not have. A non-matching line ends the block (and is consumed).
// ---------------------------------------------------------------------------
bool GrabNodes(LineReader& r, Source& out, std::string* err) {
    std::vector<LocalBone> bones;
    int maxIndex = -1;

    for (;;) {
        const std::string* line = r.Next();
        if (!line) {
            if (err) *err = "unexpected EOF in nodes block";
            return false;
        }
        int index = 0, parent = 0;
        char name[1024] = {0};
        if (std::sscanf(line->c_str(), "%d \"%1023[^\"]\" %d", &index, name, &parent) == 3) {
            if (index < 0) {
                if (err) *err = "negative bone index in nodes block";
                return false;
            }
            if (index >= static_cast<int>(bones.size()))
                bones.resize(index + 1); // gaps default to parent -1
            bones[index].name = name;
            bones[index].parent = parent;
            if (index > maxIndex)
                maxIndex = index;
            continue;
        }
        break; // end of block (the "end" line, or anything non-matching)
    }

    out.numbones = maxIndex + 1;
    if (out.numbones <= 0) {
        if (err) *err = "nodes block declared no bones";
        return false;
    }
    bones.resize(out.numbones);
    out.localBone = std::move(bones);
    return true;
}

// ---------------------------------------------------------------------------
// skeleton (reference Grab_Animation, always into "BindPose"): `time N` blocks,
// each a set of `boneindex px py pz rx ry rz` lines. Positions are scaled;
// rotations are stored as the raw authored radian Euler (the reference's
// clip_rotations acts on a dead local copy - downstream code clips). A `time`
// block need only list changed bones; missing ones inherit the previous frame.
// Build_Reference then folds the frame-0 poses into boneToPose.
// ---------------------------------------------------------------------------
bool GrabAnimation(LineReader& r, float scale, Source& out, std::string* err) {
    const int numbones = out.numbones;

    std::vector<std::vector<SrcBonePose>> frames;
    std::vector<char> frameSet;
    int startframe = -1;
    int endframe = 0;
    int curT = -1; // current local frame index

    for (;;) {
        const std::string* line = r.Next();
        if (!line) {
            if (err) *err = "unexpected EOF in skeleton block";
            return false;
        }

        int index = 0;
        float p0, p1, p2, e0, e1, e2;
        if (std::sscanf(line->c_str(), "%d %f %f %f %f %f %f", &index, &p0, &p1, &p2, &e0, &e1,
                        &e2) == 7) {
            if (startframe < 0) {
                if (err) *err = "bone data before the first `time` in skeleton block";
                return false;
            }
            if (index < 0 || index >= numbones) {
                if (err) *err = "bogus bone index in skeleton block";
                return false;
            }
            SrcBonePose& pose = frames[curT][index];
            pose.pos = pm::Vector3{p0 * scale, p1 * scale, p2 * scale};
            pose.rot = pm::RadianEuler{e0, e1, e2};
            continue;
        }

        char cmd[1024] = {0};
        int idx2 = 0;
        int n = std::sscanf(line->c_str(), "%1023s %d", cmd, &idx2);
        if (n <= 0) {
            if (err) *err = "malformed line in skeleton block";
            return false;
        }

        if (_stricmp(cmd, "time") == 0) {
            const int t = idx2;
            if (startframe == -1)
                startframe = t;
            if (t < startframe) {
                if (err) *err = "frame time went backwards in skeleton block";
                return false;
            }
            if (t > endframe)
                endframe = t;

            const int tl = t - startframe;
            curT = tl;
            if (tl >= static_cast<int>(frames.size())) {
                frames.resize(tl + 1);       // gap frames stay unset
                frameSet.resize(tl + 1, 0);
            }
            if (frameSet[tl])
                continue; // frame already present: keep filling it (reference)

            frames[tl].assign(numbones, SrcBonePose{});
            frameSet[tl] = 1;
            if (tl > 0 && frameSet[tl - 1])
                frames[tl] = frames[tl - 1]; // duplicate previous frame's keys
            continue;
        }

        if (_stricmp(cmd, "end") == 0)
            break;

        if (err) *err = "unknown token \"" + std::string(cmd) + "\" in skeleton block";
        return false;
    }

    if (startframe < 0) {
        if (err) *err = "skeleton block has no frames";
        return false;
    }

    const int numframes = endframe - startframe + 1;
    for (int t = 0; t < numframes; ++t) {
        if (t >= static_cast<int>(frameSet.size()) || !frameSet[t]) {
            if (err) *err = "skeleton block is missing frame " + std::to_string(t + startframe);
            return false;
        }
    }

    SourceAnim anim;
    anim.name = "BindPose";
    anim.startframe = startframe;
    anim.endframe = endframe;
    anim.numframes = numframes;
    anim.frames = std::move(frames);
    anim.frames.resize(numframes); // drop any trailing gap slots
    out.anims.push_back(std::move(anim));

    // Build_Reference: source-local boneToPose from the BindPose frame 0.
    const SourceAnim& bp = out.anims.back();
    out.boneToPose.resize(numbones);
    for (int i = 0; i < numbones; ++i) {
        pm::matrix3x4 m;
        pm::AngleMatrix(bp.frames[0][i].rot, bp.frames[0][i].pos, m);
        const int parent = out.localBone[i].parent;
        if (parent == -1)
            out.boneToPose[i] = m;
        else
            out.boneToPose[i] = pm::ConcatTransforms(out.boneToPose[parent], m);
    }
    return true;
}

// ---------------------------------------------------------------------------
// triangles (reference Grab_Triangles/ParseFaceData/lookup_index). Unlike the
// DMX loader's exact-index unify, SMD merges corners that share (material, exact
// pos, exact uv0) AND whose normals dot above kNormalBlend AND whose bone
// weights match. `lastref` is set on every touch and drives the material sort.
// The post-unify sort/build mirrors the shared reference BuildIndividualMeshes.
// ---------------------------------------------------------------------------

// one unified vertex + its dedup-chain bookkeeping (reference v_unify_t +
// g_StudioMdlContext streams, folded together since SMD stores per unique vert)
struct UVert {
    pm::Vector3 pos;
    pm::Vector3 normal;
    pm::Vector2 tc;
    SrcBoneWeight bone;
    int m = 0;        // material
    int lastref = 0;  // reference v_unify_t.lastref (last g_numvlist that touched it)
    int next = -1;    // next unique vert in the same hash bucket
};

struct TriFace {
    int material = 0;
    uint32_t a = 0, b = 0, c = 0; // unique-vert indices (model relative)
};

struct TriState {
    std::vector<UVert> verts;
    std::vector<TriFace> faces;
    // hash bucket head keyed by (material, pos bits, uv0 bits) - reference
    // SmdVertKey/s_smdVertMap; the chain walk does the fuzzy/weight compare.
    struct Key {
        int mat;
        uint32_t px, py, pz, u, v;
        bool operator==(const Key& o) const {
            return mat == o.mat && px == o.px && py == o.py && pz == o.pz && u == o.u && v == o.v;
        }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            uint64_t h = 14695981039346656037ULL;
            auto eat = [&](uint32_t x) { h = (h ^ static_cast<uint64_t>(x)) * 1099511628211ULL; };
            eat(static_cast<uint32_t>(k.mat));
            eat(k.px); eat(k.py); eat(k.pz); eat(k.u); eat(k.v);
            return static_cast<size_t>(h);
        }
    };
    std::unordered_map<Key, int, KeyHash> map;
};

uint32_t Bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

// reference lookup_index: find-or-add a unique vertex.
int LookupIndex(TriState& st, int material, const pm::Vector3& pos, const pm::Vector3& normal,
                const pm::Vector2& tc, const SrcBoneWeight& bone) {
    TriState::Key key{material, Bits(pos.x), Bits(pos.y), Bits(pos.z), Bits(tc.x), Bits(tc.y)};
    const int numvlist = static_cast<int>(st.verts.size());

    auto it = st.map.find(key);
    if (it != st.map.end()) {
        for (int i = it->second; i >= 0; i = st.verts[i].next) {
            const UVert& u = st.verts[i];
            const float dot = u.normal.x * normal.x + u.normal.y * normal.y + u.normal.z * normal.z;
            if (dot <= kNormalBlend)
                continue;
            if (u.bone.numbones != bone.numbones)
                continue;
            bool same = true;
            for (int j = 0; j < bone.numbones; ++j)
                if (u.bone.bone[j] != bone.bone[j] || u.bone.weight[j] != bone.weight[j]) {
                    same = false;
                    break;
                }
            if (!same)
                continue;
            st.verts[i].lastref = numvlist; // reference: set on re-reference
            return i;
        }
    }

    // new vertex
    UVert nv;
    nv.pos = pos;
    nv.normal = normal;
    nv.tc = tc;
    nv.bone = bone;
    nv.m = material;
    nv.lastref = numvlist; // == this vertex's own index (pre-grow)
    if (it != st.map.end()) {
        nv.next = it->second; // prepend to the existing bucket chain
        st.verts.push_back(nv);
        it->second = numvlist;
    } else {
        nv.next = -1;
        st.verts.push_back(nv);
        st.map.emplace(key, numvlist);
    }
    return numvlist;
}

// reference ParseFaceData: read the 3 corner lines of one triangle.
bool ParseFaceData(LineReader& r, int numbones, int version, float scale, int material,
                   TriState& st, TriFace& out, std::string* err) {
    uint32_t idx[3];
    for (int j = 0; j < 3; ++j) {
        const std::string* line = r.Next();
        if (!line) {
            if (err) *err = "unexpected EOF reading a triangle";
            return false;
        }
        const char* p = line->c_str();
        char* end = nullptr;

        long bone = std::strtol(p, &end, 10);
        p = end;
        pm::Vector3 pos, nrm;
        pos.x = std::strtof(p, &end); p = end;
        pos.y = std::strtof(p, &end); p = end;
        pos.z = std::strtof(p, &end); p = end;
        nrm.x = std::strtof(p, &end); p = end;
        nrm.y = std::strtof(p, &end); p = end;
        nrm.z = std::strtof(p, &end); p = end;
        pm::Vector2 tc;
        tc.x = std::strtof(p, &end); p = end;
        tc.y = std::strtof(p, &end); p = end;

        if (bone < 0 || bone >= numbones) {
            if (err) *err = "bogus bone index in a triangle vertex";
            return false;
        }

        pos.x *= scale; pos.y *= scale; pos.z *= scale;

        int iCount = 0;
        int bones[lim::kMaxSrcBoneWeights];
        float weights[lim::kMaxSrcBoneWeights];

        // optional explicit bone weight list
        const char* q = p;
        while (*q == ' ' || *q == '\t') ++q;
        if (*q && *q != '\n' && *q != '\r') {
            iCount = static_cast<int>(std::strtol(p, &end, 10));
            p = end;
            const int actual = iCount < lim::kMaxSrcBoneWeights ? iCount : lim::kMaxSrcBoneWeights;
            for (int k = 0; k < actual; ++k) {
                bones[k] = static_cast<int>(std::strtol(p, &end, 10)); p = end;
                weights[k] = std::strtof(p, &end); p = end;
            }
            // SMD v3 extra texcoords: parsed to advance, ignored (single UV).
            if (version >= 3) {
                const char* e = p;
                while (*e == ' ' || *e == '\t') ++e;
                if (*e && *e != '\n' && *e != '\r') {
                    int iExtras = static_cast<int>(std::strtol(p, &end, 10)); p = end;
                    for (int k = 0; k < iExtras; ++k) { std::strtof(p, &end); p = end; }
                }
            }
        }

        tc.y = 1.0f - tc.y; // invert V

        SrcBoneWeight bw;
        if (iCount == 0) {
            bw.numbones = 1;
            bw.bone[0] = static_cast<int>(bone);
            bw.weight[0] = 1.0f;
        } else {
            bw.numbones = SortAndBalanceBones(iCount, lim::kMaxSrcBoneWeights, bones, weights);
            for (int k = 0; k < bw.numbones; ++k) {
                bw.bone[k] = bones[k];
                bw.weight[k] = weights[k];
            }
        }

        idx[j] = static_cast<uint32_t>(LookupIndex(st, material, pos, nrm, tc, bw));
    }

    out.material = material;
    out.a = idx[0];
    out.b = idx[2]; // reference winding: a, c, b
    out.c = idx[1];
    return true;
}

// strip trailing non-graph chars (reference isgraph trim of the texture line)
std::string TrimTextureName(const std::string& line) {
    int i = static_cast<int>(line.size()) - 1;
    while (i >= 0) {
        unsigned char ch = static_cast<unsigned char>(line[i]);
        if (ch > ' ' && ch != 127) break; // isgraph
        --i;
    }
    return line.substr(0, i + 1);
}

// qsort context (reference uses libc qsort with globals; match it for the same
// unstable ordering the DMX path relies on)
const std::vector<UVert>* g_sortVerts = nullptr;
const std::vector<TriFace>* g_sortFaces = nullptr;

int VlistCompare(const void* a, const void* b) {
    const UVert& u1 = (*g_sortVerts)[*static_cast<const int*>(a)];
    const UVert& u2 = (*g_sortVerts)[*static_cast<const int*>(b)];
    if (u1.m < u2.m) return -1;
    if (u1.m > u2.m) return 1;
    if (u1.lastref < u2.lastref) return -1;
    if (u1.lastref > u2.lastref) return 1;
    return 0;
}
int FaceCompare(const void* a, const void* b) {
    int i1 = *static_cast<const int*>(a), i2 = *static_cast<const int*>(b);
    if ((*g_sortFaces)[i1].material < (*g_sortFaces)[i2].material) return -1;
    if ((*g_sortFaces)[i1].material > (*g_sortFaces)[i2].material) return 1;
    if (i1 < i2) return -1;
    if (i1 > i2) return 1;
    return 0;
}

// same block terminator GrabTriangles uses
bool IsEndLine(const std::string& line) {
    return line.compare(0, 3, "end") == 0 &&
           (line.size() == 3 || line[3] == '\r' || line[3] == ' ');
}

// swallow a block an animation reference has no use for, so its contents are
// not read back as commands
void SkipBlock(LineReader& r) {
    for (;;) {
        const std::string* line = r.Next();
        if (!line || IsEndLine(*line))
            return;
    }
}

bool GrabTriangles(LineReader& r, int version, float scale, MaterialTable& mats, Source& out,
                   std::string* err) {
    TriState st;

    for (;;) {
        const std::string* line = r.Next();
        if (!line)
            break; // reference: loop ends on EOF too
        if (IsEndLine(*line))
            break;

        std::string tex = TrimTextureName(*line);
        if (tex.empty()) {
            for (int k = 0; k < 3; ++k) r.Next(); // skip the 3 vertex lines
            continue;
        }
        if (_stricmp(tex.c_str(), "null.bmp") == 0 || _stricmp(tex.c_str(), "null.tga") == 0 ||
            _stricmp(tex.c_str(), "debug/debugempty") == 0) {
            for (int k = 0; k < 3; ++k) r.Next();
            continue;
        }

        // reference LookupTexture(name, version==2): v1/v3 non-relative.
        const int texture = mats.LookupTexture(tex.c_str(), version == 2);
        const int material = mats.UseTextureAsMaterial(texture);

        TriFace f;
        if (!ParseFaceData(r, out.numbones, version, scale, material, st, f, err))
            return false;

        if (f.a == f.b || f.b == f.c || f.a == f.c)
            continue; // degenerate

        st.faces.push_back(f);
    }

    // ---- sort + build (reference BuildIndividualMeshes) --------------------
    const int numvlist = static_cast<int>(st.verts.size());
    const int numfaces = static_cast<int>(st.faces.size());

    std::vector<int> v_listsort(numvlist), v_ilistsort(numvlist), facesort(numfaces);
    for (int i = 0; i < numvlist; ++i) v_listsort[i] = i;
    for (int i = 0; i < numfaces; ++i) facesort[i] = i;
    g_sortVerts = &st.verts;
    g_sortFaces = &st.faces;
    if (numvlist > 0) std::qsort(v_listsort.data(), numvlist, sizeof(int), VlistCompare);
    if (numfaces > 0) std::qsort(facesort.data(), numfaces, sizeof(int), FaceCompare);
    g_sortVerts = nullptr;
    g_sortFaces = nullptr;
    for (int i = 0; i < numvlist; ++i) v_ilistsort[v_listsort[i]] = i;

    // BuildUniqueVertexList
    out.vertex.resize(numvlist);
    for (int i = 0; i < numvlist; ++i) {
        const UVert& u = st.verts[v_listsort[i]];
        SrcVertex& v = out.vertex[i];
        v.position = u.pos;
        v.normal = u.normal;
        v.texcoord = u.tc;
        v.boneweight = u.bone;
        v.material = u.m;
    }

    // PointMeshesToVertexAndFaceData
    out.mesh.assign(lim::kMaxSkins, SrcMesh{});
    out.meshindex.assign(lim::kMaxSkins, 0);
    for (int m = 0; m < lim::kMaxSkins; ++m) {
        out.mesh[m].vertexoffset = numvlist;
        out.mesh[m].faceoffset = numfaces;
    }
    for (int i = 0; i < numvlist; ++i) {
        int m = out.vertex[i].material;
        out.mesh[m].numvertices++;
        if (out.mesh[m].vertexoffset > i) out.mesh[m].vertexoffset = i;
    }
    for (int i = 0; i < numfaces; ++i) {
        int m = st.faces[facesort[i]].material;
        out.mesh[m].numfaces++;
        if (out.mesh[m].faceoffset > i) out.mesh[m].faceoffset = i;
    }

    // BuildFaceList
    out.face.resize(numfaces);
    out.nummeshes = 0;
    for (int m = 0; m < lim::kMaxSkins; ++m) {
        if (!out.mesh[m].numfaces)
            continue;
        out.meshindex[out.nummeshes++] = m;
        for (int i = out.mesh[m].faceoffset; i < out.mesh[m].numfaces + out.mesh[m].faceoffset;
             ++i) {
            const TriFace& sf = st.faces[facesort[i]];
            out.face[i].a = static_cast<uint32_t>(v_ilistsort[sf.a] - out.mesh[m].vertexoffset);
            out.face[i].b = static_cast<uint32_t>(v_ilistsort[sf.b] - out.mesh[m].vertexoffset);
            out.face[i].c = static_cast<uint32_t>(v_ilistsort[sf.c] - out.mesh[m].vertexoffset);
        }
    }

    CalcModelTangentSpaces(out);
    return true;
}

} // namespace

bool LoadSmdSource(const std::string& path, Source& out, MaterialTable& mats, float scale,
                   std::string* err, bool /*morphSource*/, bool animOnly) {
    LineReader r;
    if (!ReadAllLines(path, r, err))
        return false;

    bool sawNodes = false;
    int version = 1; // reference Load_SMD default

    for (;;) {
        const std::string* line = r.Next();
        if (!line)
            break;

        char cmd[1024] = {0};
        int option = 0;
        const int numRead = std::sscanf(line->c_str(), "%1023s %d", cmd, &option);
        if (numRead <= 0)
            continue; // blank line

        if (_stricmp(cmd, "version") == 0) {
            if (numRead < 2 || option < 1 || option > 3) {
                if (err) *err = "bad SMD version (expected 1-3)";
                return false;
            }
            version = option;
        } else if (_stricmp(cmd, "nodes") == 0) {
            if (!GrabNodes(r, out, err))
                return false;
            sawNodes = true;
        } else if (_stricmp(cmd, "skeleton") == 0) {
            if (!sawNodes) {
                if (err) *err = "`skeleton` before `nodes` in SMD";
                return false;
            }
            if (!GrabAnimation(r, scale, out, err))
                return false;
        } else if (_stricmp(cmd, "triangles") == 0) {
            if (!sawNodes) {
                if (err) *err = "`triangles` before `nodes` in SMD";
                return false;
            }
            if (animOnly) {
                SkipBlock(r); // geometry is model data - see LoadDmxSource
                continue;
            }
            if (!GrabTriangles(r, version, scale, mats, out, err))
                return false;
        } else if (_stricmp(cmd, "vertexanimation") == 0) {
            SkipBlock(r); // morphs arrive through $vta -> LoadVtaMorphs, not here
        } else if (cmd[0] == '/' || cmd[0] == ';' || cmd[0] == '#') {
            continue; // source comment (reference ProcessSourceComment is a no-op)
        } else {
            // reference only warns; we accept-and-ignore unknown commands to
            // stay tolerant of exporter noise, matching that leniency.
            continue;
        }
    }

    if (!sawNodes) {
        if (err) *err = "SMD has no `nodes` block";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// VTA morphs. Parse: reference Grab_Vertexanimation. The
// frames hold ABSOLUTE positions against the VTA's OWN vertex numbering, which
// is the pre-unify export order and so does not line up with the mesh's; frame 0
// is the basis and every model vertex is matched to it spatially (reference
// BuildModelToVAnimMap / RemapVertexAnimations).
// ---------------------------------------------------------------------------
namespace {

const float kMaxVAnimDist = 0.3873f; // reference MAX_VANIM_DIST

struct VtaVert {
    pm::Vector3 pos;
    pm::Vector3 normal;
};

struct VtaFrame {
    int time = 0;
    std::string comment; // whatever follows `#` on the `time` line - a HINT for
                         // the listing we print, never an identifier
    std::vector<std::pair<int, VtaVert>> verts;
};

std::string FrameComment(const std::string& line) {
    const size_t h = line.find('#');
    if (h == std::string::npos)
        return {};
    const size_t b = line.find_first_not_of(" \t", h + 1);
    if (b == std::string::npos)
        return {};
    const size_t e = line.find_last_not_of(" \t\r");
    return line.substr(b, e - b + 1);
}

bool GrabVertexAnimation(LineReader& r, float scale, std::vector<VtaFrame>& out, std::string* err) {
    for (;;) {
        const std::string* line = r.Next();
        if (!line) {
            if (err) *err = "unexpected EOF in vertexanimation block";
            return false;
        }

        int index = 0;
        VtaVert v;
        if (std::sscanf(line->c_str(), "%d %f %f %f %f %f %f", &index, &v.pos.x, &v.pos.y, &v.pos.z,
                        &v.normal.x, &v.normal.y, &v.normal.z) == 7) {
            if (out.empty()) {
                if (err) *err = "vertex data before the first `time` in vertexanimation block";
                return false;
            }
            if (index < 0) {
                if (err) *err = "negative vertex index in vertexanimation block";
                return false;
            }
            v.pos.x *= scale; v.pos.y *= scale; v.pos.z *= scale;
            out.back().verts.push_back({index, v});
            continue;
        }

        char cmd[1024] = {0};
        int t = 0;
        const int n = std::sscanf(line->c_str(), "%1023s %d", cmd, &t);
        if (n <= 0) {
            if (err) *err = "malformed line in vertexanimation block";
            return false;
        }
        if (_stricmp(cmd, "end") == 0)
            return true;
        if (n < 2 || _stricmp(cmd, "time") != 0) {
            if (err) *err = "unknown token \"" + std::string(cmd) + "\" in vertexanimation block";
            return false;
        }
        VtaFrame f;
        f.time = t;
        f.comment = FrameComment(*line);
        out.push_back(std::move(f));
    }
}

// Nearest basis vertex per model vertex within kMaxVAnimDist; ties go to the
// bigger normal dot, then to the lower basis index. The reference walks a sphere
// tree with a shrinking radius - a uniform grid over the (much smaller) model
// vertex list is the same answer without the shrink's order sensitivity.
void BuildModelToVAnim(const Source& src, const std::vector<VtaVert>& base,
                       std::vector<int>& modelToVAnim) {
    modelToVAnim.assign(src.vertex.size(), -1);
    std::vector<float> bestDist(src.vertex.size(), 1e30f);
    std::vector<float> bestDot(src.vertex.size(), -1.0f);

    auto cell = [](float x) { return static_cast<int64_t>(std::floor(x / kMaxVAnimDist)); };
    // cell hash; distinct cells may collide, which only adds candidates the
    // distance test below rejects
    auto key = [](int64_t x, int64_t y, int64_t z) {
        return static_cast<uint64_t>(x) * 73856093ULL ^ static_cast<uint64_t>(y) * 19349663ULL ^
               static_cast<uint64_t>(z) * 83492791ULL;
    };

    std::unordered_map<uint64_t, std::vector<int>> grid;
    for (size_t i = 0; i < src.vertex.size(); ++i) {
        const pm::Vector3& p = src.vertex[i].position;
        grid[key(cell(p.x), cell(p.y), cell(p.z))].push_back(static_cast<int>(i));
    }

    const float maxSqr = kMaxVAnimDist * kMaxVAnimDist;
    for (size_t j = 0; j < base.size(); ++j) {
        const pm::Vector3& p = base[j].pos;
        const int64_t cx = cell(p.x), cy = cell(p.y), cz = cell(p.z);
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dy = -1; dy <= 1; ++dy)
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    auto it = grid.find(key(cx + dx, cy + dy, cz + dz));
                    if (it == grid.end())
                        continue;
                    for (int m : it->second) {
                        const SrcVertex& mv = src.vertex[m];
                        const float ex = mv.position.x - p.x, ey = mv.position.y - p.y,
                                    ez = mv.position.z - p.z;
                        const float d = ex * ex + ey * ey + ez * ez;
                        if (d > maxSqr)
                            continue;
                        const float dot = mv.normal.x * base[j].normal.x +
                                          mv.normal.y * base[j].normal.y +
                                          mv.normal.z * base[j].normal.z;
                        if (d < bestDist[m] || (d == bestDist[m] && dot > bestDot[m])) {
                            bestDist[m] = d;
                            bestDot[m] = dot;
                            modelToVAnim[m] = static_cast<int>(j);
                        }
                    }
                }
    }
}

// reference ComputeVertexAnimationSpeed
void ComputeVertexAnimationSpeed(std::vector<SrcVertAnim>& vanims, float decay) {
    float maxLen = 0.0f;
    for (const SrcVertAnim& va : vanims) {
        const float s = std::sqrt(va.pos.x * va.pos.x + va.pos.y * va.pos.y + va.pos.z * va.pos.z);
        if (s > maxLen)
            maxLen = s;
    }
    if (maxLen == 0.0f)
        maxLen = 0.01f;
    for (SrcVertAnim& va : vanims) {
        if (decay == 0.0f) {
            va.speed = 1.0f;
            continue;
        }
        const float s = std::sqrt(va.pos.x * va.pos.x + va.pos.y * va.pos.y + va.pos.z * va.pos.z);
        const float t = s / (maxLen * decay);
        va.speed = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    }
}

// Everything $vta and $vca both need: the file's frames, the basis indexed by
// the VTA's own vertex numbering, and the basis -> mesh vertex fan-out.
struct VtaBind {
    std::vector<VtaFrame> frames;
    std::vector<VtaVert> base;
    std::unordered_map<int, std::vector<int>> vanimMap;
    int matched = 0; // mesh vertices that found a basis vertex

    const VtaFrame* Frame(int time) const {
        for (const VtaFrame& f : frames)
            if (f.time == time)
                return &f;
        return nullptr;
    }
};

bool BindVta(const std::string& path, const Source& mesh, float scale, VtaBind& bind,
             std::string* err) {
    LineReader r;
    if (!ReadAllLines(path, r, err))
        return false;

    bool sawBlock = false;
    for (;;) {
        const std::string* line = r.Next();
        if (!line)
            break;
        char cmd[1024] = {0};
        if (std::sscanf(line->c_str(), "%1023s", cmd) != 1)
            continue;
        if (_stricmp(cmd, "vertexanimation") == 0) {
            if (!GrabVertexAnimation(r, scale, bind.frames, err))
                return false;
            sawBlock = true;
        } else if (_stricmp(cmd, "nodes") == 0 || _stricmp(cmd, "skeleton") == 0 ||
                   _stricmp(cmd, "triangles") == 0) {
            SkipBlock(r); // the mesh already supplied the skeleton and geometry
        }
    }

    if (!sawBlock || bind.frames.empty()) {
        if (err) *err = "no `vertexanimation` block";
        return false;
    }
    const VtaFrame* basis = bind.Frame(0);
    if (!basis) {
        if (err) *err = "vertexanimation block has no `time 0` basis frame";
        return false;
    }
    for (const auto& e : basis->verts) {
        if (e.first >= static_cast<int>(bind.base.size()))
            bind.base.resize(e.first + 1);
        bind.base[e.first] = e.second;
    }

    std::vector<int> modelToVAnim;
    BuildModelToVAnim(mesh, bind.base, modelToVAnim);

    // invert: one basis vertex feeds every mesh vertex that matched it (a
    // material or UV seam duplicates the mesh vertex, not the basis one)
    for (size_t m = 0; m < modelToVAnim.size(); ++m)
        if (modelToVAnim[m] >= 0) {
            bind.vanimMap[modelToVAnim[m]].push_back(static_cast<int>(m));
            ++bind.matched;
        }
    return true;
}

// One frame -> one delta state + its flexkey. Returns the delta count.
int ImportFrame(const VtaBind& bind, const VtaFrame& frame, const std::string& name,
                float position, float decay, Source& out) {
    SrcMorphAnim morph;
    morph.name = name;
    for (const auto& e : frame.verts) {
        if (e.first >= static_cast<int>(bind.base.size()))
            continue;
        auto it = bind.vanimMap.find(e.first);
        if (it == bind.vanimMap.end())
            continue; // basis vertex has no counterpart in this mesh
        const VtaVert& b = bind.base[e.first];
        const pm::Vector3 d{e.second.pos.x - b.pos.x, e.second.pos.y - b.pos.y,
                            e.second.pos.z - b.pos.z};
        const pm::Vector3 nd{e.second.normal.x - b.normal.x, e.second.normal.y - b.normal.y,
                             e.second.normal.z - b.normal.z};
        // reference float16-min cutoff
        if (d.x * d.x + d.y * d.y + d.z * d.z <= 0.001f * 0.001f &&
            nd.x * nd.x + nd.y * nd.y + nd.z * nd.z <= 0.001f)
            continue;
        for (int m : it->second) {
            SrcVertAnim va;
            va.vertex = m;
            va.pos = d;
            va.normal = nd;
            va.side = 1.0f; // no painted balance; $morphsplitstereo supplies it
            morph.vanims.push_back(va);
        }
    }
    ComputeVertexAnimationSpeed(morph.vanims, decay);

    const int count = static_cast<int>(morph.vanims.size());
    SrcFlexKey key;
    key.name = morph.name;
    key.target1 = position;
    out.flexkeys.push_back(std::move(key));
    out.morphs.push_back(std::move(morph));
    return count;
}

} // namespace

bool LoadVtaMorphs(const std::string& path, Source& out, float scale,
                   const std::vector<VtaFlexOption>& opts, std::string* err) {
    VtaBind bind;
    if (!BindVta(path, out, scale, bind, err))
        return false;

    std::printf("  vta %s: %d frames, %d/%d mesh verts matched\n", path.c_str(),
                static_cast<int>(bind.frames.size()), bind.matched,
                static_cast<int>(out.vertex.size()));
    for (const VtaFrame& f : bind.frames)
        std::printf("    frame %-4d %7d verts  %s\n", f.time, static_cast<int>(f.verts.size()),
                    f.comment.c_str());

    for (const VtaFlexOption& opt : opts) {
        if (opt.frame == 0) {
            if (err) *err = "flex \"" + opt.name + "\": frame 0 is the basis";
            return false;
        }
        const VtaFrame* frame = bind.Frame(opt.frame);
        if (!frame) {
            if (err) *err = "flex \"" + opt.name + "\": no frame " + std::to_string(opt.frame) +
                            " in this .vta";
            return false;
        }
        for (const SrcMorphAnim& m : out.morphs)
            if (_stricmp(m.name.c_str(), opt.name.c_str()) == 0) {
                if (err) *err = "duplicate morph \"" + opt.name + "\"";
                return false;
            }

        const int deltas = ImportFrame(bind, *frame, opt.name, opt.position, opt.decay, out);
        std::printf("    flex %-32s frame %-4d %6d deltas  position %.3f decay %.3f\n",
                    opt.name.c_str(), opt.frame, deltas, opt.position, opt.decay);
    }
    return true;
}

// reference Option_VertexCacheAnimationFile: the
// same file a $vta reads, played back rather than posed. Every non-basis frame
// becomes flexdesc "f<i>" and ONE NWAY controller sweeps the whole run over
// 0..1. Stock also emits a "default" flexkey on the basis; it is an empty morph
// that only exists to make its indices line up, so we leave it out.
bool LoadVcaMorphs(const std::string& path, Source& out, float scale, const std::string& name,
                   std::string* err) {
    if (!out.morphs.empty() || !out.flexkeys.empty()) {
        if (err) *err = "the mesh already has morphs - a .vca must be its only flex source";
        return false;
    }

    VtaBind bind;
    if (!BindVta(path, out, scale, bind, err))
        return false;

    // frame order as written, basis dropped; stock numbers f0.. over what is left
    std::vector<const VtaFrame*> anim;
    for (const VtaFrame& f : bind.frames)
        if (f.time != 0)
            anim.push_back(&f);
    if (anim.empty()) {
        if (err) *err = "only the basis frame - nothing to play back";
        return false;
    }

    int deltas = 0;
    for (size_t i = 0; i < anim.size(); ++i) {
        // decay 0 = flat per-vertex speed: a baked frame plays whole, it is not
        // a shape blended in by magnitude
        deltas += ImportFrame(bind, *anim[i], "f" + std::to_string(i), 1.0f, 0.0f, out);
    }

    // the NWAY rig: one raw control per frame, each driving its own flexkey, all
    // grouped under a single multi-controller
    ControllerRemap remap;
    remap.name = name;
    remap.type = RemapType::NWay;
    for (size_t i = 0; i < anim.size(); ++i) {
        const std::string raw = "f" + std::to_string(i);
        remap.rawControls.push_back(raw);
        out.combinationControls.push_back(CombinationControl{raw});
        CombinationRule rule;
        rule.flex = static_cast<int>(i); // index into out.flexkeys
        rule.combination.push_back(static_cast<int>(i));
        out.combinationRules.push_back(std::move(rule));
    }
    out.controllerRemaps.push_back(std::move(remap));

    std::printf("  vca %s -> \"%s\": %d frames, %d/%d mesh verts matched, %d deltas\n",
                path.c_str(), name.c_str(), static_cast<int>(anim.size()), bind.matched,
                static_cast<int>(out.vertex.size()), deltas);
    return true;
}

} // namespace pulse::source
