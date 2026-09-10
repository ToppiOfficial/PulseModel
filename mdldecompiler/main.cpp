// mdldecompiler - reads a compiled .mdl and writes back a .pulseqc describing it.
//
// Usage:
//   mdldecompiler <file.mdl|folder> ... [-o <file.pulseqc>] [-forceversion <n>]
//                            [-dmxencoding <enc>] [-dmxmodel <n>] [-smdanimation]
//                            [-pause]
//
// The script-level markup - names, materials, bodygroups, skeleton, attachments,
// hitboxes, skins - plus one .dmx render mesh per model (dmxwrite.cpp) and one
// clip per animation (animwrite.cpp).

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <map>
#include <new>
#include <set>
#include <system_error>
#include <string>
#include <vector>

#include "animwrite.h"
#include "dmxwrite.h"
#include "fatalerror.h"
#include "flexrig.h"
#include "goldsrc.h"
#include "format/phy.h"
#include "mdlfile.h"
#include "perf.h"

using namespace mdldecompiler;
using pulse::fatal::Fail;

// Name the writer in the crash footer without repeating its name as a string.
#define STAGE(fn, ...) (pulse::fatal::g_stage = #fn, fn(__VA_ARGS__))

// Defined by CMake from TOOL_VERSION, same value the .exe version resource gets.
static constexpr const char* kAppVersion = PULSEMODEL_VERSION;

namespace {

void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("PulseModel [Model Decompiler]\n");
    std::printf("version:   %s (model version 10-49)\n", kAppVersion);
    std::printf("developer: Toppi\n");
    std::printf("-------------------------------\n");
}

int Usage() {
    std::printf("usage: mdldecompiler <file.mdl|folder> ... [-o <file.pulseqc>] [-outdir <dir>]\n");
    std::printf("                     [-forceversion <n>] [-dmxencoding <enc>] [-dmxmodel <n>]\n");
    std::printf("                     [-smdanimation] [-pulseqc]\n");
    std::printf("\n");
    std::printf("  several inputs may be given (drag-and-drop); a folder decompiles every\n");
    std::printf("  .mdl under it, recursively\n");
    std::printf("\n");
    std::printf("  -o <file>     script to write; defaults to a folder named after the\n");
    std::printf("                .mdl, next to it, holding the script and its meshes\n");
    std::printf("                (ignored when more than one model is decompiled)\n");
    std::printf("  -outdir <dir> put those per-model folders under <dir> instead of beside\n");
    std::printf("                the .mdl; absolute, or relative to the current directory\n");
    std::printf("  -forceversion <n>\n");
    std::printf("                read the file as version <n>, ignoring the header field\n");
    std::printf("                (some compilers write a bogus one to block decompiling)\n");
    std::printf("  -dmxencoding <enc>\n");
    std::printf("                how the .dmx meshes are encoded: binary (default) or\n");
    std::printf("                keyvalues2 text\n");
    std::printf("  -dmxmodel <n> the `format model` version they declare: 15 (default),\n");
    std::printf("                1, 18, or 22 for Source 2 modeldoc\n");
    std::printf("  -smdanimation write the animation clips as .smd instead of .dmx\n");
    std::printf("  -pulseqc      write a .pulseqc instead of the default stock .qc\n");
    std::printf("  -pause        wait for a keypress before exiting (drag-and-drop runs)\n");
    std::printf("  -perfmetrics  print wall time in ms per process once the run ends\n");
    return 1;
}

bool ReadFile(const char* path, Mdl& m, int forceVersion, std::string& err) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) {
        err = std::string("cannot open \"") + path + "\"";
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (size < static_cast<long>(sizeof(fm::studiohdr_t))) {
        err = std::string("\"") + path + "\" is too small to be a .mdl";
        std::fclose(f);
        return false;
    }
    m.buf.resize(static_cast<size_t>(size));
    const size_t got = std::fread(m.buf.data(), 1, m.buf.size(), f);
    std::fclose(f);
    if (got != m.buf.size()) {
        err = std::string("short read on \"") + path + "\"";
        return false;
    }

    m.hdr = reinterpret_cast<const fm::studiohdr_t*>(m.buf.data());
    if (m.hdr->id != fm::kIdStudioHeader) {
        err = std::string("\"") + path + "\" is not a studio model (bad id)";
        return false;
    }
    const int32_t headerVersion = m.hdr->version;
    const int32_t version = forceVersion ? forceVersion : headerVersion;
    if (version != headerVersion)
        std::printf("file says version %d, reading it as %d\n", headerVersion, version);
    // 44 through 49 share one binary layout: Valve only ever repurposed reserved
    // padding in studiohdr_t/mstudiobone_t/mstudioanimdesc_t/mstudioseqdesc_t,
    // never moved a field, so the v49 structs below read them all. studiohdr2_t
    // only exists from 45 on; earlier files simply have studiohdr2index == 0,
    // which every reader here already treats as "no extension header".
    if (version < 44 || version > 49)
        std::printf("model version %d is outside the confirmed 44-49 range; reading it as 49 anyway\n",
                     version);
    return true;
}

// Angles we recover through trig (acos, MatrixAngles' atan2/asin) come back a
// hair off - 22 degrees reads as 21.999996. Snap to the places that survive the
// round trip. Numbers read straight out of the file are exact and skip this.
std::string Deg(float radians) {
    return F(static_cast<float>(std::round(radians * pm::kRad2Deg * 10000.0) / 10000.0));
}

std::string V3Deg(const pm::Vector3& radians) {
    return Deg(radians.x) + " " + Deg(radians.y) + " " + Deg(radians.z);
}

// $definebone and $attachment read their angles as QAngle degrees - pitch, yaw,
// roll - while a RadianEuler holds the same rotation as (roll, pitch, yaw).
// Writing one straight out cycles the three components and the recompile
// rebinds every vertex to a twisted skeleton. ($driverbone is the exception -
// its triggers are RadianEuler-ordered, so they go out through V3Deg.)
std::string QAngleDeg(const pm::RadianEuler& r) {
    return Deg(r.y) + " " + Deg(r.z) + " " + Deg(r.x);
}

// The compiler swizzles a script-space point into model space as (-y, x, z);
// $eyeposition and the bone-less $illumposition both go through it.
pm::Vector3 Unswizzle(const pm::Vector3& v) {
    return {v.y, -v.x, v.z};
}

// --- the writer -------------------------------------------------------------

struct Qc {
    std::FILE* f;
    std::string* sink = nullptr; // set to capture output instead of writing it
    void Line(const std::string& s) {
        if (sink)
            sink->append(s).append("\n");
        else
            std::fprintf(f, "%s\n", s.c_str());
    }
    void Blank() { Line(""); }
};

// Source filenames. studiomdl's $addsearchdir does nothing, so a studiomdl .qc
// spells the folder into every reference instead.
std::string MeshFile(const std::string& name) {
    return (g_studiomdl ? "meshes/" : "") + name + ".dmx";
}
std::string AnimFile(const std::string& name) {
    return (g_studiomdl ? "anims/" : "") + name + AnimExt();
}

// Re-emit captured text at `indent`, blank lines left blank.
void Indent(Qc& q, const std::string& text, const char* indent) {
    for (size_t p = 0; p < text.size();) {
        const size_t e = text.find('\n', p);
        const std::string line = text.substr(p, e - p);
        q.Line(line.empty() ? line : indent + line);
        p = e == std::string::npos ? text.size() : e + 1;
    }
}

// Which archetype the compile ran under. Static sets the flag and collapses to
// one "static_prop" bone; simple collapses to one "prop_root"; general keeps
// the skeleton. A surviving bone table is therefore the whole signal.
const char* Archetype(const Mdl& m) {
    if (m.hdr->flags & fm::STUDIOHDR_FLAGS_STATIC_PROP)
        return "static";
    return m.hdr->numbones <= 1 ? "simple" : "general";
}

// The running contents word starts at CONTENTS_SOLID and each token only adds
// or removes bits, so "notsolid" carries any value that lacks solid. Only the
// pure-add words are named; grate (which also clears solid) and anything
// unrecognized go out as one hex token, which ORs in the same way.
std::string ContentsTokens(int32_t v) {
    static const struct {
        int32_t bit;
        const char* name;
    } kNamed[] = {{fm::CONTENTS_SOLID, "solid"},
                  {fm::CONTENTS_LADDER, "ladder"},
                  {fm::CONTENTS_MONSTER, "monster"},
                  {fm::CONTENTS_DEBRIS, "debris"}};

    std::string out;
    int32_t rest = v;
    for (const auto& k : kNamed)
        if (v & k.bit) {
            out += (out.empty() ? "" : " ") + std::string(k.name);
            rest &= ~k.bit;
        }
    if (rest) {
        char hex[16];
        std::snprintf(hex, sizeof hex, "0x%X", static_cast<unsigned>(rest));
        out += (out.empty() ? "" : " ") + std::string(hex);
    }
    if (!(v & fm::CONTENTS_SOLID))
        out += (out.empty() ? "" : " ") + std::string("notsolid");
    return out.empty() ? "notsolid" : out;
}

// $illumposition with a bone becomes a rigid "__illumPosition" attachment the
// header points at; returns that attachment's index, or -1 when it is static.
int IllumAttachment(const Mdl& m) {
    const fm::studiohdr2_t* h2 =
        m.At<fm::studiohdr2_t>(m.buf.data(), m.hdr->studiohdr2index);
    if (!h2 || h2->illumpositionattachmentindex <= 0 ||
        h2->illumpositionattachmentindex > m.hdr->numlocalattachments)
        return -1;
    return h2->illumpositionattachmentindex - 1;
}

void WriteHeader(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    q.Line("$modelname \"" + std::string(h.name) + "\"");

    // sources go in meshes/ and anims/ beside the script - registered here so
    // every reference below can name a bare filename
    q.Blank();
    if (!g_studiomdl) {
        q.Line("$addsearchdir \"meshes\"");
        q.Line("$addsearchdir \"anims\"");
        q.Blank();
    }

    // stock only spells the static archetype - general and simple have no command
    if (!g_studiomdl)
        q.Line(std::string("$modelarchetype ") + Archetype(m));
    else if (std::strcmp(Archetype(m), "static") == 0)
        q.Line("$staticprop");

    const char* prop = m.Str(m.buf.data(), h.surfacepropindex);
    if (*prop)
        q.Line("$surfaceprop \"" + std::string(prop) + "\"");

    q.Line("$contents " + ContentsTokens(h.contents));

    if (h.flags & fm::STUDIOHDR_FLAGS_FORCE_OPAQUE)
        q.Line(g_studiomdl ? "$opaque" : "$renderpass opaque");
    else if (h.flags & fm::STUDIOHDR_FLAGS_TRANSLUCENT_TWOPASS)
        q.Line(g_studiomdl ? "$mostlyopaque" : "$renderpass mostlyopaque");

    if (h.flags & fm::STUDIOHDR_FLAGS_AMBIENT_BOOST)
        q.Line("$ambientboost");
    if (h.flags & fm::STUDIOHDR_FLAGS_DO_NOT_CAST_SHADOWS)
        q.Line("$donotcastshadows");
    if (h.flags & fm::STUDIOHDR_FLAGS_FORCE_PHONEME_CROSSFADE)
        q.Line("$forcephonemecrossfade");
    if (h.flags & fm::STUDIOHDR_FLAGS_NO_FORCED_FADE)
        q.Line("$noforcedfade");
    if (h.flags & fm::STUDIOHDR_FLAGS_CAST_TEXTURE_SHADOWS)
        q.Line("$casttextureshadows");
    if (h.flags & fm::STUDIOHDR_FLAGS_CONSTANT_DIRECTIONAL_LIGHT_DOT)
        q.Line("$constantdirectionallight " + F(h.constdirectionallightdot / 255.0f));

    q.Blank();
    q.Line("$bbox " + V3(h.hull_min) + "  " + V3(h.hull_max));
    if (h.view_bbmin.x || h.view_bbmin.y || h.view_bbmin.z || h.view_bbmax.x || h.view_bbmax.y ||
        h.view_bbmax.z)
        q.Line("$cbox " + V3(h.view_bbmin) + "  " + V3(h.view_bbmax));
    if (h.eyeposition.x || h.eyeposition.y || h.eyeposition.z)
        q.Line("$eyeposition " + V3(Unswizzle(h.eyeposition)));

    const fm::studiohdr2_t* h2 = m.At<fm::studiohdr2_t>(m.buf.data(), h.studiohdr2index);
    const int illumAtt = IllumAttachment(m);
    if (illumAtt < 0) {
        q.Line("$illumposition " + V3(Unswizzle(h.illumposition)));
    } else {
        // the bone form lives in the attachment, not in the header numbers
        const fm::mstudioattachment_t& a = m.At<fm::mstudioattachment_t>(
            m.buf.data(), h.localattachmentindex, h.numlocalattachments)[illumAtt];
        pm::RadianEuler rot;
        pm::Vector3 pos;
        pm::MatrixAngles(a.local, rot, pos);
        const std::vector<std::string> names = BoneNames(m);
        const std::string bone =
            (a.localbone >= 0 && static_cast<size_t>(a.localbone) < names.size())
                ? names[a.localbone]
                : std::string();
        q.Line("$illumposition " + V3(pos) + " \"" + bone + "\"");
    }
    if (h2 && h2->flMaxEyeDeflection != 0.0f)
        q.Line("$maxeyedeflection " + Deg(std::acos(h2->flMaxEyeDeflection)));
}

// $cdmaterials. An empty entry is skipped: the compiler adds one of its own the
// moment any texture is relative-path flagged, which every DMX texture is, so
// re-declaring it would leave the model with two. A model whose material path
// lives in the texture names rather than here is exactly that case.
void WriteMaterials(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const int32_t* cds = m.At<int32_t>(m.buf.data(), h.cdtextureindex, h.numcdtextures);
    if (!cds)
        return;
    bool any = false;
    for (int i = 0; i < h.numcdtextures; ++i) {
        const std::string cd = m.Str(m.buf.data(), cds[i]);
        if (cd.empty())
            continue;
        if (!any) {
            q.Blank();
            any = true;
        }
        q.Line("$cdmaterials \"" + cd + "\"");
    }
}

// Console list of the materials the model references. No .vmt is extracted -
// this is the checklist of what has to be supplied by hand for a recompile.
void PrintMaterials(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const std::vector<std::string> names = TextureNames(m);
    if (names.empty())
        return;
    std::printf("\nmaterials:\n");
    for (const std::string& n : names)
        std::printf("  %s.vmt\n", n.c_str());
    const int32_t* cds = m.At<int32_t>(m.buf.data(), h.cdtextureindex, h.numcdtextures);
    for (int i = 0; cds && i < h.numcdtextures; ++i) {
        const std::string cd = m.Str(m.buf.data(), cds[i]);
        if (!cd.empty())
            std::printf("  searched in \"%s\"\n", cd.c_str());
    }
}

// $texturegroup - one $set per skin family past family 0, listing only the
// slots that actually differ from it.
void WriteSkins(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    if (h.numskinfamilies <= 1 || h.numskinref <= 0)
        return;
    const int16_t* skins =
        m.At<int16_t>(m.buf.data(), h.skinindex, h.numskinfamilies * h.numskinref);
    if (!skins)
        return;
    const std::vector<std::string> names = TextureNames(m);
    auto name = [&](int16_t i) {
        return (i >= 0 && static_cast<size_t>(i) < names.size()) ? BaseName(names[i])
                                                                 : std::string();
    };

    // stock's form is one braced row per family including the base one, and only
    // the slots that vary - a column identical down every row is an untouched
    // material that the group must not claim.
    if (g_studiomdl) {
        std::vector<int> cols;
        for (int r = 0; r < h.numskinref; ++r)
            for (int fam = 1; fam < h.numskinfamilies; ++fam)
                if (skins[fam * h.numskinref + r] != skins[r]) {
                    cols.push_back(r);
                    break;
                }
        if (cols.empty())
            return;
        q.Blank();
        q.Line("$texturegroup \"skinfamilies\"");
        q.Line("{");
        for (int fam = 0; fam < h.numskinfamilies; ++fam) {
            std::string row = "    {";
            for (int r : cols)
                row += " \"" + name(skins[fam * h.numskinref + r]) + "\"";
            q.Line(row + " }");
        }
        q.Line("}");
        return;
    }

    q.Blank();
    q.Line("$texturegroup {");
    for (int fam = 1; fam < h.numskinfamilies; ++fam) {
        q.Line("    $set {");
        for (int r = 0; r < h.numskinref; ++r) {
            const int16_t base = skins[r], cur = skins[fam * h.numskinref + r];
            if (base != cur)
                q.Line("        material \"" + name(base) + "\" \"" + name(cur) + "\"");
        }
        q.Line("    }");
    }
    q.Line("}");
}

// Does this model carry morph or eyeball data - i.e. must it be a $model?
bool ModelHasFace(const Mdl& m, const fm::mstudiomodel_t& mo) {
    if (mo.numeyeballs > 0)
        return true;
    const fm::mstudiomesh_t* meshes = m.At<fm::mstudiomesh_t>(&mo, mo.meshindex, mo.nummeshes);
    for (int k = 0; meshes && k < mo.nummeshes; ++k)
        if (meshes[k].numflexes > 0)
            return true;
    return false;
}

// The alias each model's .dmx is written under, "" for a blank body. dmxwrite
// writes <alias>.dmx and every reference in the script names it.
std::vector<std::vector<std::string>> MeshNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::vector<std::string>> out;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    if (!parts)
        return out;
    std::set<std::string> used;
    out.resize(h.numbodyparts);
    for (int i = 0; i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            if (models[j].numvertices == 0) {
                out[i].push_back(""); // blank body
                continue;
            }
            const std::string file(models[j].name, strnlen(models[j].name, sizeof models[j].name));
            std::string name = StripExt(BaseName(file));
            if (!CleanName(name))
                name = "mesh";
            std::string unique = name;
            for (int n = 2; !used.insert(unique).second; ++n)
                unique = name + "_" + std::to_string(n);
            out[i].push_back(unique);
        }
    }
    return out;
}

// The bodypart carrying the morph/eyeball data - in studiomdl mode the one that
// has to be a $model. -1 when nothing does.
int FacePart(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    for (int i = 0; parts && i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j)
            if (ModelHasFace(m, models[j]))
                return i;
    }
    return -1;
}

// The .dmx holding the face model's morphs - what dmxeyelid loads deltas from.
std::string FaceMesh(const Mdl& m, const std::vector<std::vector<std::string>>& names) {
    const int p = FacePart(m);
    if (p < 0 || static_cast<size_t>(p) >= names.size())
        return std::string();
    for (const std::string& n : names[p])
        if (!n.empty())
            return MeshFile(n);
    return std::string();
}

// $rendermesh + $modelgroup, or in -studiomdl mode $bodygroup / $model.
// `faceBody` is the captured flex/eye markup, which only a $model can hold.
void WriteBodyParts(Qc& q, const Mdl& m, const std::vector<std::vector<std::string>>& meshNames,
                    const std::string& faceBody) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    if (!parts || meshNames.empty())
        return;

    // studiomdl has no $rendermesh - a $bodygroup/$model names the file itself
    if (!g_studiomdl) {
        q.Blank();
        for (const std::vector<std::string>& part : meshNames)
            for (const std::string& name : part)
                if (!name.empty())
                    q.Line("$rendermesh \"" + name + "\" \"" + MeshFile(name) + "\"");
    }

    // the markup has to land on some $model even when no mesh admits to
    // carrying a morph - bodypart 0 is the only sane fallback
    int facePart = FacePart(m);
    if (facePart < 0 && !faceBody.empty())
        facePart = 0;

    bool placed = false;
    for (int i = 0; i < h.numbodyparts; ++i) {
        const std::string part = m.Str(&parts[i], parts[i].sznameindex);
        q.Blank();
        if (!g_studiomdl) {
            q.Line("$modelgroup \"" + part + "\" {");
            for (const std::string& name : meshNames[i])
                // the reference is quoted too - a model name with a space in it
                // is one token only when it is
                q.Line(name.empty() ? "    blank"
                                    : "    mesh name \"" + name + "\" \"" + name + "\"");
            q.Line("}");
            continue;
        }

        // A $bodygroup entry takes no options, so a model with flex/eyeball data
        // has to be a $model - which makes its own single-model bodypart. Only a
        // part with something to choose between loses anything that way, so only
        // that one gets the grouping written out commented.
        const bool split = i == facePart;
        const char* pre = split ? "// " : "";
        if (!split || meshNames[i].size() > 1) {
            if (split)
                q.Line("// flex/eyeball data forces $model, which cannot be a bodygroup member:");
            q.Line(pre + ("$bodygroup \"" + part + "\""));
            q.Line(std::string(pre) + "{");
            for (const std::string& name : meshNames[i])
                q.Line(pre +
                       (name.empty() ? "    blank" : "    studio \"" + MeshFile(name) + "\""));
            q.Line(std::string(pre) + "}");
        }
        if (!split)
            continue;
        for (const std::string& name : meshNames[i]) {
            if (name.empty())
                continue;
            // $model reads its options off the command's own line, so a '{' on
            // the next one never opens the block - the file becomes an option
            const bool body = !placed && !faceBody.empty();
            q.Line("$model \"" + name + "\" \"" + MeshFile(name) + "\"" + (body ? " {" : ""));
            if (!body)
                continue;
            Indent(q, faceBody, "    ");
            q.Line("}");
            placed = true;
        }
    }
}


void WriteBones(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || h.numbones <= 0)
        return;
    // static/simple collapse to one compiler-made bone ("static_prop" /
    // "prop_root"); re-declaring it would force a bone the source never had
    if (std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);
    const std::vector<LocalPose> poses = LocalPoses(m);

    q.Blank();
    int rebuilt = 0;
    for (int i = 0; i < h.numbones; ++i) {
        const int32_t p = bones[i].parent;
        rebuilt += poses[i].rebuilt;

        const std::string parent =
            (p >= 0 && static_cast<size_t>(p) < names.size()) ? names[p] : std::string();
        std::string line = "$definebone \"" + names[i] + "\" \"" + parent + "\" " +
                           V3(poses[i].pos) + " " + QAngleDeg(poses[i].rot);
        // The identity second sextet marks the bone pre-aligned. A bone table
        // read out of a finished .mdl is ALREADY realigned, and without this
        // RealignBones aims an $ikchain's first two links at their child a
        // SECOND time - which walks their bind pose off by tens of units.
        // Identity, not the stored srcbonetransform: the animations are in the
        // final frame too, so a real srcRealign would be applied to them twice.
        q.Line(line + "  0 0 0 0 0 0");
    }
    if (rebuilt)
        q.Line("// " + std::to_string(rebuilt) + " of " + std::to_string(h.numbones) +
               " bones stored a pos/rot that contradicts poseToBone - rebuilt from the matrix");
    int renamed = 0;
    for (int i = 0; i < h.numbones; ++i)
        renamed += (names[i] != m.Str(&bones[i], bones[i].sznameindex));
    if (renamed)
        q.Line("// " + std::to_string(renamed) + " of " + std::to_string(h.numbones) +
               " bone names were junk or duplicated - renamed bone_<index>, real names are lost");
}

// $bonemerge - the tag survives as a bone flag, so it reads straight back.
void WriteBoneMerges(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);

    bool any = false;
    for (int i = 0; i < h.numbones; ++i) {
        if (!(bones[i].flags & fm::BONE_USED_BY_BONE_MERGE))
            continue;
        if (!any) {
            q.Blank();
            any = true;
        }
        q.Line("$bonemerge \"" + names[i] + "\"");
    }
}

// $jigglebone. Every field of an active section is written, rather than diffed
// against the parser's defaults, so the block round-trips whatever those are.
// The constraint clauses are the exception - writing one sets its flag, so they
// follow the flags instead.
void WriteJiggleBones(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);

    for (int i = 0; i < h.numbones; ++i) {
        if (bones[i].proctype != fm::STUDIO_PROC_JIGGLE)
            continue;
        const fm::mstudiojigglebone_t* j =
            m.At<fm::mstudiojigglebone_t>(&bones[i], bones[i].procindex);
        if (!j)
            continue;
        const int32_t f = j->flags;
        auto kv = [&](const char* k, const std::string& v) {
            q.Line(std::string("        ") + k + " " + v);
        };

        // the common options are legal in every section but is_boing, so they
        // ride along in the first one that takes them
        const char* host = (f & fm::JIGGLE_IS_FLEXIBLE)      ? "is_flexible"
                           : (f & fm::JIGGLE_IS_RIGID)       ? "is_rigid"
                           : (f & fm::JIGGLE_HAS_BASE_SPRING) ? "has_base_spring"
                                                              : nullptr;
        auto common = [&](const char* sect) {
            if (!host || std::strcmp(host, sect) != 0)
                return;
            kv("length", F(j->length));
            kv("tip_mass", F(j->tipMass));
            kv("yaw_friction", F(j->yawFriction));
            kv("yaw_bounce", F(j->yawBounce));
            kv("pitch_friction", F(j->pitchFriction));
            kv("pitch_bounce", F(j->pitchBounce));
            if (f & fm::JIGGLE_HAS_ANGLE_CONSTRAINT)
                kv("angle_constraint", Deg(j->angleLimit));
            if (f & fm::JIGGLE_HAS_YAW_CONSTRAINT)
                kv("yaw_constraint", Deg(j->minYaw) + " " + Deg(j->maxYaw));
            if (f & fm::JIGGLE_HAS_PITCH_CONSTRAINT)
                kv("pitch_constraint", Deg(j->minPitch) + " " + Deg(j->maxPitch));
        };

        q.Blank();
        if (!host)
            q.Line("// no is_flexible/is_rigid/has_base_spring section - length, tip mass and the"
                   "\n// constraints have nowhere legal to go and are lost");
        q.Line("$jigglebone \"" + names[i] + "\" {");

        if (f & fm::JIGGLE_IS_FLEXIBLE) {
            q.Line("    is_flexible {");
            kv("yaw_stiffness", F(j->yawStiffness));
            kv("yaw_damping", F(j->yawDamping));
            kv("pitch_stiffness", F(j->pitchStiffness));
            kv("pitch_damping", F(j->pitchDamping));
            kv("along_stiffness", F(j->alongStiffness));
            kv("along_damping", F(j->alongDamping));
            if (!(f & fm::JIGGLE_HAS_LENGTH_CONSTRAINT))
                q.Line("        allow_length_flex");
            common("is_flexible");
            q.Line("    }");
        }
        if (f & fm::JIGGLE_IS_RIGID) {
            q.Line("    is_rigid {");
            common("is_rigid");
            q.Line("    }");
        }
        if (f & fm::JIGGLE_HAS_BASE_SPRING) {
            q.Line("    has_base_spring {");
            kv("stiffness", F(j->baseStiffness));
            kv("damping", F(j->baseDamping));
            kv("base_mass", F(j->baseMass));
            kv("left_constraint", F(j->baseMinLeft) + " " + F(j->baseMaxLeft));
            kv("left_friction", F(j->baseLeftFriction));
            kv("up_constraint", F(j->baseMinUp) + " " + F(j->baseMaxUp));
            kv("up_friction", F(j->baseUpFriction));
            kv("forward_constraint", F(j->baseMinForward) + " " + F(j->baseMaxForward));
            kv("forward_friction", F(j->baseForwardFriction));
            common("has_base_spring");
            q.Line("    }");
        }
        if (f & fm::JIGGLE_IS_BOING) {
            q.Line("    is_boing {");
            kv("impact_speed", F(j->boingImpactSpeed));
            // stored as a cosine; %.6f degrees keeps the round trip on the same float
            kv("impact_angle",
               F(static_cast<float>(std::acos(std::max(-1.0f, std::min(1.0f, j->boingImpactAngle))) *
                                    pm::kRad2Deg)));
            kv("damping_rate", F(j->boingDampingRate));
            kv("frequency", F(j->boingFrequency));
            kv("amplitude", F(j->boingAmplitude));
            q.Line("    }");
        }
        q.Line("}");
    }
}

// $driverbone. What is on disk is the helper pose parent-relative in full,
// which is what `absolute` means, so the block is written that way - the
// relative form would have the compiler fold the bind pose in a second time.
// basepos is 0 because it is already summed into each trigger's own offset.
void WriteDriverBones(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);

    for (int i = 0; i < h.numbones; ++i) {
        if (bones[i].proctype != fm::STUDIO_PROC_QUATINTERP)
            continue;
        const fm::mstudioquatinterpbone_t* qi =
            m.At<fm::mstudioquatinterpbone_t>(&bones[i], bones[i].procindex);
        if (!qi)
            continue;
        const fm::mstudioquatinterpinfo_t* tr =
            m.At<fm::mstudioquatinterpinfo_t>(qi, qi->triggerindex, qi->numtriggers);
        if (!tr)
            continue;
        const std::string driver =
            (qi->control >= 0 && static_cast<size_t>(qi->control) < names.size())
                ? names[qi->control]
                : std::string();

        q.Blank();
        q.Line("$driverbone \"" + names[i] + "\" \"" + driver + "\" absolute {");
        for (int t = 0; t < qi->numtriggers; ++t) {
            pm::RadianEuler driverRot, helperRot;
            pm::QuaternionAngles(tr[t].trigger, driverRot);
            pm::QuaternionAngles(tr[t].quat, helperRot);
            // the file keeps 1/tolerance; a zero would have been refused on the
            // way in, so the reciprocal is safe
            // RadianEuler order, not QAngle - a trigger's angles go straight
            // into AngleQuaternion, same as the VRD line they mirror
            q.Line("    trigger " + Deg(1.0f / tr[t].inv_tolerance) + "  " + V3Deg(driverRot) +
                   "  " + V3Deg(helperRot) + "  " + V3(tr[t].pos));
        }
        q.Line("}");
    }
}

// $driveraimat. The base position is not in the command - the compiler seeds it
// from the bone's rest pose - so a stored one that disagrees came from a VRD and
// is called out rather than silently dropped.
void WriteAimAtBones(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);
    const std::vector<LocalPose> poses = LocalPoses(m);
    const fm::mstudioattachment_t* atts =
        m.At<fm::mstudioattachment_t>(m.buf.data(), h.localattachmentindex, h.numlocalattachments);

    bool any = false;
    for (int i = 0; i < h.numbones; ++i) {
        const bool attach = bones[i].proctype == fm::STUDIO_PROC_AIMATATTACH;
        if (!attach && bones[i].proctype != fm::STUDIO_PROC_AIMATBONE)
            continue;
        const fm::mstudioaimatbone_t* ab =
            m.At<fm::mstudioaimatbone_t>(&bones[i], bones[i].procindex);
        if (!ab)
            continue;

        std::string target;
        if (attach) {
            if (atts && ab->aim >= 0 && ab->aim < h.numlocalattachments)
                target = m.Str(&atts[ab->aim], atts[ab->aim].sznameindex);
        } else if (ab->aim >= 0 && static_cast<size_t>(ab->aim) < names.size()) {
            target = names[ab->aim];
        }

        if (!any) {
            q.Blank();
            any = true;
        }
        const pm::Vector3& rest = poses[i].pos;
        if (std::fabs(ab->basepos.x - rest.x) > 0.01f ||
            std::fabs(ab->basepos.y - rest.y) > 0.01f ||
            std::fabs(ab->basepos.z - rest.z) > 0.01f)
            q.Line("// aim-at base position " + V3(ab->basepos) + " is not the bone's rest pose -"
                   " $driveraimat always uses the rest pose, so that offset is lost");
        q.Line("$driveraimat \"" + names[i] + "\" \"" + target + "\" " + V3(ab->upvector) + " " +
               V3(ab->aimvector));
    }
}

// $proceduralbones + the .vrd it names - the file form of $driverbone and
// $driveraimat, and the only spelling stock studiomdl has for either. Written
// beside the script; the aim-at block carries its own <basepos>, which the
// inline $driveraimat cannot.
void WriteProceduralBones(Qc& q, const Mdl& m, const std::string& dir) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones || std::strcmp(Archetype(m), "general") != 0)
        return;
    const std::vector<std::string> names = BoneNames(m);
    const fm::mstudioattachment_t* atts =
        m.At<fm::mstudioattachment_t>(m.buf.data(), h.localattachmentindex, h.numlocalattachments);
    // stock's VRD scanner is a bare sscanf: whitespace-delimited, no quoting,
    // and it drops a name's prefix up to the first '.'. The prefix eating is
    // VRD-only - every other command in the script takes the full bone name.
    std::set<std::string> spaced;
    auto vrd = [&](std::string n) {
        const size_t dot = n.find('.');
        if (dot != std::string::npos)
            n.erase(0, dot + 1);
        if (n.find(' ') != std::string::npos)
            spaced.insert(n);
        return n;
    };
    auto pick = [&](int32_t i) {
        return vrd((i >= 0 && static_cast<size_t>(i) < names.size()) ? names[i] : std::string());
    };

    // One procedural bone. The triggers are kept as cells so their columns can
    // be padded to a common width once the block is complete.
    struct Block {
        std::string head;
        std::vector<std::string> tail;              // the aim-at value lines
        std::vector<std::vector<std::string>> rows; // <trigger>, keyword dropped
    };
    std::vector<Block> blocks;

    for (int i = 0; i < h.numbones; ++i) {
        if (bones[i].proctype == fm::STUDIO_PROC_QUATINTERP) {
            const fm::mstudioquatinterpbone_t* qi =
                m.At<fm::mstudioquatinterpbone_t>(&bones[i], bones[i].procindex);
            const fm::mstudioquatinterpinfo_t* tr =
                qi ? m.At<fm::mstudioquatinterpinfo_t>(qi, qi->triggerindex, qi->numtriggers)
                   : nullptr;
            if (!tr)
                continue;
            const int32_t ctl = qi->control;
            Block b;
            b.head = "<helper> " + vrd(names[i]) + " " + pick(bones[i].parent) + " " +
                     pick(ctl >= 0 && ctl < h.numbones ? bones[ctl].parent : -1) + " " + pick(ctl);
            for (int t = 0; t < qi->numtriggers; ++t) {
                pm::RadianEuler driverRot, helperRot;
                pm::QuaternionAngles(tr[t].trigger, driverRot);
                pm::QuaternionAngles(tr[t].quat, helperRot);
                b.rows.push_back({Deg(1.0f / tr[t].inv_tolerance), Deg(driverRot.x),
                                  Deg(driverRot.y), Deg(driverRot.z), Deg(helperRot.x),
                                  Deg(helperRot.y), Deg(helperRot.z), F(tr[t].pos.x),
                                  F(tr[t].pos.y), F(tr[t].pos.z)});
            }
            blocks.push_back(std::move(b));
            continue;
        }
        const bool attach = bones[i].proctype == fm::STUDIO_PROC_AIMATATTACH;
        if (!attach && bones[i].proctype != fm::STUDIO_PROC_AIMATBONE)
            continue;
        const fm::mstudioaimatbone_t* ab =
            m.At<fm::mstudioaimatbone_t>(&bones[i], bones[i].procindex);
        if (!ab)
            continue;
        std::string target;
        if (attach) {
            if (atts && ab->aim >= 0 && ab->aim < h.numlocalattachments)
                target = m.Str(&atts[ab->aim], atts[ab->aim].sznameindex);
        } else {
            target = pick(ab->aim);
        }
        Block b;
        b.head = "<aimconstraint> " + vrd(names[i]) + " " + pick(bones[i].parent) + " " +
                 vrd(target);
        b.tail = {"<aimvector> " + V3(ab->aimvector), "<upvector> " + V3(ab->upvector),
                  "<basepos> " + V3(ab->basepos)};
        blocks.push_back(std::move(b));
    }
    if (blocks.empty())
        return;

    // basepos is file-scoped and carries across helpers, so every block states
    // its own. A helper's is always 0 - the offset is summed into each trigger.
    std::vector<std::string> lines;
    for (const Block& b : blocks) {
        if (!lines.empty())
            lines.push_back("");
        lines.push_back(b.head);
        for (const std::string& t : b.tail)
            lines.push_back(t);
        if (b.rows.empty())
            continue;
        lines.push_back("<basepos> 0 0 0");
        std::vector<size_t> w(b.rows.front().size(), 0);
        for (const std::vector<std::string>& r : b.rows)
            for (size_t c = 0; c < r.size(); ++c)
                w[c] = std::max(w[c], r[c].size());
        for (const std::vector<std::string>& r : b.rows) {
            std::string s = "<trigger>";
            for (size_t c = 0; c < r.size(); ++c)
                // a wider gap opens each triple; padding trails the value so the
                // columns line up on the left
                s += (c == 1 || c == 4 || c == 7 ? "  " : " ") + r[c] +
                     std::string(w[c] - r[c].size(), ' ');
            s.erase(s.find_last_not_of(' ') + 1);
            lines.push_back(s);
        }
    }

    const std::string name = BaseName(dir) + ".vrd";
    std::FILE* f = std::fopen((std::filesystem::path(dir) / name).string().c_str(), "wb");
    if (!f) {
        q.Blank();
        q.Line("// could not write " + name + " - the procedural bones are lost");
        return;
    }
    // a .vrd has no way to quote, so a name with a space in it splits into two
    // tokens and the line it is on will not parse
    for (const std::string& s : spaced)
        std::fprintf(f, "// \"%s\" has a space - the lines naming it will not parse\n", s.c_str());
    for (const std::string& l : lines)
        std::fprintf(f, "%s\n", l.c_str());
    std::fclose(f);
    std::printf("\nprocedural bones:\n  wrote %s\n", name.c_str());

    q.Blank();
    if (!spaced.empty())
        q.Line("// " + std::to_string(spaced.size()) +
               " bone name(s) in the .vrd contain a space, which it cannot quote - those"
               "\n// procedural bones will not load; rename them in the source");
    q.Line("$proceduralbones \"" + name + "\"");
}

// --- physics ----------------------------------------------------------------

// One `section { "key" "value" ... }` out of the .phy's plain-text tail. Every
// physics setting a script authored is in there except the hull geometry, which
// is the binary half of the file.
struct PhySection {
    std::string name;
    std::vector<std::pair<std::string, std::string>> kv;
    std::string raw;

    const std::string* Find(const char* key) const {
        for (const auto& p : kv)
            if (p.first == key)
                return &p.second;
        return nullptr;
    }
    std::string Get(const char* key) const {
        const std::string* v = Find(key);
        return v ? *v : std::string();
    }
    float Getf(const char* key, float dflt = 0.0f) const {
        const std::string* v = Find(key);
        return v ? static_cast<float>(std::atof(v->c_str())) : dflt;
    }
};

// Skips the header and the per-solid collision blobs, then reads the text that
// follows up to its NUL. Missing or malformed file = no sections, no output.
std::vector<PhySection> ReadPhy(const std::string& mdlPath) {
    std::vector<PhySection> out;
    std::FILE* f = std::fopen((StripExt(mdlPath) + ".phy").c_str(), "rb");
    if (!f)
        return out;
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<char> buf(size > 0 ? static_cast<size_t>(size) : 0);
    const size_t got = buf.empty() ? 0 : std::fread(buf.data(), 1, buf.size(), f);
    std::fclose(f);
    if (got != buf.size() || buf.size() < sizeof(fm::phyheader_t))
        return out;

    const fm::phyheader_t* h = reinterpret_cast<const fm::phyheader_t*>(buf.data());
    size_t p = sizeof(fm::phyheader_t);
    for (int i = 0; i < h->solidCount; ++i) {
        int32_t blob = 0;
        if (p + sizeof(blob) > buf.size())
            return out;
        std::memcpy(&blob, &buf[p], sizeof(blob));
        p += sizeof(blob);
        if (blob < 0 || p + static_cast<size_t>(blob) > buf.size())
            return out;
        p += static_cast<size_t>(blob);
    }

    struct PhyToken {
        std::string text;
        size_t begin = 0;
        size_t end = 0;
    };
    std::vector<PhyToken> toks;
    while (p < buf.size() && buf[p]) {
        const size_t begin = p;
        const char c = buf[p];
        if (std::isspace(static_cast<unsigned char>(c))) {
            ++p;
        } else if (c == '"') {
            size_t e = p + 1;
            while (e < buf.size() && buf[e] && buf[e] != '"')
                ++e;
            if (e >= buf.size() || !buf[e])
                return out;
            p = e + 1;
            toks.push_back({std::string(&buf[begin + 1], e - begin - 1), begin, p});
        } else if (c == '{' || c == '}') {
            ++p;
            toks.push_back({std::string(1, c), begin, p});
        } else {
            size_t e = p;
            while (e < buf.size() && buf[e] && !std::isspace(static_cast<unsigned char>(buf[e])) &&
                   buf[e] != '{' && buf[e] != '}')
                ++e;
            p = e;
            toks.push_back({std::string(&buf[begin], e - begin), begin, p});
        }
    }

    for (size_t i = 0; i + 1 < toks.size();) {
        if (toks[i + 1].text != "{") {
            ++i;
            continue;
        }
        PhySection s;
        s.name = toks[i].text;
        const size_t first = i;
        size_t cursor = i + 2;
        int depth = 1;
        while (cursor < toks.size() && depth > 0) {
            if (toks[cursor].text == "{") {
                depth++;
                cursor++;
            } else if (toks[cursor].text == "}") {
                depth--;
                cursor++;
            } else if (depth == 1 && cursor + 1 < toks.size() &&
                       toks[cursor + 1].text != "{" && toks[cursor + 1].text != "}") {
                s.kv.emplace_back(toks[cursor].text, toks[cursor + 1].text);
                cursor += 2;
            } else {
                cursor++;
            }
        }
        if (depth != 0)
            return out;
        s.raw.assign(&buf[toks[first].begin], toks[cursor - 1].end - toks[first].begin);
        out.push_back(std::move(s));
        i = cursor;
    }
    return out;
}

void EmitKeyValues(Qc& q, const std::string& text, std::string indent,
                   bool normalizeModelPaths = false);

// $physicsmodel. The hulls go out as a DMX collision mesh beside the render
// meshes; everything else round-trips out of the .phy's text tail.
void WritePhysics(Qc& q, const Mdl& m, const std::string& mdlPath, const std::string& dir) {
    const std::vector<PhySection> secs = ReadPhy(mdlPath);
    std::vector<const PhySection*> solids;
    std::vector<const PhySection*> collisionText;
    const PhySection* edit = nullptr;
    for (const PhySection& s : secs) {
        if (s.name == "solid")
            solids.push_back(&s);
        else if (s.name == "editparams")
            edit = &s;
        else if (s.name != "ragdollconstraint" && s.name != "collisionrules" &&
                 s.name != "animatedfriction")
            collisionText.push_back(&s);
    }
    if (solids.empty())
        return;
    auto solidName = [&](int i) {
        return (i >= 0 && static_cast<size_t>(i) < solids.size()) ? solids[i]->Get("name")
                                                                  : std::string();
    };

    std::printf("\nphysics:\n");
    const std::string meshName = BaseName(StripExt(mdlPath)) + "_physics";
    const PhysicsMeshInfo phys = WritePhysicsMesh(m, mdlPath, dir, meshName);

    const bool concave = phys.concave || (edit && edit->Get("concave") == "1");
    q.Blank();
    // stock splits the command by archetype: several solids is a ragdoll
    if (g_studiomdl) {
        q.Line((solids.size() > 1 ? "$collisionjoints \"" : "$collisionmodel \"") +
               MeshFile(meshName) + "\"");
        q.Line("{");
    } else {
        q.Line("$physicsmodel {");
    }
    if (!phys.written)
        q.Line("    // the .phy's hulls could not be read - this file has to be supplied");
    if (g_studiomdl) {
        if (concave)
            q.Line("    $concave");
    } else {
        // importtype perjoint is the default; concave is the only thing that
        // still needs a block
        const std::string shape = "    $physicsshape fromfile \"" + MeshFile(meshName) + "\"";
        if (!concave) {
            q.Line(shape);
        } else {
            q.Line(shape + " {");
            q.Line("        concave");
            q.Line("    }");
        }
        q.Blank();
    }

    const float totalmass = edit ? edit->Getf("totalmass", 1.0f) : 1.0f;
    q.Line(totalmass < 0.0f ? "    $automass" : "    $mass " + F(totalmass));

    // The model-wide values are whatever body 0 got; a body that differs gets a
    // $physicsmarkup below. A differing rotdamping may be the compiler's own
    // long-body adjustment rather than something authored - restating it as
    // markup reproduces it either way, since markup is applied last.
    q.Line("    $damping " + F(solids[0]->Getf("damping")));
    q.Line("    $rotdamping " + F(solids[0]->Getf("rotdamping")));
    q.Line("    $inertia " + F(solids[0]->Getf("inertia")));
    if (solids[0]->Find("drag"))
        q.Line("    $drag " + F(solids[0]->Getf("drag")));

    for (const PhySection& s : secs)
        if (s.name == "collisionrules" && s.Get("selfcollisions") == "0")
            q.Line("    $noselfcollisions");

    if (edit && !edit->Get("rootname").empty()) {
        q.Blank();
        q.Line("    $rootbone \"" + edit->Get("rootname") + "\"");
    }

    // Everything per-joint is grouped under the bone it belongs to - markup,
    // constraints, then the pairs it opens - rather than one run per command.
    // A ragdoll is read and edited bone by bone.
    std::vector<std::string> order;
    std::map<std::string, std::vector<std::string>> byBone;
    for (const PhySection* s : solids)
        order.push_back(s->Get("name"));
    auto add = [&](const std::string& bone, std::string line) {
        if (byBone.find(bone) == byBone.end() &&
            std::find(order.begin(), order.end(), bone) == order.end())
            order.push_back(bone);
        byBone[bone].push_back(std::move(line));
    };

    for (const PhySection* s : solids) {
        std::vector<std::string> lines;
        static const char* kPer[] = {"massbias", "inertia", "damping", "rotdamping"};
        for (const char* k : kPer) {
            const std::string* v = s->Find(k);
            // massbias is only written when it is not 1, so its presence alone
            // is the signal; the rest are always written and have to be compared
            if (!v || (std::strcmp(k, "massbias") != 0 &&
                       s->Getf(k) == solids[0]->Getf(k)))
                continue;
            lines.push_back(std::string(k) + " " + F(s->Getf(k)));
        }
        // one command per value, no block - stock spells it $joint<field>
        for (const std::string& l : lines) {
            const size_t sp = l.find(' ');
            add(s->Get("name"),
                g_studiomdl
                    ? "    $joint" + l.substr(0, sp) + " \"" + s->Get("name") + "\"" + l.substr(sp)
                    : "    $physicsmarkup \"" + s->Get("name") + "\" " + l);
        }
    }

    // "a,b" = the bone b was merged into a
    if (edit)
        for (const auto& kv : edit->kv) {
            if (kv.first != "jointmerge")
                continue;
            const size_t comma = kv.second.find(',');
            if (comma == std::string::npos)
                continue;
            add(kv.second.substr(comma + 1),
                g_studiomdl ? "    $jointmerge \"" + kv.second.substr(0, comma) + "\" \"" +
                                  kv.second.substr(comma + 1) + "\""
                            : "    $physicsmarkup \"" + kv.second.substr(comma + 1) +
                                  "\" mergeinto \"" + kv.second.substr(0, comma) + "\"");
        }

    static const char* kAxis[3][4] = {{"x", "xmin", "xmax", "xfriction"},
                                      {"y", "ymin", "ymax", "yfriction"},
                                      {"z", "zmin", "zmax", "zfriction"}};
    for (const PhySection& s : secs) {
        if (s.name != "ragdollconstraint")
            continue;
        const std::string joint = solidName(std::atoi(s.Get("child").c_str()));
        // stock reads the limits from the args either way, so `limit` always
        // reproduces the stored pair - free/fixed are only spelling
        if (g_studiomdl) {
            for (const auto& a : kAxis)
                add(joint, "    $jointconstrain \"" + joint + "\" " + a[0] + " limit " +
                               F(s.Getf(a[1])) + " " + F(s.Getf(a[2])) + " " + F(s.Getf(a[3])));
            continue;
        }
        for (const auto& a : kAxis) {
            const float lo = s.Getf(a[1]), hi = s.Getf(a[2]), fr = s.Getf(a[3]);
            // an axis the script never named was zero-filled, which is exactly
            // what `fixed` writes - so it comes back as fixed
            const bool fixed = lo == 0.0f && hi == 0.0f && fr == 0.0f;
            std::string line = "    $physicsjoint \"" + joint + "\" " + a[0];
            if (lo == -360.0f && hi == 360.0f)
                line += " free";
            else if (fixed)
                line += " fixed";
            else
                line += " limit " + F(lo) + " " + F(hi);
            // a fixed axis zeroes its own friction, and an omitted one is 1 -
            // so anything else has to be written out as the trailing number
            if (!fixed && fr != 1.0f)
                line += " " + F(fr);
            add(joint, line);
        }
    }

    // cosmetic: emit the groups in skeleton order, root before its children
    const std::vector<std::string> boneNames = BoneNames(m);
    auto boneIndex = [&](const std::string& n) {
        const auto it = std::find(boneNames.begin(), boneNames.end(), n);
        return static_cast<size_t>(it - boneNames.begin());
    };
    std::stable_sort(order.begin(), order.end(),
                     [&](const std::string& a, const std::string& b) {
                         return boneIndex(a) < boneIndex(b);
                     });

    for (const std::string& bone : order) {
        const auto it = byBone.find(bone);
        if (it == byBone.end())
            continue;
        q.Blank();
        for (const std::string& l : it->second)
            q.Line(l);
    }

    // the pairs name two bones each, so they belong to neither group - they run
    // as one list after them
    bool anyPair = false;
    for (const PhySection& s : secs) {
        if (s.name != "collisionrules")
            continue;
        for (const auto& kv : s.kv) {
            if (kv.first != "collisionpair")
                continue;
            const size_t comma = kv.second.find(',');
            if (comma == std::string::npos)
                continue;
            if (!anyPair) {
                q.Blank();
                anyPair = true;
            }
            q.Line(std::string(g_studiomdl ? "    $jointcollide \"" : "    $physicscollide \"") +
                   solidName(std::atoi(kv.second.c_str())) + "\" \"" +
                   solidName(std::atoi(kv.second.c_str() + comma + 1)) + "\"");
        }
    }
    if (anyPair || !byBone.empty())
        q.Blank();

    for (const PhySection& s : secs)
        if (s.name == "animatedfriction")
            q.Line("    $animatedfriction " +
                   std::to_string(static_cast<int>(s.Getf("animfrictionmin"))) + " " +
                   std::to_string(static_cast<int>(s.Getf("animfrictionmax"))) + " " +
                   F(s.Getf("animfrictiontimein")) + " " + F(s.Getf("animfrictiontimeout")) + " " +
                   F(s.Getf("animfrictiontimehold")));

    q.Line("}");

    if (!collisionText.empty()) {
        q.Blank();
        q.Line("$collisiontext {");
        for (const PhySection* section : collisionText)
            EmitKeyValues(q, section->raw, "    ", true);
        q.Line("}");
    }
}

// $poseparameter <name> <min> <max> [wrap | loop <v>]. "wrap" is just a loop of
// exactly the range, so it is spelled back that way when it fits.
void WritePoseParams(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioposeparamdesc_t* pp = m.At<fm::mstudioposeparamdesc_t>(
        m.buf.data(), h.localposeparamindex, h.numlocalposeparameters);
    if (!pp || h.numlocalposeparameters <= 0)
        return;
    q.Blank();
    for (int i = 0; i < h.numlocalposeparameters; ++i) {
        std::string line = "$poseparameter \"" + std::string(m.Str(&pp[i], pp[i].sznameindex)) +
                           "\" " + F(pp[i].start) + " " + F(pp[i].end);
        if (pp[i].flags & fm::STUDIO_LOOPING)
            line += (pp[i].loop == pp[i].end - pp[i].start) ? " wrap" : " loop " + F(pp[i].loop);
        q.Line(line);
    }
}

std::vector<std::string> IkChainNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const fm::mstudioikchain_t* c =
        m.At<fm::mstudioikchain_t>(m.buf.data(), h.ikchainindex, h.numikchains);
    for (int i = 0; c && i < h.numikchains; ++i)
        names.push_back(m.Str(&c[i], c[i].sznameindex));
    return names;
}

std::vector<std::string> PoseParamNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const fm::mstudioposeparamdesc_t* p = m.At<fm::mstudioposeparamdesc_t>(
        m.buf.data(), h.localposeparamindex, h.numlocalposeparameters);
    for (int i = 0; p && i < h.numlocalposeparameters; ++i)
        names.push_back(m.Str(&p[i], p[i].sznameindex));
    return names;
}

// transition graph nodes. The name offsets are relative to the file start, and
// a sequence's entry/exit node is 1-based.
std::vector<std::string> NodeNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const int32_t* off = m.At<int32_t>(m.buf.data(), h.localnodenameindex, h.numlocalnodes);
    for (int i = 0; off && i < h.numlocalnodes; ++i)
        names.push_back(m.Str(m.buf.data(), off[i]));
    return names;
}

// $ikchain / $ikautoplaylock. The chain's height/pad/floor/center clauses are
// never written to the .mdl - they only feed the ik rules inside sequences -
// so only the end bone and the knee direction come back.
void WriteIk(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioikchain_t* chains =
        m.At<fm::mstudioikchain_t>(m.buf.data(), h.ikchainindex, h.numikchains);
    if (!chains || h.numikchains <= 0)
        return;
    const std::vector<std::string> boneNames = BoneNames(m);
    auto pick = [&](int32_t i) {
        return (i >= 0 && static_cast<size_t>(i) < boneNames.size()) ? boneNames[i]
                                                                     : std::string();
    };

    const std::vector<std::string> chainNames = IkChainNames(m);
    q.Blank();
    for (int i = 0; i < h.numikchains; ++i) {
        const fm::mstudioiklink_t* links =
            m.At<fm::mstudioiklink_t>(&chains[i], chains[i].linkindex, chains[i].numlinks);
        // link[2] is the bone the script named; [1]/[0] are its parents
        std::string line = "$ikchain \"" + chainNames[i] + "\" \"" +
                           (links && chains[i].numlinks >= 3 ? pick(links[2].bone) : std::string()) +
                           "\"";
        // an unauthored knee is derived from the animations, so writing the
        // stored value keeps the result reproducible without them
        if (links && chains[i].numlinks >= 1 &&
            (links[0].kneeDir.x || links[0].kneeDir.y || links[0].kneeDir.z))
            line += " knee " + V3(links[0].kneeDir);
        q.Line(line);
    }

    const fm::mstudioiklock_t* locks =
        m.At<fm::mstudioiklock_t>(m.buf.data(), h.localikautoplaylockindex,
                                  h.numlocalikautoplaylocks);
    if (!locks || h.numlocalikautoplaylocks <= 0)
        return;
    q.Blank();
    for (int i = 0; i < h.numlocalikautoplaylocks; ++i) {
        const std::string name =
            (locks[i].chain >= 0 && static_cast<size_t>(locks[i].chain) < chainNames.size())
                ? chainNames[locks[i].chain]
                : std::string();
        q.Line("$ikautoplaylock \"" + name + "\" " + F(locks[i].flPosWeight) + " " +
               F(locks[i].flLocalQWeight));
    }
}

void WriteAttachments(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioattachment_t* atts =
        m.At<fm::mstudioattachment_t>(m.buf.data(), h.localattachmentindex, h.numlocalattachments);
    if (!atts || h.numlocalattachments <= 0)
        return;
    const std::vector<std::string> names = BoneNames(m);

    const int illumAtt = IllumAttachment(m);

    q.Blank();
    for (int i = 0; i < h.numlocalattachments; ++i) {
        if (i == illumAtt) // $illumposition re-emits it
            continue;
        // `rigid` / `absolute` are consumed at compile time and are not in the
        // file; the baked matrix reproduces the same result without them.
        pm::RadianEuler rot;
        pm::Vector3 pos;
        pm::MatrixAngles(atts[i].local, rot, pos);
        const std::string bone =
            (atts[i].localbone >= 0 && static_cast<size_t>(atts[i].localbone) < names.size())
                ? names[atts[i].localbone]
                : std::string();
        // stock takes the position bare after the bone and spells the rotation
        // `rotate`; both are the same QAngle either way
        std::string line = "$attachment \"" + std::string(m.Str(&atts[i], atts[i].sznameindex)) +
                           "\" \"" + bone + "\" " + (g_studiomdl ? "" : "origin ") + V3(pos) +
                           (g_studiomdl ? " rotate " : " angles ") + QAngleDeg(rot);
        if (atts[i].flags & 0x10000u) // ATTACHMENT_FLAG_WORLD_ALIGN
            line += " world_align";
        q.Line(line);
    }
}

// --- eyes / flex ------------------------------------------------------------

// An eyeball plus the two things its own struct does not carry: the eye
// material (sznameindex and texture are never written - they stay zero) and a
// synthesized name, since $eyeball and $eyelid both have to say one.
struct EyeballRef {
    const fm::mstudioeyeball_t* e;
    std::string material;
    std::string name;
    pm::Vector3 origin; // script space, what $eyeball origin took
};

// SetupEyeballs clones one authored $eyeball onto every model whose mesh
// carries the eye material, so the same eyeball reappears once per body. With
// no name to key on, identical bone+org+radius means it is the same one.
std::vector<EyeballRef> GatherEyeballs(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const std::vector<std::string> texNames = TextureNames(m);
    std::vector<EyeballRef> eyes;
    std::set<std::string> seen;

    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    for (int i = 0; parts && i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            const fm::mstudioeyeball_t* eb = m.At<fm::mstudioeyeball_t>(
                &models[j], models[j].eyeballindex, models[j].numeyeballs);
            const fm::mstudiomesh_t* meshes =
                m.At<fm::mstudiomesh_t>(&models[j], models[j].meshindex, models[j].nummeshes);
            for (int k = 0; eb && k < models[j].numeyeballs; ++k) {
                const std::string key = std::to_string(eb[k].bone) + "|" + V3(eb[k].org) + "|" +
                                        F(eb[k].radius);
                if (!seen.insert(key).second)
                    continue;
                EyeballRef ref{&eb[k], std::string(), std::string(), {}};
                // the owning mesh is the one WriteModel tagged as a custom
                // (eyeball) material, keyed by this eyeball's ordinal
                for (int n = 0; meshes && n < models[j].nummeshes; ++n)
                    if (meshes[n].materialtype == 1 && meshes[n].materialparam == k &&
                        meshes[n].material >= 0 &&
                        static_cast<size_t>(meshes[n].material) < texNames.size()) {
                        ref.material = texNames[meshes[n].material];
                        break;
                    }
                eyes.push_back(ref);
            }
        }
    }
    return eyes;
}

// `faceMesh` is the .dmx the lid deltas were written into - dmxeyelid names it.
void WriteEyes(Qc& q, const Mdl& m, const std::string& faceMesh) {
    std::vector<EyeballRef> eyes = GatherEyeballs(m);
    if (eyes.empty())
        return;
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    const std::vector<std::string> boneNames = BoneNames(m);
    const std::vector<std::string> descs = FlexDescNames(m);
    auto pick = [](const std::vector<std::string>& v, int32_t i) {
        return (i >= 0 && static_cast<size_t>(i) < v.size()) ? v[i] : std::string();
    };

    // org is bone space; the script wrote it in the source's own space, so push
    // it back out through the bone (SetupEyeballs' VectorITransform, inverted).
    for (EyeballRef& r : eyes) {
        r.origin = r.e->org;
        if (bones && r.e->bone >= 0 && r.e->bone < h.numbones)
            r.origin = pm::VectorITransform(r.e->org, bones[r.e->bone].poseToBone);
    }

    // No name survives the compile, and both $eyeball and $eyelid need one.
    // Default to an ordinal; only a clean two-eye pair sitting on opposite
    // sides earns a side name. Script X becomes model Y under the default +90
    // yaw, so negative X is the model's right.
    const bool sided = eyes.size() == 2 && eyes[0].origin.x * eyes[1].origin.x < 0.0f;
    for (size_t i = 0; i < eyes.size(); ++i)
        eyes[i].name = sided ? (eyes[i].origin.x < 0.0f ? "right_eye" : "left_eye")
                             : "eyeball_" + std::to_string(i);

    q.Blank();
    for (const EyeballRef& r : eyes) {
        // stock: eyeball <name> <bone> <x y z> <material> <diameter> <angle>
        // <iris material, read and discarded> <pupil scale>
        if (g_studiomdl) {
            q.Line("eyeball \"" + r.name + "\" \"" + pick(boneNames, r.e->bone) + "\" " +
                   V3(r.origin) + " \"" + BaseName(r.material) + "\" " + F(r.e->radius * 2.0f) +
                   " " + F(std::atan(r.e->zoffset) * pm::kRad2Deg) + " \"iris_unused\" " +
                   F(r.e->iris_scale != 0.0f ? 1.0f / r.e->iris_scale : 1.0f));
            continue;
        }
        std::string line = "$eyeball \"" + r.name + "\" bone \"" + pick(boneNames, r.e->bone) +
                           "\" origin " + V3(r.origin) + " diameter " + F(r.e->radius * 2.0f) +
                           " angle " + F(std::atan(r.e->zoffset) * pm::kRad2Deg);
        if (r.e->iris_scale != 0.0f)
            line += " pupilscale " + F(1.0f / r.e->iris_scale);
        q.Line(line + " material \"" + BaseName(r.material) + "\"");
    }

    // Which lid poses carry vertex data, per lid flexdesc, and whether the file
    // splits them stereo (one desc pair for both eyes, the DMX form) or mono
    // (one desc per eye, what every VTA-era v44-48 model has).
    const std::set<int> lids = LidDescs(m);
    std::map<int, int> lidSlots; // flexdesc -> bitmask of poses with vertex data
    bool lidStereo = false;
    ForEachFlex(m, [&](const fm::mstudioflex_t& fx) {
        if (!lids.count(fx.flexdesc))
            return;
        lidSlots[fx.flexdesc] |= 1 << LidSlot(fx);
        lidStereo = lidStereo || fx.flexpair > 0;
    });
    auto slotsOf = [&](int32_t d) {
        const auto it = lidSlots.find(d);
        return it == lidSlots.end() ? 0 : it->second;
    };

    bool noted = false;
    for (int upper = 1; upper >= 0; --upper) {
        const std::string type = upper ? "upper" : "lower";
        auto lidDesc = [&](const EyeballRef& r) {
            return upper ? r.e->upperlidflexdesc : r.e->lowerlidflexdesc;
        };
        // The lid poses share a desc, so each is named per slot; a slot with no
        // vertex data (the VTA neutral is usually the base frame) writes "-".
        auto poses = [&](const EyeballRef& r, const std::string& base, int have) {
            const float* target = upper ? r.e->uppertarget : r.e->lowertarget;
            std::string s;
            for (int i = 0; i < 3; ++i)
                s += std::string(" ") + kLidSlot[i] + " " +
                     (((have >> i) & 1) ? "\"" + LidDeltaName(base, i) + "\"" : "-") + " " +
                     F(target[i]);
            return s;
        };
        auto blank = [&] {
            if (!noted) {
                q.Blank();
                noted = true;
            }
        };
        // stock takes ONE delta per slot and splits it L/R by balance, so both
        // eyes ride one command and the deltas are the merged per-lid ones
        // dmxwrite wrote. It creates its own flexdescs (upper_left, ...), so a
        // model whose lid descs are named otherwise loses the rules fetching them.
        if (g_studiomdl) {
            const EyeballRef* side[2] = {nullptr, nullptr}; // [0] left, [1] right
            for (const EyeballRef& r : eyes)
                if (const int s = r.origin.x < 0.0f ? 1 : 0; !side[s])
                    side[s] = &r;
            if (!side[0] || !side[1] || faceMesh.empty() ||
                (!slotsOf(lidDesc(*side[0])) && !slotsOf(lidDesc(*side[1]))))
                continue;
            const float* lt = upper ? side[0]->e->uppertarget : side[0]->e->lowertarget;
            const float* rt = upper ? side[1]->e->uppertarget : side[1]->e->lowertarget;
            std::string body;
            // one target per slot, so the two eyes' windows meet in the middle
            for (int i = 0; i < 3; ++i)
                body += std::string(" ") + kLidSlot[i] + " \"" + LidDeltaName(type, i) + "\" " +
                        F(0.5f * (lt[i] + rt[i]));
            blank();
            q.Line("dmxeyelid " + type + " \"" + faceMesh + "\"" + body + " righteyeball \"" +
                   side[1]->name + "\" lefteyeball \"" + side[0]->name + "\"");
            continue;
        }

        if (lidStereo) {
            // one command for both eyes; pair them by which side's flexdesc
            // each eyeball's lid base points at
            const EyeballRef* side[2] = {nullptr, nullptr}; // [0] left, [1] right
            for (const EyeballRef& r : eyes) {
                const std::string n = pick(descs, lidDesc(r));
                int s = -1; // unauthored lids got the shared "dummy_eyelid" desc
                if (n.size() >= 6 && n.compare(n.size() - 6, 6, "_right") == 0)
                    s = 1;
                else if (n.size() >= 5 && n.compare(n.size() - 5, 5, "_left") == 0)
                    s = 0;
                if (s >= 0 && !side[s])
                    side[s] = &r;
            }
            if (!side[0] || !side[1])
                continue;
            // the split's flexdesc is the LEFT desc, so that is what the deltas
            // were written under
            const std::string base = pick(descs, lidDesc(*side[0]));
            blank();
            q.Line("$eyelid " + type + poses(*side[1], base, slotsOf(lidDesc(*side[0]))) +
                   " righteyeball \"" + side[1]->name + "\" lefteyeball \"" + side[0]->name +
                   "\"");
        } else {
            for (const EyeballRef& r : eyes) {
                const int have = slotsOf(lidDesc(r));
                if (!have)
                    continue;
                const std::string base = pick(descs, lidDesc(r));
                blank();
                q.Line("$eyelid " + type + " flexdesc \"" + base + "\"" + poses(r, base, have) +
                       " eyeball \"" + r.name + "\"");
            }
        }
    }
}

// One partial expression plus the precedence of its top operator, so the
// rebuild only parenthesizes where it changes meaning.
struct Expr {
    std::string s;
    int prec; // 1 = +-, 2 = */, 3 = atom
};

// RPN op stream -> the infix text $flexrule takes. Fails on the ops that have
// no spelling in an expression - combo/dominate/nway/2way/eyelid come from the
// DMX rig ($datamodelflexes) or $flexcorrective, never from rule text.
bool RebuildExpr(const fm::mstudioflexop_t* ops, int n, const std::vector<std::string>& ctrls,
                 const std::vector<std::string>& descs, std::string& out) {
    std::vector<Expr> st;
    auto name = [](const std::vector<std::string>& v, int32_t i) {
        return (i >= 0 && static_cast<size_t>(i) < v.size()) ? v[i] : std::string("?");
    };
    auto bin = [&](const char* sym, int prec) {
        if (st.size() < 2)
            return false;
        const Expr b = st.back();
        st.pop_back();
        const Expr a = st.back();
        st.pop_back();
        st.push_back({(a.prec < prec ? "(" + a.s + ")" : a.s) + " " + sym + " " +
                          (b.prec <= prec ? "(" + b.s + ")" : b.s),
                      prec});
        return true;
    };
    auto fn = [&](const char* sym) {
        if (st.size() < 2)
            return false;
        const Expr b = st.back();
        st.pop_back();
        const Expr a = st.back();
        st.pop_back();
        st.push_back({std::string(sym) + "(" + a.s + ", " + b.s + ")", 3});
        return true;
    };

    for (int i = 0; i < n; ++i) {
        bool ok = true;
        switch (ops[i].op) {
            // a bare leading '-' lexes as subtraction, so a negative constant
            // is parenthesized to put it after '(' where it reads as unary
            case fm::STUDIO_CONST: {
                const std::string v = F(ops[i].d.value);
                st.push_back({v[0] == '-' ? "(" + v + ")" : v, 3});
                break;
            }
            case fm::STUDIO_FETCH1: st.push_back({name(ctrls, ops[i].d.index), 3}); break;
            case fm::STUDIO_FETCH2: st.push_back({"%" + name(descs, ops[i].d.index), 3}); break;
            case fm::STUDIO_ADD: ok = bin("+", 1); break;
            case fm::STUDIO_SUB: ok = bin("-", 1); break;
            case fm::STUDIO_MUL: ok = bin("*", 2); break;
            case fm::STUDIO_DIV: ok = bin("/", 2); break;
            case fm::STUDIO_MAX: ok = fn("max"); break;
            case fm::STUDIO_MIN: ok = fn("min"); break;
            case fm::STUDIO_NEG:
                if (st.empty())
                    return false;
                // always wrapped: the lexer only reads '-' as unary after "(+-*/,"
                st.back() = {"(-" + st.back().s + ")", 3};
                break;
            default: return false; // combo / dominate / nway / 2way / eyelid
        }
        if (!ok)
            return false;
    }
    if (st.size() != 1)
        return false;
    out = st.front().s;
    return true;
}

// Which flexdescs already exist by the time the rules are read - only the ones
// a morph's vertex data brings in, which land when the mesh is loaded. The
// $eyeball/$eyelid/$mouth descs are created later, so a rule naming one still
// needs a $flexlocalvar ahead of it or it resolves against nothing.
std::vector<bool> DescHasGeometry(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<bool> used(h.numflexdesc > 0 ? h.numflexdesc : 0, false);
    auto mark = [&](int d) {
        if (d >= 0 && d < h.numflexdesc)
            used[d] = true;
    };

    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    for (int i = 0; parts && i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            const fm::mstudiomesh_t* meshes =
                m.At<fm::mstudiomesh_t>(&models[j], models[j].meshindex, models[j].nummeshes);
            for (int k = 0; meshes && k < models[j].nummeshes; ++k) {
                const fm::mstudioflex_t* fx =
                    m.At<fm::mstudioflex_t>(&meshes[k], meshes[k].flexindex, meshes[k].numflexes);
                for (int n = 0; fx && n < meshes[k].numflexes; ++n) {
                    mark(fx[n].flexdesc);
                    if (fx[n].flexpair > 0)
                        mark(fx[n].flexpair);
                }
            }
        }
    }
    return used;
}

void WriteFlexes(Qc& q, const Mdl& m, const FlexRig& rig, const std::string& faceMesh) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioflexcontroller_t* fc =
        m.At<fm::mstudioflexcontroller_t>(m.buf.data(), h.flexcontrollerindex, h.numflexcontrollers);

    // The combination rig went back into the mesh .dmx, so the controllers and
    // rules it rebuilds are imported rather than written out again here. Only
    // the .pulseqc can import one; stock reads the operator off the mesh itself.
    const bool importRig = !g_studiomdl && !rig.empty() && !faceMesh.empty();
    if (importRig) {
        q.Blank();
        q.Line("$datamodelflexes \"" + faceMesh +
               "\" flexcontroller flexcorrective flexdominator");
        q.Line("// " + std::to_string(rig.correctives.size()) + " correctives and " +
               std::to_string(rig.dominations.size()) +
               " domination rules, rebuilt from the flex rules into that file");
        if (rig.dropped)
            q.Line("// " + std::to_string(rig.dropped) +
                   " rules did not decode as a combination and are commented out below");
        if (rig.domMismatch)
            q.Line("// " + std::to_string(rig.domMismatch) +
                   " correctives get a different dominator set than the model had - the rules "
                   "are\n// per-combination, and one written against a subset also hits its "
                   "supersets");
    }

    std::vector<std::string> ctrls;
    if (fc && h.numflexcontrollers > 0) {
        q.Blank();
        // stock auto-creates a controller per combination control on top of the
        // real ones below, which double-registers them. This keeps the deltas
        // and the rig in the .dmx and drops only that.
        if (g_studiomdl)
            q.Line("noautodmxrules");
        for (int i = 0; i < h.numflexcontrollers; ++i) {
            ctrls.push_back(m.Str(&fc[i], fc[i].sznameindex));
            if (importRig && rig.controllers.count(i))
                continue; // the imported controls recreate this one
            std::string line = (g_studiomdl ? "flexcontroller " : "$flexcontroller ") +
                               std::string(m.Str(&fc[i], fc[i].sztypeindex));
            if (fc[i].min != 0.0f || fc[i].max != 1.0f)
                line += " range " + F(fc[i].min) + " " + F(fc[i].max);
            q.Line(line + " \"" + ctrls.back() + "\"");
        }
    }

    const std::vector<std::string> descs = FlexDescNames(m);
    const fm::mstudioflexdesc_t* fd =
        m.At<fm::mstudioflexdesc_t>(m.buf.data(), h.flexdescindex, h.numflexdesc);
    int renamed = 0;
    for (int i = 0; fd && i < h.numflexdesc; ++i)
        renamed += (descs[i] != m.Str(&fd[i], fd[i].szFACSindex));
    if (renamed) {
        q.Blank();
        q.Line("// " + std::to_string(renamed) + " of " + std::to_string(h.numflexdesc) +
               " flex names were junk or duplicated - renamed flex<index>, real names are lost");
    }

    const fm::mstudiomouth_t* mouths =
        m.At<fm::mstudiomouth_t>(m.buf.data(), h.mouthindex, h.nummouths);
    if (mouths && h.nummouths > 0) {
        const std::vector<std::string> bones = BoneNames(m);
        q.Blank();
        for (int i = 0; i < h.nummouths; ++i) {
            // the mouth's flexdesc was added under the controller name it was written with
            const std::string ctrl =
                (mouths[i].flexdesc >= 0 && static_cast<size_t>(mouths[i].flexdesc) < descs.size())
                    ? descs[mouths[i].flexdesc]
                    : std::string();
            const std::string bone =
                (mouths[i].bone >= 0 && static_cast<size_t>(mouths[i].bone) < bones.size())
                    ? bones[mouths[i].bone]
                    : std::string();
            // stock takes the mouth's own index first
            q.Line(g_studiomdl ? "mouth " + std::to_string(i) + " \"" + ctrl + "\" \"" + bone +
                                     "\" " + V3(mouths[i].forward)
                               : "$mouth \"" + ctrl + "\" \"" + bone + "\" " +
                                     V3(mouths[i].forward));
        }
    }

    const fm::mstudioflexrule_t* rules =
        m.At<fm::mstudioflexrule_t>(m.buf.data(), h.flexruleindex, h.numflexrules);
    if (!rules || h.numflexrules <= 0)
        return;

    // A desc no morph drives and no face markup created exists only because a
    // rule names it - as its result or as a %fetch operand. $flexrule needs the
    // desc to already be there, so those are reserved with $flexlocalvar.
    const std::vector<bool> geo = DescHasGeometry(m);
    std::vector<std::string> localvars;
    std::set<int> seenVar;
    auto reserve = [&](int d) {
        if (d < 0 || d >= h.numflexdesc || geo[d] || !seenVar.insert(d).second)
            return;
        localvars.push_back(descs[d]);
    };
    for (int i = 0; i < h.numflexrules; ++i) {
        reserve(rules[i].flex);
        const fm::mstudioflexop_t* ops =
            m.At<fm::mstudioflexop_t>(&rules[i], rules[i].opindex, rules[i].numops);
        for (int k = 0; ops && k < rules[i].numops; ++k)
            if (ops[k].op == fm::STUDIO_FETCH2)
                reserve(ops[k].d.index);
    }
    if (!localvars.empty()) {
        q.Blank();
        for (const std::string& v : localvars)
            q.Line(g_studiomdl ? "localvar " + v : "$flexlocalvar \"" + v + "\"");
    }

    // A rule naming a desc the recompile cannot recreate goes out commented -
    // stock studiomdl tolerates a rule with no morph behind it, we do not, and
    // dropping the line would lose the rig for whoever reads the script.
    const std::vector<bool> reachable = DescIsReproducible(m, descs);
    auto known = [&](int32_t d) {
        return d >= 0 && static_cast<size_t>(d) < reachable.size() && reachable[d];
    };

    q.Blank();
    int skipped = 0, commented = 0;
    for (int i = 0; i < h.numflexrules; ++i) {
        if (importRig && rig.descs.count(rules[i].flex))
            continue; // the imported rig rebuilds this one
        const fm::mstudioflexop_t* ops =
            m.At<fm::mstudioflexop_t>(&rules[i], rules[i].opindex, rules[i].numops);
        const std::string target =
            (rules[i].flex >= 0 && static_cast<size_t>(rules[i].flex) < descs.size())
                ? descs[rules[i].flex]
                : std::string();
        std::string expr;
        if (!ops || !RebuildExpr(ops, rules[i].numops, ctrls, descs, expr)) {
            ++skipped;
            continue;
        }
        bool ok = known(rules[i].flex);
        for (int k = 0; ok && k < rules[i].numops; ++k)
            if (ops[k].op == fm::STUDIO_FETCH2 && !known(ops[k].d.index))
                ok = false;
        commented += !ok;
        // stock spells the target as one %-prefixed token, not a quoted name
        q.Line((ok ? "" : "// ") + (g_studiomdl ? "%" + target + " = " + expr
                                                : "$flexrule \"" + target + "\" = " + expr));
    }
    if (commented)
        q.Line("// " + std::to_string(commented) + " of " + std::to_string(h.numflexrules) +
               " rules name a morph whose real name was lost - commented out, they would not"
               "\n// compile. The expression is intact if you can work out what it drove.");
    if (skipped) {
        q.Line("// " + std::to_string(skipped) + " of " + std::to_string(h.numflexrules) +
               " flex rules use combo/dominate/nway/2way ops, which have no $flexrule");
        q.Line(importRig ? "// spelling and did not rebuild as a combination - they are lost."
                         : "// spelling - they come back from the DMX rig via $datamodelflexes.");
    }
}

// $includemodel. The loader prefixes "models/" on the way in, so it comes back
// off - re-adding it would compile to "models/models/...".
void WriteIncludeModels(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiomodelgroup_t* g =
        m.At<fm::mstudiomodelgroup_t>(m.buf.data(), h.includemodelindex, h.numincludemodels);
    if (!g || h.numincludemodels <= 0)
        return;
    q.Blank();
    for (int i = 0; i < h.numincludemodels; ++i) {
        std::string n = m.Str(&g[i], g[i].sznameindex);
        std::string lower = n.substr(0, 7);
        std::transform(lower.begin(), lower.end(), lower.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lower == "models/")
            n.erase(0, 7);
        q.Line("$includemodel \"" + n + "\"");
    }
}

// Re-indent a keyvalues1 blob: one pair per line, `key {` opening a block.
// Quoting is preserved as authored; a `//` comment runs to end of line.
void EmitKeyValues(Qc& q, const std::string& text, std::string indent,
                   bool normalizeModelPaths) {
    struct Tok { std::string s; bool quoted; };
    std::vector<Tok> t;
    for (size_t p = 0; p < text.size();) {
        const char c = text[p];
        if (std::isspace(static_cast<unsigned char>(c))) { ++p; continue; }
        if (c == '/' && p + 1 < text.size() && text[p + 1] == '/') {
            p = text.find('\n', p);
            if (p == std::string::npos) break;
            continue;
        }
        if (c == '"') {
            const size_t e = text.find('"', p + 1);
            if (e == std::string::npos) break;
            t.push_back({text.substr(p + 1, e - p - 1), true});
            p = e + 1;
        } else if (c == '{' || c == '}') {
            t.push_back({std::string(1, c), false});
            ++p;
        } else {
            const size_t e = text.find_first_of(" \t\r\n{}\"", p);
            t.push_back({text.substr(p, e - p), false});
            p = e == std::string::npos ? text.size() : e;
        }
    }

    auto spell = [](const Tok& k) { return k.quoted ? "\"" + k.s + "\"" : k.s; };
    for (size_t i = 0; i < t.size(); ++i) {
        if (t[i].s == "}" && !t[i].quoted) {
            if (indent.size() >= 4)
                indent.resize(indent.size() - 4);
            q.Line(indent + "}");
        } else if (i + 1 < t.size() && t[i + 1].s == "{" && !t[i + 1].quoted) {
            q.Line(indent + spell(t[i]) + " {");
            indent += "    ";
            ++i;
        } else if (i + 1 < t.size()) {
            Tok value = t[i + 1];
            const bool modelKey = t[i].s.size() == 5 &&
                std::equal(t[i].s.begin(), t[i].s.end(), "model",
                           [](char a, char b) {
                               return std::tolower(static_cast<unsigned char>(a)) == b;
                           });
            if (normalizeModelPaths && modelKey)
                std::replace(value.s.begin(), value.s.end(), '\\', '/');
            q.Line(indent + spell(t[i]) + " " + spell(value));
            ++i;
        } else {
            q.Line(indent + spell(t[i]));
        }
    }
}

// $keyvalues. The stored text is the block's contents wrapped in an outer
// "mdlkeyvalue { }" that the compiler puts back on write - strip it here or the
// wrapper nests one level deeper every round trip.
void WriteKeyValues(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const char* kv = m.At<char>(m.buf.data(), h.keyvalueindex, h.keyvaluesize);
    if (!kv || h.keyvaluesize <= 0)
        return;
    std::string text(kv, strnlen(kv, h.keyvaluesize));

    const size_t open = text.find('{'), close = text.rfind('}');
    if (text.compare(0, 11, "mdlkeyvalue") == 0 && open != std::string::npos &&
        close != std::string::npos && close > open)
        text = text.substr(open + 1, close - open - 1);

    const size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return;
    text = text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);

    q.Blank();
    q.Line("$keyvalues {");
    EmitKeyValues(q, text, "    ");
    q.Line("}");
}

// $defaultweightlist / $weightlist. A seqdesc keeps the resolved per-bone
// rotation weights, so the lists come back as values - the names they were
// authored under are not in the file, and neither are position weights (an
// entry with no `posweight` takes the weight, which is how most are written).
struct WeightLists {
    std::vector<std::pair<std::string, std::vector<float>>> lists;
    bool sawPlain = false; // some sequence kept the untouched all-ones default

    // "" when the array is that plain default, which needs no command
    std::string NameFor(const std::vector<float>& v) const {
        for (const auto& l : lists)
            if (l.second == v)
                return l.first;
        return std::string();
    }

    bool HasName(const std::string& n) const {
        for (const auto& l : lists)
            if (l.first == n)
                return true;
        return false;
    }

    // True when the sole list is emitted as $defaultweightlist - it auto-applies
    // to every sequence, so none gets (or references) a per-sequence weightlist.
    bool DefaultApplies() const { return lists.size() == 1 && !sawPlain; }
};

// Distinct arrays in first-use order. A $declaresequence slot has no animation
// behind it, so its all-zero array is not an authored list; an all-ones array is
// the untouched default and needs no command. Each list is named after the first
// sequence that uses it, the convention other decompilers settled on - the
// authored name itself is not in the file.
WeightLists GatherWeightLists(const Mdl& m) {
    WeightLists out;
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioseqdesc_t* seqs =
        m.At<fm::mstudioseqdesc_t>(m.buf.data(), h.localseqindex, h.numlocalseq);
    if (!seqs || std::strcmp(Archetype(m), "general") != 0)
        return out;

    for (int i = 0; i < h.numlocalseq; ++i) {
        if (seqs[i].flags & fm::STUDIO_OVERRIDE)
            continue;
        const float* w = m.At<float>(&seqs[i], seqs[i].weightlistindex, h.numbones);
        if (!w)
            continue;
        std::vector<float> v(w, w + h.numbones);
        if (std::all_of(v.begin(), v.end(), [](float f) { return f == 1.0f; })) {
            out.sawPlain = true;
            continue;
        }
        if (!out.NameFor(v).empty())
            continue;
        const std::string label = m.Str(&seqs[i], seqs[i].szlabelindex);
        std::string name =
            "weights_" + (CleanName(label) ? label : std::to_string(out.lists.size()));
        // two sequences can share a name but keep different weights - .mdl stores
        // values, not the authored list name, so uniquify or recompile duplicates.
        std::string base = name;
        for (int n = 2; out.HasName(name); ++n)
            name = base + "_" + std::to_string(n);
        out.lists.emplace_back(std::move(name), std::move(v));
    }
    return out;
}

void WriteWeightLists(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    const WeightLists w = GatherWeightLists(m);
    const auto& lists = w.lists;
    if (!bones || lists.empty())
        return;

    // Written as the entries that differ from what the resolver rebuilds: a
    // root seeds at 1 in the default list and 0 in a named one, and every other
    // bone inherits its parent's weight.
    const std::vector<std::string> names = BoneNames(m);
    auto body = [&](const std::vector<float>& w, float rootSeed) {
        for (int b = 0; b < h.numbones; ++b) {
            const int32_t p = bones[b].parent;
            const float pred = (p >= 0 && p < b) ? w[p] : rootSeed;
            // nonzero weights ride inheritance; a 0 is always spelled out so a
            // masked-out bone is never mistaken for an inherited default.
            if (w[b] != pred || w[b] == 0.0f)
                q.Line("    \"" + names[b] + "\" " + F(w[b]));
        }
    };

    q.Blank();
    // one list and no sequence left on the plain default means every sequence
    // shared it - which is what $defaultweightlist does
    if (w.DefaultApplies()) {
        q.Line("$defaultweightlist {");
        body(lists[0].second, 1.0f);
        q.Line("}");
        return;
    }
    for (const auto& l : lists) {
        q.Line("$weightlist \"" + l.first + "\" {");
        body(l.second, 0.0f);
        q.Line("}");
    }
}

// Where one clip's data ended up. Only the leading sections can stay in the
// .mdl - that is what `nostallframes` buys - so counting them is enough.
struct BlockUse {
    bool external = false;
    int localSections = 0;
};
BlockUse AnimBlockUse(const Mdl& m, const fm::mstudioanimdesc_t& a) {
    BlockUse u;
    const int frames = a.numframes > 0 ? a.numframes : 1;
    const int n = a.sectionframes > 0 ? frames / a.sectionframes + 2 : 0;
    const fm::mstudioanimsections_t* s =
        n ? m.At<fm::mstudioanimsections_t>(&a, a.sectionindex, n) : nullptr;
    if (!s) {
        u.external = a.animblock > 0;
        return u;
    }
    for (int i = 0; i < n; ++i) {
        if (s[i].animblock > 0) {
            u.external = true;
            break;
        }
        ++u.localSections;
    }
    return u;
}

// `highres` stores positions as full floats, and the only record of it is the
// per-bone flag byte array of a frame-anim section - which usually sits in the
// .ani, so that file is read too. Undetectable if the .ani is missing.
bool AnimBlockHighRes(const Mdl& m, const fm::mstudioanimdesc_t* a, int count, int numbones,
                      const fm::mstudioanimblock_t* blocks, int numblocks,
                      const std::vector<char>& ani) {
    for (int i = 0; i < count; ++i) {
        if (!(a[i].flags & fm::STUDIO_FRAMEANIM) || (a[i].flags & fm::STUDIO_ALLZEROS))
            continue;
        int32_t block = a[i].animblock, off = a[i].animindex;
        if (a[i].sectionframes > 0) {
            const fm::mstudioanimsections_t* s =
                m.At<fm::mstudioanimsections_t>(&a[i], a[i].sectionindex, 1);
            if (!s)
                continue;
            block = s->animblock;
            off = s->animindex;
        }
        const uint8_t* flags = nullptr;
        if (block == 0) {
            const fm::mstudio_frame_anim_t* fa = m.At<fm::mstudio_frame_anim_t>(&a[i], off);
            flags = fa ? m.At<uint8_t>(fa, static_cast<int32_t>(sizeof(*fa)), numbones) : nullptr;
        } else if (blocks && block < numblocks && !ani.empty()) {
            const int64_t at = static_cast<int64_t>(blocks[block].datastart) + off +
                               static_cast<int64_t>(sizeof(fm::mstudio_frame_anim_t));
            if (at >= 0 && at + numbones <= static_cast<int64_t>(ani.size()))
                flags = reinterpret_cast<const uint8_t*>(ani.data()) + at;
        }
        for (int j = 0; flags && j < numbones; ++j)
            if (flags[j] & (fm::STUDIO_FRAME_ANIM_POS2 | fm::STUDIO_FRAME_CONST_POS2))
                return true;
    }
    return false;
}

// $sectionframes / $animblocksize / $bonesaveframe - compression sectioning and
// demand loading. A .ani exists iff the block table has real entries; index 0 is
// the pseudo block meaning "in the .mdl".
void WriteAnimBlocks(Qc& q, const Mdl& m, const std::string& mdlPath) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* a =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    if (!a || h.numlocalanim <= 0)
        return;
    std::vector<std::string> lines;

    // Every clip at or over the frame limit was sectioned, all at the same
    // length. Any limit above the longest unsectioned clip and at or below the
    // shortest sectioned one reproduces the file, so prefer the default 30.
    int section = 0, sectioned = 0, plain = 0;
    bool frameanim = false;
    for (int i = 0; i < h.numlocalanim; ++i) {
        if (a[i].flags & fm::STUDIO_OVERRIDE)
            continue;
        frameanim |= (a[i].flags & fm::STUDIO_FRAMEANIM) != 0;
        const int frames = a[i].numframes > 0 ? a[i].numframes : 1;
        if (a[i].sectionframes > 0) {
            section = a[i].sectionframes;
            if (!sectioned || frames < sectioned)
                sectioned = frames;
        } else if (frames > plain) {
            plain = frames;
        }
    }
    const int limit = (section && plain < 30 && 30 <= sectioned) ? 30 : sectioned;
    if (section && (section != 30 || limit != 30))
        lines.push_back("$sectionframes " + std::to_string(section) + " " + std::to_string(limit));

    const fm::mstudioanimblock_t* blocks =
        m.At<fm::mstudioanimblock_t>(m.buf.data(), h.animblockindex, h.numanimblocks);
    if (blocks && h.numanimblocks > 1) {
        auto len = [&](int i) {
            const int64_t n = static_cast<int64_t>(blocks[i].dataend) - blocks[i].datastart;
            return n > 0 ? static_cast<size_t>(n) : size_t(0);
        };
        // A block closes BEFORE the section that would push it over
        // $animblocksize, so a closed block that took more than one section is a
        // lower bound on it - and blocks pack tight, so the largest such block
        // rounded up to the next KB lands on the authored value. A block holding
        // one oversized section overshoots the limit instead and is no evidence.
        const int last = h.numanimblocks - 1;
        std::vector<int> sections(static_cast<size_t>(h.numanimblocks), 0);
        for (int i = 0; i < h.numlocalanim; ++i) {
            if (a[i].flags & fm::STUDIO_OVERRIDE)
                continue;
            const int frames = a[i].numframes > 0 ? a[i].numframes : 1;
            const int n = a[i].sectionframes > 0 ? frames / a[i].sectionframes + 2 : 0;
            const fm::mstudioanimsections_t* s =
                n ? m.At<fm::mstudioanimsections_t>(&a[i], a[i].sectionindex, n) : nullptr;
            for (int w = 0; w < (s ? n : 1); ++w) {
                const int32_t blk = s ? s[w].animblock : a[i].animblock;
                if (blk > 0 && blk < h.numanimblocks)
                    ++sections[static_cast<size_t>(blk)];
            }
        }
        size_t est = 0;
        for (int i = 1; i < last; ++i)
            if (sections[static_cast<size_t>(i)] > 1 && len(i) > est)
                est = len(i);
        // no block ever took a second section: fall back to the longest one,
        // which is all the file still says
        if (!est)
            for (int i = 1; i <= last; ++i)
                est = std::max(est, len(i));
        const int kb = std::max<int>(1, static_cast<int>((est + 1023) / 1024));

        std::vector<char> ani;
        ReadWhole(StripExt(mdlPath) + ".ani", ani);
        std::string cmd = "$animblocksize " + std::to_string(kb);
        if (!frameanim)
            cmd += " lowres";
        else if (AnimBlockHighRes(m, a, h.numlocalanim, h.numbones, blocks, h.numanimblocks, ani))
            cmd += " highres";
        lines.push_back(cmd + "  // KB; estimated from where the blocks fell");

        // Stating any entry replaces the writer's automatic choice for EVERY
        // bone, so every flagged bone is listed.
        const fm::mstudiobone_t* bones =
            m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
        const std::vector<std::string> names = BoneNames(m);
        for (int i = 0; bones && i < h.numbones; ++i) {
            std::string o;
            if (bones[i].flags & fm::BONE_HAS_SAVEFRAME_POS)
                o += " position";
            if ((bones[i].flags & fm::BONE_HAS_SAVEFRAME_ROT32) ||
                (DmxModelVersion() == 1 && (bones[i].flags & fm::BONE_HAS_SAVEFRAME_ROT64)))
                o += " rotation";
            if (DmxModelVersion() != 1 && (bones[i].flags & fm::BONE_HAS_SAVEFRAME_ROT64))
                o += " rotation64";
            if (!o.empty())
                lines.push_back("$bonesaveframe \"" + names[i] + "\"" + o);
        }
    }

    if (lines.empty())
        return;
    q.Blank();
    for (const std::string& l : lines)
        q.Line(l);
}

// The `walkframe` control names, in the bit order LookupControl declares them.
const struct { int32_t bit; const char* name; } kMotionControls[] = {
    {fm::STUDIO_X, "X"},     {fm::STUDIO_Y, "Y"},     {fm::STUDIO_Z, "Z"},
    {fm::STUDIO_XR, "XR"},   {fm::STUDIO_YR, "YR"},   {fm::STUDIO_ZR, "ZR"},
    {fm::STUDIO_LX, "LX"},   {fm::STUDIO_LY, "LY"},   {fm::STUDIO_LZ, "LZ"},
    {fm::STUDIO_LXR, "LXR"}, {fm::STUDIO_LYR, "LYR"}, {fm::STUDIO_LZR, "LZR"},
    {fm::STUDIO_LINEAR, "LM"}, {fm::STUDIO_QUADRATIC_MOTION, "LQ"},
};

// ikrule is an animation option, so the rules ride on whichever animation owns
// them - writing blend anim 0's set on the sequence would drop the rules of
// every other grid animation and desync the compiler's per-rule realign check.
// A chain's own height/pad/floor never reach the .mdl - only the resolved
// per-rule copies do - so those are always written out rather than inherited.
// The ramp and contact are cycle fractions of this animation. One line each,
// for the { } body of whatever declared the animation.
std::vector<std::string> IkRules(const Mdl& m, const fm::mstudioanimdesc_t& a) {
    // animblockikruleindex is relative to the .ani block, not the .mdl - a
    // demand-loaded clip's rules are simply out of reach here.
    const fm::mstudioikrule_t* rules =
        m.At<fm::mstudioikrule_t>(&a, a.ikruleindex, a.numikrules);
    if (!rules)
        return {};
    const std::vector<std::string> chains = IkChainNames(m);
    const std::vector<std::string> boneNames = BoneNames(m);
    const float lastframe = static_cast<float>(a.numframes - 1);
    auto pick = [](const std::vector<std::string>& v, int i) {
        return (i >= 0 && static_cast<size_t>(i) < v.size()) ? v[i] : std::string();
    };
    std::vector<std::string> out;
    for (int k = 0; k < a.numikrules; ++k) {
        const fm::mstudioikrule_t& r = rules[k];
        std::string line = "ikrule \"" + pick(chains, r.chain) + "\"";
        switch (r.type) {
            case fm::IK_SELF: line += " touch \"" + pick(boneNames, r.bone) + "\""; break;
            case fm::IK_GROUND: line += " footstep"; break;
            case fm::IK_RELEASE: line += " release"; break;
            case fm::IK_ATTACHMENT:
                // a raw string after the error streams, not the string table
                line += " attachment \"" + std::string(m.Str(&r, r.szattachmentindex)) + "\"";
                break;
            default:
                // IK_WORLD / IK_UNLATCH: no script spelling to write back
                continue;
        }
        line += " height " + F(r.height) + " radius " + F(r.radius) + " floor " + F(r.floor);
        // -1 is the "never set" the parser starts from; it survives as a
        // negative cycle
        if (r.contact >= 0.0f)
            line += " contact " + std::to_string(std::lround(r.contact * lastframe));
        line += " range " + std::to_string(std::lround(r.start * lastframe)) + " " +
                std::to_string(std::lround(r.peak * lastframe)) + " " +
                std::to_string(std::lround(r.tail * lastframe)) + " " +
                std::to_string(std::lround(r.end * lastframe));
        if (r.type == fm::IK_SELF && a.numframes == 1) {
            const auto* error =
                m.At<fm::mstudiocompressedikerror_t>(&r, r.compressedikerrorindex);
            if (error) {
                float value[6] = {};
                for (int axis = 0; axis < 6; ++axis) {
                    const auto* stream = m.At<fm::mstudioanimvalue_t>(error, error->offset[axis]);
                    const auto* sample = stream && stream->num.valid
                                             ? m.At<fm::mstudioanimvalue_t>(stream, sizeof(*stream))
                                             : nullptr;
                    if (sample)
                        value[axis] = sample->value * error->scale[axis];
                }
                line += " fakeorigin " + V3({value[0], value[1], value[2]});
                line += " fakerotate " + QAngleDeg({value[3], value[4], value[5]});
            }
        }
        out.push_back(std::move(line));
    }
    return out;
}

// The options an animation carries, shared by $animation and the inline form.
// One per line so the result stays editable; the bare flags share the fps line
// because they read as one clause and never need arguments.
std::vector<std::string> AnimOptions(const Mdl& m, const fm::mstudioanimdesc_t& a,
                                     const std::string& subtract) {
    std::string first = "fps " + F(a.fps);
    if (a.flags & fm::STUDIO_LOOPING)
        first += " loop";
    if (a.flags & fm::STUDIO_NOFORCELOOP)
        first += " noforceloop";
    if (a.flags & fm::STUDIO_SNAP)
        first += " snap";
    if (a.flags & fm::STUDIO_POST)
        first += " post";
    std::vector<std::string> out{std::move(first)};
    if ((a.flags & fm::STUDIO_DELTA) && !subtract.empty())
        out.push_back("subtract \"" + subtract + "\" 0");
    // walkframe, one per stored movement key and in the same order - the
    // compiler chains each from the previous key's end frame. smdwrite.cpp puts
    // the travel back into the clip so there is something left to extract.
    const fm::mstudiomovement_t* mv =
        m.At<fm::mstudiomovement_t>(&a, a.movementindex, a.nummovements);
    for (int k = 0; mv && k < a.nummovements; ++k) {
        std::string ctrl;
        for (const auto& c : kMotionControls)
            if (mv[k].motionflags & c.bit)
                ctrl += " " + std::string(c.name);
        if (!ctrl.empty())
            out.push_back("walkframe " + std::to_string(mv[k].endframe) + ctrl);
    }
    // demand loading: which side of the .ani this clip's data actually landed on
    if (m.hdr->numanimblocks > 1) {
        const BlockUse u = AnimBlockUse(m, a);
        if (!u.external)
            out.push_back("noanimblock");
        else if (u.localSections > 0)
            out.push_back("noanimblockstall nostallframes " +
                          std::to_string(u.localSections * a.sectionframes));
    }
    for (std::string& r : IkRules(m, a))
        out.push_back(std::move(r));
    return out;
}

// $animation / $declareanimation, naming the clip animwrite.cpp put in anims/.
// An implied animation is skipped - it goes back inside its own $sequence.
void WriteAnimations(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* a =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    if (!a || h.numlocalanim <= 0)
        return;
    const std::vector<AnimRef> refs = AnimRefs(m);
    const int base = SubtractBase(m, refs);

    std::vector<std::string> lines;
    bool prevBlock = false; // a { } body gets a blank line either side
    // declared first so every delta below can name it
    if (NeedsBindPoseAnim(m, base))
        lines.push_back("$animation \"" + std::string(kBindPoseAnim) + "\" \"" +
                        AnimFile(kBindPoseAnim) + "\" fps 30  // the pose the deltas subtract");
    for (int i = 0; i < h.numlocalanim; ++i) {
        if (a[i].flags & fm::STUDIO_OVERRIDE) {
            lines.push_back("$declareanimation \"" + refs[i].name + "\"");
            continue;
        }
        if (refs[i].implied)
            continue;
        const std::string decl =
            "$animation \"" + refs[i].name + "\" \"" + AnimFile(refs[i].name) + "\" ";
        const std::string frames = "  // " + std::to_string(a[i].numframes) + " frames";
        // more than one option takes the { } body - one per line, editable
        const std::vector<std::string> opts = AnimOptions(m, a[i], SubtractNameFor(refs, base, i));
        const bool block = opts.size() > 1;
        if (!lines.empty() && (block || prevBlock))
            lines.push_back("");
        prevBlock = block;
        if (!block) {
            lines.push_back(decl + opts[0] + frames);
            continue;
        }
        lines.push_back(decl + "{" + frames);
        for (const std::string& o : opts)
            lines.push_back("    " + o);
        lines.push_back("}");
    }
    if (lines.empty())
        return;

    q.Blank();
    for (const std::string& l : lines)
        q.Line(l);
}

// $sequence / $declaresequence, in file order: the two interleave, and that
// order is the sequence index every activity lookup and $includemodel override
// resolves against.
void WriteSequences(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioseqdesc_t* seqs =
        m.At<fm::mstudioseqdesc_t>(m.buf.data(), h.localseqindex, h.numlocalseq);
    if (!seqs || h.numlocalseq <= 0)
        return;
    const fm::mstudioanimdesc_t* anims =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    const std::vector<AnimRef> animRefs = AnimRefs(m);
    const int subBase = SubtractBase(m, animRefs);
    const std::vector<std::string> boneNames = BoneNames(m);
    const std::vector<std::string> poses = PoseParamNames(m);
    const std::vector<std::string> nodes = NodeNames(m);
    const std::vector<std::string> chains = IkChainNames(m);
    const WeightLists weights = GatherWeightLists(m);
    std::vector<std::string> labels;
    // the engine allows duplicate sequence names, the compiler does not - suffix
    // repeats so the emitted script recompiles
    std::map<std::string, int> labelSeen;
    for (int i = 0; i < h.numlocalseq; ++i) {
        std::string name = m.Str(&seqs[i], seqs[i].szlabelindex);
        std::string key = name;
        for (char& c : key) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        int& n = labelSeen[key];
        if (n++ > 0) name += "_" + std::to_string(n);
        labels.push_back(name);
    }
    auto pick = [](const std::vector<std::string>& v, int i) {
        return (i >= 0 && static_cast<size_t>(i) < v.size()) ? v[i] : std::string();
    };

    bool prevDeclare = false;
    for (int i = 0; i < h.numlocalseq; ++i) {
        const fm::mstudioseqdesc_t& s = seqs[i];
        const bool declare = (s.flags & fm::STUDIO_OVERRIDE) != 0;
        if (!declare || !prevDeclare) // a run of declares stays one tight block
            q.Blank();
        prevDeclare = declare;
        if (declare) {
            q.Line("$declaresequence \"" + labels[i] + "\"");
            continue;
        }
        q.Line("$sequence \"" + labels[i] + "\" {");
        auto opt = [&](const std::string& line) { q.Line("    " + line); };

        // blend grid. The grid is stored in the order the script listed it, so
        // it reads straight back; groupsize only says how to fold it.
        const int g0 = s.groupsize[0] > 0 ? s.groupsize[0] : 1;
        const int g1 = s.groupsize[1] > 0 ? s.groupsize[1] : 1;
        const int16_t* grid = m.At<int16_t>(&s, s.animindexindex, g0 * g1);
        int32_t animFlags = 0;
        for (int k = 0; grid && k < g0 * g1; ++k) {
            const bool known = grid[k] >= 0 && grid[k] < static_cast<int>(animRefs.size());
            if (!known) {
                opt("\"\"");
                continue;
            }
            // an implied animation was written inline as a file, not by name
            const AnimRef& r = animRefs[grid[k]];
            opt(r.implied ? "\"" + AnimFile(r.name) + "\"  // " +
                                std::to_string(anims[grid[k]].numframes) + " frames"
                          : "\"" + r.name + "\"");
            animFlags |= anims[grid[k]].flags;
        }
        if (g0 * g1 > 1)
            opt("blendwidth " + std::to_string(g0));

        // events and layer ramps are stored as cycle fractions of blend anim 0,
        // which is what turns them back into the frames a script writes
        const int anim0 = (grid && g0 * g1 > 0) ? grid[0] : -1;
        // animation options in a sequence body land on blend animation 0, so
        // they only belong here when that one was declared inline
        if (anims && anim0 >= 0 && anim0 < h.numlocalanim && animRefs[anim0].implied)
            for (const std::string& o :
                 AnimOptions(m, anims[anim0], SubtractNameFor(animRefs, subBase, anim0)))
                opt(o);
        const float lastframe = (anims && anim0 >= 0 && anim0 < h.numlocalanim)
                                    ? static_cast<float>(anims[anim0].numframes - 1)
                                    : 0.0f;

        const char* act = m.Str(&s, s.szactivitynameindex);
        if (*act)
            opt("activity \"" + std::string(act) + "\" " + std::to_string(s.actweight));

        // a sequence ORs in its animations' flags, so only a bit they do not
        // carry can have come from a sequence option
        const int32_t f = s.flags & ~animFlags;
        if (f & fm::STUDIO_SNAP)
            opt("snap");
        if (f & fm::STUDIO_AUTOPLAY)
            opt("autoplay");
        if (f & fm::STUDIO_HIDDEN)
            opt("hidden");
        if (f & fm::STUDIO_REALTIME)
            opt("realtime");
        if (f & fm::STUDIO_WORLD_AND_RELATIVE)
            opt("worldrelative");
        else if (f & fm::STUDIO_WORLD)
            opt("worldspace");
        // `delta` is DELTA|POST and `predelta` is DELTA alone, so the delta bit
        // is read off the sequence even when its animation is what carries it -
        // spelling it `post` would recompile to the same flags but hide why
        if (s.flags & fm::STUDIO_DELTA) {
            if (f & fm::STUDIO_POST)
                opt("delta");
            else if (f & fm::STUDIO_DELTA)
                opt("predelta");
        } else if ((f & fm::STUDIO_POST) &&
                   !(f & (fm::STUDIO_WORLD | fm::STUDIO_WORLD_AND_RELATIVE))) {
            opt("post");
        }
        if (f & fm::STUDIO_ROOTXFORM)
            opt("rootdriver \"" + pick(boneNames, s.rootDriverIndex) + "\"");
        if (s.flags & fm::STUDIO_CYCLEPOSE)
            opt("posecycle \"" + pick(poses, s.cycleposeindex) + "\"");

        if (s.fadeintime != 0.2f)
            opt("fadein " + F(s.fadeintime));
        if (s.fadeouttime != 0.2f)
            opt("fadeout " + F(s.fadeouttime));

        // calcblend's measured range lands in the same fields, so it comes back
        // as a plain blend - the same grid without needing the attachment
        for (int p = 0; p < 2; ++p)
            if (s.paramindex[p] >= 0)
                opt("blend \"" + pick(poses, s.paramindex[p]) + "\" " + F(s.paramstart[p]) +
                    " " + F(s.paramend[p]));

        const fm::mstudioautolayer_t* al =
            m.At<fm::mstudioautolayer_t>(&s, s.autolayerindex, s.numautolayers);
        for (int k = 0; al && k < s.numautolayers; ++k) {
            std::string tail;
            if (al[k].flags & fm::STUDIO_AL_LOCAL)
                tail += " local";
            if (al[k].flags & fm::STUDIO_AL_XFADE)
                tail += " xfade";
            if (al[k].flags & fm::STUDIO_AL_SPLINE)
                tail += " spline";
            if (al[k].flags & fm::STUDIO_AL_NOBLEND)
                tail += " noblend";
            if (al[k].flags & fm::STUDIO_AL_POSE)
                tail += " poseparameter \"" + pick(poses, al[k].iPose) + "\"";
            const std::string target = "\"" + pick(labels, al[k].iSequence) + "\"";
            const bool ramp = al[k].start || al[k].peak || al[k].tail || al[k].end;
            // addlayer is the no-ramp form and takes nothing but `local`
            if (!ramp && (al[k].flags & ~fm::STUDIO_AL_LOCAL) == 0) {
                opt("addlayer " + target + tail);
                continue;
            }
            // a pose-driven layer keeps raw pose units; everything else is a
            // cycle fraction of this sequence's first animation
            const float sc = (al[k].flags & fm::STUDIO_AL_POSE) ? 1.0f : lastframe;
            opt("blendlayer " + target + " " + F(al[k].start * sc) + " " + F(al[k].peak * sc) +
                " " + F(al[k].tail * sc) + " " + F(al[k].end * sc) + tail);
        }

        const fm::mstudioiklock_t* locks =
            m.At<fm::mstudioiklock_t>(&s, s.iklockindex, s.numiklocks);
        for (int k = 0; locks && k < s.numiklocks; ++k)
            opt("iklock \"" + pick(chains, locks[k].chain) + "\" " + F(locks[k].flPosWeight) +
                " " + F(locks[k].flLocalQWeight));

        const fm::mstudioevent_t* ev = m.At<fm::mstudioevent_t>(&s, s.eventindex, s.numevents);
        for (int k = 0; ev && k < s.numevents; ++k) {
            const std::string id =
                (ev[k].type & fm::NEW_EVENT_STYLE)
                    ? "\"" + std::string(m.Str(&ev[k], ev[k].szeventindex)) + "\""
                    : std::to_string(ev[k].event);
            std::string line = "event " + id + " " +
                               std::to_string(std::lround(ev[k].cycle * lastframe));
            const std::string o(ev[k].options, strnlen(ev[k].options, sizeof ev[k].options));
            if (!o.empty())
                line += " \"" + o + "\"";
            opt("{ " + line + " }");
        }

        const fm::mstudioanimtag_t* tag =
            m.At<fm::mstudioanimtag_t>(&s, s.animtagindex, s.numanimtags);
        for (int k = 0; tag && k < s.numanimtags; ++k)
            opt("animtag \"" + std::string(m.Str(&tag[k], tag[k].sztagindex)) + "\" " +
                F(tag[k].cycle));

        const fm::mstudioactivitymodifier_t* am = m.At<fm::mstudioactivitymodifier_t>(
            &s, s.activitymodifierindex, s.numactivitymodifiers);
        for (int k = 0; am && k < s.numactivitymodifiers; ++k)
            opt("activitymodifier \"" + std::string(m.Str(&am[k], am[k].sznameindex)) + "\"");

        // the stored text is the block's contents, without its outer braces
        if (s.keyvaluesize > 0) {
            const char* kv = m.At<char>(&s, s.keyvalueindex, s.keyvaluesize);
            if (kv)
                opt("keyvalues { " + std::string(kv, strnlen(kv, s.keyvaluesize)) + " }");
        }

        if (s.localentrynode || s.localexitnode) {
            const std::string from = pick(nodes, s.localentrynode - 1);
            const std::string to = pick(nodes, s.localexitnode - 1);
            if (s.localentrynode == s.localexitnode)
                opt("node \"" + from + "\"");
            else
                opt(std::string(s.nodeflags & 1 ? "rtransition" : "transition") + " \"" + from +
                    "\" \"" + to + "\"");
        }

        // a weightlist rides on blend animation 0, which is where the sequence's
        // own array came from
        const float* w = m.At<float>(&s, s.weightlistindex, h.numbones);
        if (w && !weights.DefaultApplies()) {
            const std::string wl = weights.NameFor(std::vector<float>(w, w + h.numbones));
            if (!wl.empty())
                opt("weightlist \"" + wl + "\"");
        }

        q.Line("}");
    }
}

void WriteHitboxes(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiohitboxset_t* sets =
        m.At<fm::mstudiohitboxset_t>(m.buf.data(), h.hitboxsetindex, h.numhitboxsets);
    if (!sets)
        return;

    if (h.flags & fm::STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX) {
        // nothing was authored - let the compiler generate them again. The one
        // input that can't be re-derived that way is $skipboneinbbox: without
        // it every auto box is seeded at the bone's local origin (0,0,0), so a
        // box that ends up excluding it proves the flag was set.
        for (int s = 0; s < h.numhitboxsets; ++s) {
            const fm::mstudiobbox_t* boxes =
                m.At<fm::mstudiobbox_t>(&sets[s], sets[s].hitboxindex, sets[s].numhitboxes);
            for (int i = 0; boxes && i < sets[s].numhitboxes; ++i) {
                const fm::mstudiobbox_t& b = boxes[i];
                if (b.bbmin.x > 0.0f || b.bbmax.x < 0.0f || b.bbmin.y > 0.0f ||
                    b.bbmax.y < 0.0f || b.bbmin.z > 0.0f || b.bbmax.z < 0.0f) {
                    q.Blank();
                    q.Line("$skipboneinbbox");
                    return;
                }
            }
        }
        return;
    }

    const std::vector<std::string> names = BoneNames(m);

    for (int s = 0; s < h.numhitboxsets; ++s) {
        const fm::mstudiobbox_t* boxes =
            m.At<fm::mstudiobbox_t>(&sets[s], sets[s].hitboxindex, sets[s].numhitboxes);
        q.Blank();
        // stock's $hbox lines are top level, not a block body
        q.Line("$hboxset \"" + std::string(m.Str(&sets[s], sets[s].sznameindex)) + "\"" +
               (g_studiomdl ? "" : " {"));
        for (int i = 0; boxes && i < sets[s].numhitboxes; ++i) {
            const fm::mstudiobbox_t& b = boxes[i];
            const std::string bone =
                (b.bone >= 0 && static_cast<size_t>(b.bone) < names.size()) ? names[b.bone]
                                                                            : std::string();
            std::string line = (g_studiomdl ? "$hbox " : "    $hbox ") + std::to_string(b.group) +
                               " \"" + bone + "\" " + V3(b.bbmin) + "  " + V3(b.bbmax);
            // a box is only a capsule when the radius is positive, and the
            // orientation is read only for a capsule - on a box it is dead data
            if (b.flCapsuleRadius > 0.0f) {
                if (b.angOffsetOrientation.x || b.angOffsetOrientation.y ||
                    b.angOffsetOrientation.z)
                    line += " angles " + V3(b.angOffsetOrientation);
                line += " radius " + F(b.flCapsuleRadius);
            }
            // whitespace or junk is not a name worth restating
            std::string hbname = m.Str(&b, b.szhitboxnameindex);
            const size_t first = hbname.find_first_not_of(" \t");
            hbname = first == std::string::npos
                         ? std::string()
                         : hbname.substr(first, hbname.find_last_not_of(" \t") - first + 1);
            if (CleanName(hbname))
                line += " name \"" + hbname + "\"";
            q.Line(line);
        }
        if (!g_studiomdl)
            q.Line("}");
    }
}

// Every .mdl under `dir`, recursively, in a stable order. Case-insensitive so a
// .MDL out of an old pack is not skipped.
void CollectMdl(const std::string& dir, std::vector<std::string>& out) {
    std::error_code ec;
    std::vector<std::string> found;
    for (const auto& e : std::filesystem::recursive_directory_iterator(
             dir, std::filesystem::directory_options::skip_permission_denied, ec)) {
        std::string ext = e.path().extension().string();
        for (char& c : ext)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (ext == ".mdl")
            found.push_back(e.path().string());
    }
    std::sort(found.begin(), found.end());
    out.insert(out.end(), found.begin(), found.end());
}

// Turn the in-flight exception into a footer - shared by the per-file guard and
// main's outer one.
int FailCaught() {
    try {
        throw;
    } catch (const std::bad_alloc&) {
        return Fail("out of memory", "an allocation failed - a bogus count in the file can ask for"
                                     " more than available RAM");
    } catch (const std::exception& e) {
        return Fail("internal error", e.what());
    } catch (...) {
        return Fail("internal error", "unknown C++ exception");
    }
}

// Everything a decompile produces goes in its own folder named after the model,
// next to the .mdl or under -outdir (relative paths are off the cwd). -o is an
// explicit override for the script path and is used verbatim.
std::string OutFolder(const std::string& in, const char* outDir) {
    return outDir ? (std::filesystem::path(outDir) / BaseName(StripExt(in))).string()
                  : StripExt(in);
}

std::string OutScript(const std::string& dir, const char* out) {
    return out ? out
               : (std::filesystem::path(dir) / (BaseName(dir) + (g_studiomdl ? ".qc" : ".pulseqc")))
                     .string();
}

int DecompileOne(const std::string& in, const char* out, const char* outDir, int forceVersion) {
    std::printf("Decompiling: %s\n", in.c_str());

    // GoldSrc shares only the "IDST" magic with v44+; it has its own reader.
    if (IsGoldSrcMdl(in)) {
        pulse::fatal::g_stage = "goldsrc";
        const std::string dir = OutFolder(in, outDir);
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        return DecompileGoldSrc(in, dir, OutScript(dir, out));
    }

    using Clock = std::chrono::steady_clock;
    auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto t0 = Clock::now();

    pulse::fatal::g_stage = "read";
    Mdl m;
    std::string err;
    if (!ReadFile(in.c_str(), m, forceVersion, err))
        return Fail("read error", err);
    const auto tRead = Clock::now();

    const fm::studiohdr_t& h = *m.hdr;
    std::printf("model:       \"%s\"\n", h.name);
    std::printf("contents:    %d bones, %d bodyparts, %d materials, %d animations, %d sequences,\n"
                "             %d flex controllers, %d attachments, %d hitbox sets\n",
                h.numbones, h.numbodyparts, h.numtextures, h.numlocalanim, h.numlocalseq,
                h.numflexcontrollers, h.numlocalattachments, h.numhitboxsets);
    STAGE(PrintMaterials, m);

    pulse::fatal::g_stage = "output folder";
    const std::string dir = OutFolder(in, outDir);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string outPath = OutScript(dir, out);
    std::FILE* f = std::fopen(outPath.c_str(), "wb");
    if (!f)
        return Fail("write error",
                    "cannot write \"" + outPath + "\"" +
                        (ec ? " (" + ec.message() + ")" : std::string()));

    Qc q{f};
    q.Line("// mdldecompiler version " + std::string(kAppVersion));
    q.Line("// " + in);
    q.Blank();
    STAGE(WriteHeader, q, m);
    STAGE(WriteMaterials, q, m);

    // stock has no top-level flex/eye commands - they are $model options, so
    // capture them here and let WriteBodyParts put them in the block.
    pulse::fatal::g_stage = "MeshNames";
    const std::vector<std::vector<std::string>> meshNames = MeshNames(m);
    pulse::fatal::g_stage = "BuildFlexRig";
    const FlexRig rig = BuildFlexRig(m);
    const std::string faceMesh = FaceMesh(m, meshNames);
    std::string faceBody;
    if (g_studiomdl) {
        q.sink = &faceBody;
        STAGE(WriteEyes, q, m, faceMesh);
        STAGE(WriteFlexes, q, m, rig, faceMesh);
        q.sink = nullptr;
        faceBody.erase(0, faceBody.find_first_not_of('\n'));
    }
    pulse::fatal::g_stage = "WriteBodyParts";
    WriteBodyParts(q, m, meshNames, faceBody);

    // $lod / $shadowlod, right after the bodygroups. The meshes are written
    // first so the blocks only name files that exist. LOD 0 is the root the
    // loader makes on its own and has no block; a negative switch is the shadow LOD.
    std::printf("\nmeshes:\n");
    pulse::fatal::g_stage = "WriteRenderMeshes";
    const auto tMesh0 = Clock::now();
    const std::vector<LodInfo> lods = WriteRenderMeshes(m, in, dir, meshNames, rig);
    const auto tMesh1 = Clock::now();
    // The LOD meshes come out of the .vvd already rigged, so stock must not
    // re-derive their weights from LOD 0 the way an authored LOD needs.
    if (g_studiomdl && lods.size() > 1) {
        q.Blank();
        q.Line("// $skinnedlods  // Alien Swarm and newer");
    }
    for (size_t l = 1; l < lods.size(); ++l) {
        q.Blank();
        // $shadowlod eats whatever is left on its line as a switch value, so its
        // '{' has to go on the next one. $lod wants the opposite - inline.
        if (lods[l].switchPoint < 0.0f) {
            q.Line("$shadowlod");
            q.Line("{");
        } else {
            q.Line("$lod " + F(lods[l].switchPoint) + " {");
        }
        for (const std::vector<std::string>& part : meshNames)
            for (const std::string& name : part)
                if (!name.empty())
                    // stock matches the source by filename, not by model name
                    q.Line("    replacemodel \"" + (g_studiomdl ? MeshFile(name) : name) + "\" \"" +
                           MeshFile(name + "_lod" + std::to_string(l)) + "\"");
        for (const auto& r : lods[l].materialReplacements)
            q.Line("    replacematerial \"" + r.first + "\" \"" + r.second + "\"");
        q.Line("}");
    }

    if (!g_studiomdl) {
        STAGE(WriteEyes, q, m, std::string());
        STAGE(WriteFlexes, q, m, rig, faceMesh);
    }
    STAGE(WriteSkins, q, m);
    STAGE(WriteAttachments, q, m);
    STAGE(WriteHitboxes, q, m);
    STAGE(WriteBones, q, m);
    STAGE(WriteBoneMerges, q, m);
    // stock has neither $driverbone nor $driveraimat - only the .vrd file form
    if (g_studiomdl) {
        STAGE(WriteProceduralBones, q, m, dir);
    } else {
        STAGE(WriteDriverBones, q, m);
        STAGE(WriteAimAtBones, q, m);
    }
    STAGE(WriteJiggleBones, q, m);
    STAGE(WriteAnimBlocks, q, m, in);
    STAGE(WritePoseParams, q, m);
    STAGE(WriteIk, q, m);
    STAGE(WriteWeightLists, q, m);
    STAGE(WriteAnimations, q, m);
    STAGE(WriteSequences, q, m);
    STAGE(WriteIncludeModels, q, m);
    const auto tPhys0 = Clock::now();
    STAGE(WritePhysics, q, m, in, dir);
    const auto tPhys1 = Clock::now();
    STAGE(WriteKeyValues, q, m);
    std::fclose(f);

    if (h.numlocalanim > 0)
        std::printf("\nanimations:\n");
    const auto tAnim0 = Clock::now();
    STAGE(WriteAnimationFiles, m, in, dir);
    const auto tAnim1 = Clock::now();

    pulse::fatal::g_stage = "done";
    std::printf("\nwrote %s\n", outPath.c_str());

    if (pulse::perf::g_enabled) {
        pulse::perf::Record("total", "read", ms(t0, tRead));
        pulse::perf::Record("total", "mesh dmx write", ms(tMesh0, tMesh1));
        pulse::perf::Record("total", "physics write", ms(tPhys0, tPhys1));
        pulse::perf::Record("total", "animation write", ms(tAnim0, tAnim1));
        pulse::perf::Record("total", "whole decompile", ms(t0, Clock::now()));
    }
    return 0;
}

} // namespace

int RunDecompile(int argc, char** argv) {
    pulse::fatal::g_stage = "command line";
    std::vector<std::string> inputs;
    const char* out = nullptr;
    const char* outDir = nullptr;
    int forceVersion = 0;
    std::string dmxEncoding = "binary";
    int dmxModel = 15;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc)
            out = argv[++i];
        else if (std::strcmp(argv[i], "-outdir") == 0 && i + 1 < argc)
            outDir = argv[++i];
        else if (std::strcmp(argv[i], "-forceversion") == 0 && i + 1 < argc)
            forceVersion = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "-dmxencoding") == 0 && i + 1 < argc)
            dmxEncoding = argv[++i];
        else if (std::strcmp(argv[i], "-dmxmodel") == 0 && i + 1 < argc)
            dmxModel = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "-smdanimation") == 0)
            SetAnimFormat(true);
        else if (std::strcmp(argv[i], "-pulseqc") == 0)
            g_studiomdl = false;
        else if (std::strcmp(argv[i], "-studiomdl") == 0)
            g_studiomdl = true; // now the default; still accepted so old runs work
        else if (std::strcmp(argv[i], "-pause") == 0)
            pulse::fatal::g_pause = true;
        else if (std::strcmp(argv[i], "-perfmetrics") == 0)
            pulse::perf::g_enabled = true;
        else
            inputs.push_back(argv[i]);
    }
    if (inputs.empty())
        return Usage();
    if (const char* err = SetDmxOutput(dmxEncoding, dmxModel))
        return Fail("command line", err);

    // a folder input expands to the models under it; everything is collected
    // before the first decompile, so the output folders it makes are not rescanned
    std::vector<std::string> files;
    for (const std::string& in : inputs) {
        std::error_code ec;
        if (std::filesystem::is_directory(in, ec))
            CollectMdl(in, files);
        else
            files.push_back(in);
    }
    if (files.empty())
        return Fail("command line", "no .mdl files found");
    if (files.size() > 1) {
        // batch runs are usually drag-and-drop; hold the window so the summary
        // stays readable even if -pause was not passed
        pulse::fatal::g_pause = true;
        if (out) {
            std::printf("-o names one script - ignored, %zu models are being decompiled\n",
                        files.size());
            out = nullptr;
        }
    }

    // one bad model must not end a batch, so each is guarded on its own
    int failed = 0;
    for (size_t i = 0; i < files.size(); ++i) {
        if (files.size() > 1)
            std::printf("\n===== [%zu/%zu] =====\n", i + 1, files.size());
        int rc;
        try {
            rc = DecompileOne(files[i], out, outDir, forceVersion);
        } catch (...) {
            rc = FailCaught();
        }
        failed += rc != 0;
    }
    if (files.size() > 1)
        std::printf("\n%zu of %zu decompiled, %d failed\n", files.size() - failed, files.size(),
                    failed);
    pulse::perf::Report();
    return failed ? 1 : 0;
}

int main(int argc, char** argv) {
    // progress lines are useless if they sit in the CRT buffer until exit -
    // MSVC has no line buffering (_IOLBF == _IOFBF), so go unbuffered
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    pulse::fatal::Install();
    PrintHeader();

    // a malformed .mdl walks the writers off the end of an array as often as it
    // trips a check, so the footer names the writer that died
    int rc;
    try {
        rc = RunDecompile(argc, argv);
    } catch (...) {
        rc = FailCaught();
    }
    pulse::fatal::PauseIfAsked();
    return rc;
}
