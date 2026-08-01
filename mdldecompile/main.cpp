// mdldecompile - reads a compiled .mdl and writes back a .pulseqc describing it.
//
// Usage:
//   mdldecompile <file.mdl> [-o <file.pulseqc>] [-forceversion <n>]
//
// The script-level markup - names, materials, bodygroups, skeleton, attachments,
// hitboxes, skins - plus one .dmx render mesh per model (dmxwrite.cpp). Animation
// clips are still only named, not extracted.

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

#include "dmxwrite.h"
#include "fatalerror.h"
#include "format/phy.h"
#include "mdlfile.h"
#include "smdwrite.h"

using namespace mdldecompile;
using pulse::fatal::Fail;

// Name the writer in the crash footer without repeating its name as a string.
#define STAGE(fn, ...) (pulse::fatal::g_stage = #fn, fn(__VA_ARGS__))

// Defined by CMake from PROJECT_VERSION, same value the .exe version resource gets.
static constexpr const char* kAppVersion = PULSEMDL2_VERSION;

namespace {

void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("MDLDecompiler\n");
    std::printf("version:   %s (model version 49)\n", kAppVersion);
    std::printf("developer: Toppi (MIT License)\n");
    std::printf("-------------------------------\n");
}

int Usage() {
    std::printf("usage: mdldecompile <file.mdl> [-o <file.pulseqc>] [-forceversion <n>]\n");
    std::printf("\n");
    std::printf("  -o <file>     script to write; defaults to a folder named after the\n");
    std::printf("                .mdl, next to it, holding the script and its meshes\n");
    std::printf("  -forceversion <n>\n");
    std::printf("                read the file as version <n>, ignoring the header field\n");
    std::printf("                (some compilers write a bogus one to block decompiling)\n");
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
    // Some compilers hex-edit a bogus version into a file whose layout is
    // really an older one, to throw decompilers off. -forceversion reads it as
    // the version given and ignores the field.
    const int32_t version = forceVersion ? forceVersion : m.hdr->version;
    if (version != 49) {
        err = "unsupported .mdl version " + std::to_string(version) +
              " (only 49; -forceversion 49 reads it anyway)";
        return false;
    }
    if (version != m.hdr->version)
        std::printf("file says version %d, reading it as %d\n", m.hdr->version, version);
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

// $definebone, $attachment and $driverbone all read their angles as QAngle
// degrees - pitch, yaw, roll - while a RadianEuler holds the same rotation as
// (roll, pitch, yaw). Writing one straight out cycles the three components and
// the recompile rebinds every vertex to a twisted skeleton.
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
    void Line(const std::string& s) { std::fprintf(f, "%s\n", s.c_str()); }
    void Blank() { std::fprintf(f, "\n"); }
};

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

void WriteHeader(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    q.Line("$modelname \"" + std::string(h.name) + "\"");

    q.Line(std::string("$modelarchetype ") + Archetype(m));

    const char* prop = m.Str(m.buf.data(), h.surfacepropindex);
    if (*prop)
        q.Line("$surfaceprop \"" + std::string(prop) + "\"");

    q.Line("$contents " + ContentsTokens(h.contents));

    if (h.flags & fm::STUDIOHDR_FLAGS_FORCE_OPAQUE)
        q.Line("$renderpass opaque");
    else if (h.flags & fm::STUDIOHDR_FLAGS_TRANSLUCENT_TWOPASS)
        q.Line("$renderpass mostlyopaque");

    if (h.flags & fm::STUDIOHDR_FLAGS_AMBIENT_BOOST)
        q.Line("$ambientboost");
    if (h.flags & fm::STUDIOHDR_FLAGS_DO_NOT_CAST_SHADOWS)
        q.Line("$donotcastshadows");
    if (h.flags & fm::STUDIOHDR_FLAGS_FORCE_PHONEME_CROSSFADE)
        q.Line("$forcephonemecrossfade");

    q.Blank();
    q.Line("$bbox " + V3(h.hull_min) + "  " + V3(h.hull_max));
    if (h.view_bbmin.x || h.view_bbmin.y || h.view_bbmin.z || h.view_bbmax.x || h.view_bbmax.y ||
        h.view_bbmax.z)
        q.Line("$cbox " + V3(h.view_bbmin) + "  " + V3(h.view_bbmax));
    if (h.eyeposition.x || h.eyeposition.y || h.eyeposition.z)
        q.Line("$eyeposition " + V3(Unswizzle(h.eyeposition)));

    const fm::studiohdr2_t* h2 = m.At<fm::studiohdr2_t>(m.buf.data(), h.studiohdr2index);
    // a bone-relative $illumposition became an attachment - only the static
    // form is recoverable from the header numbers
    if (!h2 || h2->illumpositionattachmentindex == 0)
        q.Line("$illumposition " + V3(Unswizzle(h.illumposition)));
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

// $rendermesh + $modelgroup. Returns the alias picked for each model ("" for a
// blank body) - dmxwrite writes <alias>.dmx next to the script, which is what
// the emitted $rendermesh names.
std::vector<std::vector<std::string>> WriteBodyParts(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::vector<std::string>> meshNames;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    if (!parts)
        return meshNames;

    // pass 1: one $rendermesh per non-blank model, names made unique
    std::set<std::string> used;
    meshNames.resize(h.numbodyparts);
    q.Blank();
    for (int i = 0; i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            if (models[j].numvertices == 0) {
                meshNames[i].push_back(""); // blank body
                continue;
            }
            const std::string file(models[j].name, strnlen(models[j].name, sizeof models[j].name));
            std::string name = StripExt(BaseName(file));
            if (!CleanName(name))
                name = "mesh";
            std::string unique = name;
            for (int n = 2; !used.insert(unique).second; ++n)
                unique = name + "_" + std::to_string(n);
            meshNames[i].push_back(unique);
            q.Line("$rendermesh \"" + unique + "\" \"meshes/" + unique + ".dmx\"");
        }
    }

    // pass 2: the bodygroups that pick between them
    for (int i = 0; i < h.numbodyparts; ++i) {
        q.Blank();
        q.Line("$modelgroup \"" + std::string(m.Str(&parts[i], parts[i].sznameindex)) + "\" {");
        for (const std::string& name : meshNames[i])
            // the reference is quoted too - a model name with a space in it is
            // one token only when it is
            q.Line(name.empty() ? "    blank" : "    mesh name \"" + name + "\" \"" + name + "\"");
        q.Line("}");
    }
    return meshNames;
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
            q.Line("    trigger " + Deg(1.0f / tr[t].inv_tolerance) + "  " + QAngleDeg(driverRot) +
                   "  " + QAngleDeg(helperRot) + "  " + V3(tr[t].pos));
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

// --- physics ----------------------------------------------------------------

// One `section { "key" "value" ... }` out of the .phy's plain-text tail. Every
// physics setting a script authored is in there except the hull geometry, which
// is the binary half of the file.
struct PhySection {
    std::string name;
    std::vector<std::pair<std::string, std::string>> kv;

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

    std::vector<std::string> toks;
    while (p < buf.size() && buf[p]) {
        const char c = buf[p];
        if (std::isspace(static_cast<unsigned char>(c))) {
            ++p;
        } else if (c == '"') {
            size_t e = p + 1;
            while (e < buf.size() && buf[e] && buf[e] != '"')
                ++e;
            toks.emplace_back(&buf[p + 1], e - p - 1);
            p = e + 1;
        } else if (c == '{' || c == '}') {
            toks.emplace_back(1, c);
            ++p;
        } else {
            size_t e = p;
            while (e < buf.size() && buf[e] && !std::isspace(static_cast<unsigned char>(buf[e])) &&
                   buf[e] != '{' && buf[e] != '}')
                ++e;
            toks.emplace_back(&buf[p], e - p);
            p = e;
        }
    }

    for (size_t i = 0; i + 1 < toks.size();) {
        if (toks[i + 1] != "{") {
            ++i;
            continue;
        }
        PhySection s;
        s.name = toks[i];
        i += 2;
        for (; i + 1 < toks.size() && toks[i] != "}"; i += 2)
            s.kv.emplace_back(toks[i], toks[i + 1]);
        if (i < toks.size())
            ++i; // past '}'
        out.push_back(std::move(s));
    }
    return out;
}

// $physicsmodel. The hulls go out as a DMX collision mesh beside the render
// meshes; everything else round-trips out of the .phy's text tail.
void WritePhysics(Qc& q, const Mdl& m, const std::string& mdlPath, const std::string& dir) {
    const std::vector<PhySection> secs = ReadPhy(mdlPath);
    std::vector<const PhySection*> solids;
    const PhySection* edit = nullptr;
    for (const PhySection& s : secs) {
        if (s.name == "solid")
            solids.push_back(&s);
        else if (s.name == "editparams")
            edit = &s;
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

    q.Blank();
    q.Line("$physicsmodel {");
    if (!phys.written)
        q.Line("    // the .phy's hulls could not be read - this file has to be supplied");
    q.Line("    $physicsshape fromfile {");
    q.Line("        file \"meshes/" + meshName + ".dmx\"");
    q.Line("        importtype perjoint");
    if (phys.concave || (edit && edit->Get("concave") == "1"))
        q.Line("        concave");
    q.Line("    }");

    const float totalmass = edit ? edit->Getf("totalmass", 1.0f) : 1.0f;
    q.Line(totalmass < 0.0f ? "    $automass" : "    $mass " + F(totalmass));
    if (edit && !edit->Get("rootname").empty())
        q.Line("    $rootbone \"" + edit->Get("rootname") + "\"");

    // The model-wide values are whatever body 0 got; a body that differs gets a
    // $physicsmarkup below. A differing rotdamping may be the compiler's own
    // long-body adjustment rather than something authored - restating it as
    // markup reproduces it either way, since markup is applied last.
    q.Line("    $damping " + F(solids[0]->Getf("damping")));
    q.Line("    $rotdamping " + F(solids[0]->Getf("rotdamping")));
    q.Line("    $inertia " + F(solids[0]->Getf("inertia")));
    if (solids[0]->Find("drag"))
        q.Line("    $drag " + F(solids[0]->Getf("drag")));

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
            lines.push_back(std::string("        ") + k + " " + F(s->Getf(k)));
        }
        if (lines.empty())
            continue;
        q.Line("    $physicsmarkup \"" + s->Get("name") + "\" {");
        for (const std::string& l : lines)
            q.Line(l);
        q.Line("    }");
    }

    // "a,b" = the bone b was merged into a
    if (edit)
        for (const auto& kv : edit->kv) {
            if (kv.first != "jointmerge")
                continue;
            const size_t comma = kv.second.find(',');
            if (comma == std::string::npos)
                continue;
            q.Line("    $physicsmarkup \"" + kv.second.substr(comma + 1) + "\" { mergeinto \"" +
                   kv.second.substr(0, comma) + "\" }");
        }

    static const char* kAxis[3][4] = {{"x", "xmin", "xmax", "xfriction"},
                                      {"y", "ymin", "ymax", "yfriction"},
                                      {"z", "zmin", "zmax", "zfriction"}};
    for (const PhySection& s : secs) {
        if (s.name != "ragdollconstraint")
            continue;
        q.Line("    $physicsjoint \"" + solidName(std::atoi(s.Get("child").c_str())) + "\" {");
        for (const auto& a : kAxis) {
            const float lo = s.Getf(a[1]), hi = s.Getf(a[2]), fr = s.Getf(a[3]);
            // an axis the script never named was zero-filled, which is exactly
            // what `fixed` writes - so it comes back as fixed
            std::string line = std::string("        ") + a[0];
            if (lo == -360.0f && hi == 360.0f)
                line += " free";
            else if (lo == 0.0f && hi == 0.0f && fr == 0.0f)
                line += " fixed";
            else
                line += " limit " + F(lo) + " " + F(hi);
            if (fr != 0.0f)
                line += " friction " + F(fr);
            q.Line(line);
        }
        q.Line("    }");
    }

    for (const PhySection& s : secs) {
        if (s.name != "collisionrules")
            continue;
        if (s.Get("selfcollisions") == "0")
            q.Line("    $noselfcollisions");
        for (const auto& kv : s.kv) {
            if (kv.first != "collisionpair")
                continue;
            const size_t comma = kv.second.find(',');
            if (comma == std::string::npos)
                continue;
            q.Line("    $physicscollide \"" + solidName(std::atoi(kv.second.c_str())) + "\" \"" +
                   solidName(std::atoi(kv.second.c_str() + comma + 1)) + "\"");
        }
    }

    for (const PhySection& s : secs)
        if (s.name == "animatedfriction")
            q.Line("    $animatedfriction " +
                   std::to_string(static_cast<int>(s.Getf("animfrictionmin"))) + " " +
                   std::to_string(static_cast<int>(s.Getf("animfrictionmax"))) + " " +
                   F(s.Getf("animfrictiontimein")) + " " + F(s.Getf("animfrictiontimeout")) + " " +
                   F(s.Getf("animfrictiontimehold")));

    q.Line("}");
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

    q.Blank();
    for (int i = 0; i < h.numlocalattachments; ++i) {
        // `rigid` / `absolute` are consumed at compile time and are not in the
        // file; the baked matrix reproduces the same result without them.
        pm::RadianEuler rot;
        pm::Vector3 pos;
        pm::MatrixAngles(atts[i].local, rot, pos);
        const std::string bone =
            (atts[i].localbone >= 0 && static_cast<size_t>(atts[i].localbone) < names.size())
                ? names[atts[i].localbone]
                : std::string();
        std::string line = "$attachment \"" + std::string(m.Str(&atts[i], atts[i].sznameindex)) +
                           "\" \"" + bone + "\" origin " + V3(pos) + " angles " + QAngleDeg(rot);
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

void WriteEyes(Qc& q, const Mdl& m) {
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
        std::string line = "$eyeball \"" + r.name + "\" bone \"" + pick(boneNames, r.e->bone) +
                           "\" origin " + V3(r.origin) + " diameter " + F(r.e->radius * 2.0f) +
                           " angle " + F(std::atan(r.e->zoffset) * pm::kRad2Deg);
        if (r.e->iris_scale != 0.0f)
            line += " pupilscale " + F(1.0f / r.e->iris_scale);
        q.Line(line + " material \"" + BaseName(r.material) + "\"");
    }

    // $eyelid is one command per lid covering both eyes, so pair them by which
    // side's flexdesc each eyeball's lid base points at.
    auto sideOf = [&](const EyeballRef& r, bool upper) {
        const std::string n = pick(descs, upper ? r.e->upperlidflexdesc : r.e->lowerlidflexdesc);
        if (n.size() >= 6 && n.compare(n.size() - 6, 6, "_right") == 0)
            return 1;
        if (n.size() >= 5 && n.compare(n.size() - 5, 5, "_left") == 0)
            return 0;
        return -1; // unauthored - CheckEyeballSetup filled in "dummy_eyelid"
    };

    static const char* kSlot[3] = {"lowerer", "neutral", "raiser"};
    bool noted = false;
    for (int upper = 1; upper >= 0; --upper) {
        const EyeballRef* side[2] = {nullptr, nullptr}; // [0] left, [1] right
        for (const EyeballRef& r : eyes) {
            const int s = sideOf(r, upper != 0);
            if (s >= 0 && !side[s])
                side[s] = &r;
        }
        if (!side[0] || !side[1])
            continue;
        if (!noted) {
            q.Blank();
            noted = true;
        }
        const int32_t* flex = upper ? side[1]->e->upperflexdesc : side[1]->e->lowerflexdesc;
        const float* target = upper ? side[1]->e->uppertarget : side[1]->e->lowertarget;
        std::string line = std::string("$eyelid ") + (upper ? "upper" : "lower");
        for (int i = 0; i < 3; ++i)
            line += std::string(" ") + kSlot[i] + " \"" + pick(descs, flex[i]) + "\" " + F(target[i]);
        q.Line(line + " righteyeball \"" + side[1]->name + "\" lefteyeball \"" + side[0]->name +
               "\"");
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

void WriteFlexes(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioflexcontroller_t* fc =
        m.At<fm::mstudioflexcontroller_t>(m.buf.data(), h.flexcontrollerindex, h.numflexcontrollers);

    std::vector<std::string> ctrls;
    if (fc && h.numflexcontrollers > 0) {
        q.Blank();
        for (int i = 0; i < h.numflexcontrollers; ++i) {
            ctrls.push_back(m.Str(&fc[i], fc[i].sznameindex));
            std::string line = "$flexcontroller " + std::string(m.Str(&fc[i], fc[i].sztypeindex));
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
            q.Line("$mouth \"" + ctrl + "\" \"" + bone + "\" " + V3(mouths[i].forward));
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
            q.Line("$flexlocalvar \"" + v + "\"");
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
        q.Line((ok ? "" : "// ") + ("$flexrule \"" + target + "\" = " + expr));
    }
    if (commented)
        q.Line("// " + std::to_string(commented) + " of " + std::to_string(h.numflexrules) +
               " rules name a morph whose real name was lost - commented out, they would not"
               "\n// compile. The expression is intact if you can work out what it drove.");
    if (skipped) {
        q.Line("// " + std::to_string(skipped) + " of " + std::to_string(h.numflexrules) +
               " flex rules use combo/dominate/nway/2way ops, which have no $flexrule");
        q.Line("// spelling - they come back from the DMX rig via $datamodelflexes.");
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
        out.lists.emplace_back(
            "weights_" + (CleanName(label) ? label : std::to_string(out.lists.size())),
            std::move(v));
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
            if (w[b] != pred)
                q.Line("    \"" + names[b] + "\" " + F(w[b]));
        }
    };

    q.Blank();
    // one list and no sequence left on the plain default means every sequence
    // shared it - which is what $defaultweightlist does
    if (lists.size() == 1 && !w.sawPlain) {
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

// The options an animation carries, shared by $animation and the inline form.
std::string AnimOptions(const fm::mstudioanimdesc_t& a, const std::string& subtract) {
    std::string s = "fps " + F(a.fps);
    if ((a.flags & fm::STUDIO_DELTA) && !subtract.empty())
        s += " subtract \"" + subtract + "\" 0";
    if (a.flags & fm::STUDIO_LOOPING)
        s += " loop";
    if (a.flags & fm::STUDIO_NOFORCELOOP)
        s += " noforceloop";
    if (a.flags & fm::STUDIO_SNAP)
        s += " snap";
    if (a.flags & fm::STUDIO_POST)
        s += " post";
    return s;
}

// $animation / $declareanimation, naming the clip smdwrite.cpp put in anims/.
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
    // declared first so every delta below can name it
    if (NeedsBindPoseAnim(m, base))
        lines.push_back("$animation \"" + std::string(kBindPoseAnim) + "\" \"anims/" +
                        kBindPoseAnim + ".smd\" fps 30  // the pose the deltas subtract");
    for (int i = 0; i < h.numlocalanim; ++i) {
        if (a[i].flags & fm::STUDIO_OVERRIDE) {
            lines.push_back("$declareanimation \"" + refs[i].name + "\"");
            continue;
        }
        if (refs[i].implied)
            continue;
        lines.push_back("$animation \"" + refs[i].name + "\" \"anims/" + refs[i].name + ".smd\" " +
                        AnimOptions(a[i], SubtractNameFor(refs, base, i)) + "  // " +
                        std::to_string(a[i].numframes) + " frames");
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
    for (int i = 0; i < h.numlocalseq; ++i)
        labels.push_back(m.Str(&seqs[i], seqs[i].szlabelindex));
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
            opt(r.implied ? "\"anims/" + r.name + ".smd\"  // " +
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
            opt(AnimOptions(anims[anim0], SubtractNameFor(animRefs, subBase, anim0)));
        const float lastframe = (anims && anim0 >= 0 && anim0 < h.numlocalanim)
                                    ? static_cast<float>(anims[anim0].numframes - 1)
                                    : 0.0f;

        const char* act = m.Str(&s, s.szactivitynameindex);
        if (*act)
            opt("activity \"" + std::string(act) + "\"" +
                (s.actweight > 0 ? " " + std::to_string(s.actweight) : ""));

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
            opt(line);
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
        if (w) {
            const std::string wl = weights.NameFor(std::vector<float>(w, w + h.numbones));
            if (!wl.empty())
                opt("weightlist \"" + wl + "\"");
        }

        // ikrule. The rules are stored on the animation but spelled in the
        // sequence, so they come off blend animation 0. A chain's own
        // height/pad/floor never reach the .mdl - only the resolved per-rule
        // copies do - so those are always written out rather than left to be
        // inherited. The ramp and contact are cycle fractions of this animation.
        if (anims && anim0 >= 0 && anim0 < h.numlocalanim) {
            const fm::mstudioikrule_t* rules = m.At<fm::mstudioikrule_t>(
                &anims[anim0], anims[anim0].ikruleindex, anims[anim0].numikrules);
            for (int k = 0; rules && k < anims[anim0].numikrules; ++k) {
                const fm::mstudioikrule_t& r = rules[k];
                std::string line = "ikrule \"" + pick(chains, r.chain) + "\"";
                switch (r.type) {
                    case fm::IK_SELF:
                        line += " touch \"" + pick(boneNames, r.bone) + "\"";
                        break;
                    case fm::IK_GROUND: line += " footstep"; break;
                    case fm::IK_RELEASE: line += " release"; break;
                    case fm::IK_ATTACHMENT:
                        // a raw string after the error streams, not the string table
                        line += " attachment \"" + std::string(m.Str(&r, r.szattachmentindex)) +
                                "\"";
                        break;
                    default:
                        // IK_WORLD / IK_UNLATCH: no script spelling to write back
                        opt("// ikrule type " + std::to_string(r.type) + " on chain \"" +
                            pick(chains, r.chain) + "\" has no script spelling");
                        continue;
                }
                line += " height " + F(r.height) + " radius " + F(r.radius) + " floor " +
                        F(r.floor);
                // -1 is the "never set" the parser starts from; it survives as a
                // negative cycle
                if (r.contact >= 0.0f)
                    line += " contact " + std::to_string(std::lround(r.contact * lastframe));
                line += " range " + std::to_string(std::lround(r.start * lastframe)) + " " +
                        std::to_string(std::lround(r.peak * lastframe)) + " " +
                        std::to_string(std::lround(r.tail * lastframe)) + " " +
                        std::to_string(std::lround(r.end * lastframe));
                opt(line);
            }
        }

        q.Line("}");
    }
}

void WriteHitboxes(Qc& q, const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    if (h.flags & fm::STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX)
        return; // nothing was authored - let the compiler generate them again
    const fm::mstudiohitboxset_t* sets =
        m.At<fm::mstudiohitboxset_t>(m.buf.data(), h.hitboxsetindex, h.numhitboxsets);
    if (!sets)
        return;
    const std::vector<std::string> names = BoneNames(m);

    for (int s = 0; s < h.numhitboxsets; ++s) {
        const fm::mstudiobbox_t* boxes =
            m.At<fm::mstudiobbox_t>(&sets[s], sets[s].hitboxindex, sets[s].numhitboxes);
        q.Blank();
        q.Line("$hitboxset \"" + std::string(m.Str(&sets[s], sets[s].sznameindex)) + "\" {");
        for (int i = 0; boxes && i < sets[s].numhitboxes; ++i) {
            const fm::mstudiobbox_t& b = boxes[i];
            const std::string bone =
                (b.bone >= 0 && static_cast<size_t>(b.bone) < names.size()) ? names[b.bone]
                                                                            : std::string();
            std::string line = "    $hbox " + std::to_string(b.group) + " \"" + bone + "\" " +
                               V3(b.bbmin) + "  " + V3(b.bbmax);
            if (b.angOffsetOrientation.x || b.angOffsetOrientation.y || b.angOffsetOrientation.z)
                line += " angles " + V3(b.angOffsetOrientation);
            if (b.flCapsuleRadius > 0.0f)
                line += " radius " + F(b.flCapsuleRadius);
            const char* hbname = m.Str(&b, b.szhitboxnameindex);
            if (*hbname)
                line += " name \"" + std::string(hbname) + "\"";
            q.Line(line);
        }
        q.Line("}");
    }
}

} // namespace

int RunDecompile(int argc, char** argv) {
    pulse::fatal::g_stage = "command line";
    const char* in = nullptr;
    const char* out = nullptr;
    int forceVersion = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc)
            out = argv[++i];
        else if (std::strcmp(argv[i], "-forceversion") == 0 && i + 1 < argc)
            forceVersion = std::atoi(argv[++i]);
        else if (!in)
            in = argv[i];
    }
    if (!in)
        return Usage();

    std::printf("Decompiling: %s\n", in);

    pulse::fatal::g_stage = "read";
    Mdl m;
    std::string err;
    if (!ReadFile(in, m, forceVersion, err))
        return Fail("read error", err);

    const fm::studiohdr_t& h = *m.hdr;
    std::printf("model:       \"%s\"\n", h.name);
    std::printf("contents:    %d bones, %d bodyparts, %d materials, %d animations, %d sequences,\n"
                "             %d flex controllers, %d attachments, %d hitbox sets\n",
                h.numbones, h.numbodyparts, h.numtextures, h.numlocalanim, h.numlocalseq,
                h.numflexcontrollers, h.numlocalattachments, h.numhitboxsets);
    STAGE(PrintMaterials, m);

    // everything a decompile produces goes in its own folder next to the .mdl,
    // named after the model. -o is an explicit override and is used verbatim.
    pulse::fatal::g_stage = "output folder";
    const std::string dir = StripExt(in);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string outPath =
        out ? out : (std::filesystem::path(dir) / (BaseName(dir) + ".pulseqc")).string();
    std::FILE* f = std::fopen(outPath.c_str(), "wb");
    if (!f)
        return Fail("write error",
                    "cannot write \"" + outPath + "\"" +
                        (ec ? " (" + ec.message() + ")" : std::string()));

    Qc q{f};
    q.Line("// decompiled by mdldecompile from " + std::string(in));
    q.Blank();
    STAGE(WriteHeader, q, m);
    STAGE(WriteMaterials, q, m);
    pulse::fatal::g_stage = "WriteBodyParts";
    const std::vector<std::vector<std::string>> meshNames = WriteBodyParts(q, m);

    // $lod / $shadowlod, right after the bodygroups. The meshes are written
    // first so the blocks only name files that exist. LOD 0 is the root the
    // loader makes on its own and has no block; a negative switch is the shadow LOD.
    std::printf("\nmeshes:\n");
    pulse::fatal::g_stage = "WriteRenderMeshes";
    const std::vector<LodInfo> lods = WriteRenderMeshes(m, in, dir, meshNames);
    for (size_t l = 1; l < lods.size(); ++l) {
        q.Blank();
        q.Line(lods[l].switchPoint < 0.0f ? "$shadowlod {"
                                          : "$lod " + F(lods[l].switchPoint) + " {");
        for (const std::vector<std::string>& part : meshNames)
            for (const std::string& name : part)
                if (!name.empty())
                    q.Line("    replacemodel \"" + name + "\" \"meshes/" + name + "_lod" +
                           std::to_string(l) + ".dmx\"");
        for (const auto& r : lods[l].materialReplacements)
            q.Line("    replacematerial \"" + r.first + "\" \"" + r.second + "\"");
        q.Line("}");
    }

    STAGE(WriteEyes, q, m);
    STAGE(WriteFlexes, q, m);
    STAGE(WriteSkins, q, m);
    STAGE(WriteAttachments, q, m);
    STAGE(WriteHitboxes, q, m);
    STAGE(WriteBones, q, m);
    STAGE(WriteBoneMerges, q, m);
    STAGE(WriteDriverBones, q, m);
    STAGE(WriteAimAtBones, q, m);
    STAGE(WriteJiggleBones, q, m);
    STAGE(WritePoseParams, q, m);
    STAGE(WriteIk, q, m);
    STAGE(WriteWeightLists, q, m);
    STAGE(WriteAnimations, q, m);
    STAGE(WriteSequences, q, m);
    STAGE(WriteIncludeModels, q, m);
    STAGE(WritePhysics, q, m, in, dir);
    std::fclose(f);

    if (h.numlocalanim > 0)
        std::printf("\nanimations:\n");
    STAGE(WriteAnimationSmds, m, in, dir);

    pulse::fatal::g_stage = "done";
    std::printf("\nwrote %s\n", outPath.c_str());
    return 0;
}

int main(int argc, char** argv) {
    // progress lines are useless if they sit in the CRT buffer until exit -
    // MSVC has no line buffering (_IOLBF == _IOFBF), so go unbuffered
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    pulse::fatal::Install();
    PrintHeader();

    // a malformed .mdl walks the writers off the end of an array as often as it
    // trips a check, so the footer names the writer that died
    try {
        return RunDecompile(argc, argv);
    } catch (const std::bad_alloc&) {
        return Fail("out of memory", "an allocation failed - a bogus count in the file can ask for"
                                     " more than available RAM");
    } catch (const std::exception& e) {
        return Fail("internal error", e.what());
    } catch (...) {
        return Fail("internal error", "unknown C++ exception");
    }
}
