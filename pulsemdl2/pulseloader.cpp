// pulseloader.cpp - .pulsemdl compile-script loader. See pulseloader.h.

#include "pulseloader.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>

#include "dmx/dmx.h"
#include "dmxloader.h"
#include "facemarkup.h"
#include "flexreg.h"
#include "smdloader.h"

namespace pulse::loader {

namespace dmx = pulse::dmx;
namespace cm = pulse::compile;
namespace pm = pulse::math;
namespace fs = std::filesystem;

namespace {

// animation/sequence flag values (format/mdl.h, mirrored to avoid the header
// dependency)
constexpr int kStudioLooping = 0x0001;
constexpr int kStudioSnap = 0x0002;
constexpr int kStudioDelta = 0x0004;
constexpr int kStudioAutoplay = 0x0008;
constexpr int kStudioPost = 0x0010;
constexpr int kStudioRealtime = 0x0100;
constexpr int kStudioLocal = 0x0200;
constexpr int kStudioHidden = 0x0400;
constexpr int kStudioOverride = 0x0800;
// BSP contents flags, the subset $contents names (mirrored likewise)
constexpr int kContentsSolid = 0x1;
constexpr int kContentsGrate = 0x8;
constexpr int kContentsMonster = 0x2000000;
constexpr int kContentsDebris = 0x4000000;
constexpr int kContentsLadder = 0x20000000;
// autolayer flags
constexpr int kStudioAlSpline = 0x0040;
constexpr int kStudioAlXfade = 0x0080;
constexpr int kStudioAlNoblend = 0x0200;
constexpr int kStudioAlLocal = 0x1000;
constexpr int kStudioAlPose = 0x4000;

constexpr double kPiD = 3.14159265358979323846;

// scalar vector3 attribute (no typed accessor on dmx::Element for these)
bool GetVec3(const dmx::Element* e, const char* name, pm::Vector3& out) {
    const dmx::Attribute* a = e->Get(name);
    if (!a)
        return false;
    if (auto v = std::get_if<dmx::Vector3>(&a->value)) {
        out = {v->x, v->y, v->z};
        return true;
    }
    return false;
}

float DegToRad(float deg) { return deg * (pm::kPiF / 180.0f); }

std::string StripExtension(const std::string& s) {
    size_t dot = s.find_last_of('.');
    size_t slash = s.find_last_of("/\\");
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
        return s.substr(0, dot);
    return s;
}

// load-or-reuse a DMX source (reference Load_Source caches by filename; the
// first load's context wins, so a file first referenced as an animation and
// later used on a body keeps its morph-less load - same as the reference).
//
// `kind` is the role the reference fills (source::LoadKind), which decides how
// much of the file is read and what may be reused: an Animation reference gets
// bones and clips only and will reuse any entry, but the entry it leaves behind
// has no mesh, so a Model reference skips it and loads the file properly.
// `.pulsemdl` has no Collision kind - PhysicsShapeFromFile names a render mesh
// rather than a file, so there is no collision-only load here.
source::Source* LoadSource(cm::CompileInput& in, const fs::path& scriptDir,
                           const std::string& filename, std::string* err,
                           bool morphSource = false,
                           source::LoadKind kind = source::LoadKind::Model) {
    for (auto& sp : in.sources)
        if (_stricmp(sp->filename.c_str(), filename.c_str()) == 0 && sp->kind >= kind)
            return sp.get();

    fs::path full = scriptDir / filename;
    std::string loadErr;

    // one line per file actually read - a reuse of a loaded Source prints nothing
    std::printf("loading %s: %s\n",
                kind == source::LoadKind::Animation ? "animation" : "model",
                full.string().c_str());

    auto src = std::make_unique<source::Source>();
    src->filename = filename;
    src->kind = kind;
    const bool animOnly = kind == source::LoadKind::Animation;

    // legacy SMD (.smd/.sma/.phys) routes through the text reader; everything
    // else is DMX. SMD still follows the DMX rules - a mesh SMD is only usable
    // through a rendermesh reference, exactly like a DMX body.
    auto endsWith = [&](const char* ext) {
        const size_t n = std::strlen(ext);
        const std::string& s = filename;
        return s.size() >= n && _stricmp(s.c_str() + s.size() - n, ext) == 0;
    };
    const bool isSmd = endsWith(".smd") || endsWith(".sma") || endsWith(".phys");

    if (isSmd) {
        if (!source::LoadSmdSource(full.string(), *src, in.mats, in.scale, &loadErr, morphSource,
                                   animOnly)) {
            if (err) *err = "cannot load \"" + full.string() + "\": " + loadErr;
            return nullptr;
        }
    } else {
        auto dm = dmx::Datamodel::Load(full.string().c_str(), &loadErr);
        if (!dm) {
            if (err) *err = "cannot load \"" + full.string() + "\": " + loadErr;
            return nullptr;
        }
        if (!source::LoadDmxSource(*dm, *src, in.mats, in.scale, &loadErr, morphSource,
                                   /*filter=*/nullptr, animOnly)) {
            if (err) *err = "cannot parse \"" + full.string() + "\": " + loadErr;
            return nullptr;
        }
    }

    in.sources.push_back(std::move(src));
    return in.sources.back().get();
}

namespace lim = pulse::limits;

} // namespace

bool LoadPulseScript(const char* path, cm::CompileInput& out, std::string* err) {
    std::string loadErr;
    auto dm = dmx::Datamodel::Load(path, &loadErr);
    if (!dm) {
        if (err) *err = loadErr;
        return false;
    }
    const dmx::Element* root = dm->root;
    if (!root || root->className != "RootModel") {
        if (err) *err = "not a .pulsemdl script (root element must be RootModel)";
        return false;
    }

    fs::path scriptDir = fs::path(path).parent_path();
    out.mdlPath = path;

    // name -> outname (strip .mdl extension; the writer re-appends it)
    if (auto name = root->GetString("name"))
        out.outname = StripExtension(*name);
    if (out.outname.empty()) {
        if (err) *err = "RootModel has no name";
        return false;
    }

    // vtx_archetype: which .vtx StripGroup layout to write (replaces the old
    // -vtxformat switch). 0 = legacy (TF2/L4D2), 1 = full (SFM/CS:GO).
    out.vtxArchetype = root->GetInt("vtx_archetype", 0);
    if (out.vtxArchetype != 0 && out.vtxArchetype != 1) {
        if (err) *err = "unknown vtx_archetype (expected 0 or 1)";
        return false;
    }

    // model_archetype: "character"/"general" = normal, "static" = $staticprop,
    // "simple" = $simpleprop (schema decision 2026-07-16)
    if (auto arch = root->GetString("model_archetype")) {
        if (_stricmp(arch->c_str(), "character") == 0 ||
            _stricmp(arch->c_str(), "general") == 0) {
            out.archetype = cm::Archetype::General;
        } else if (_stricmp(arch->c_str(), "static") == 0) {
            out.archetype = cm::Archetype::Static;
        } else if (_stricmp(arch->c_str(), "simple") == 0) {
            out.archetype = cm::Archetype::Simple;
        } else {
            if (err) *err = "unknown model_archetype \"" + *arch +
                            "\" (expected character/general/static/simple)";
            return false;
        }
    }

    // modelmodifierlist. Skeleton-wide settings live on bonemarkuplist instead.
    bool unlockDefineBones = false;
    if (const dmx::Element* mods = root->GetElement("modelmodifierlist")) {
        out.scale = mods->GetFloat("scale", 1.0f);
        // render_pass: 0 = neither flag, 1 = $opaque, 2 = $mostlyopaque
        out.renderPass = mods->GetInt("render_pass", 0);
        if (out.renderPass < 0 || out.renderPass > 2) {
            if (err) *err = "modelmodifierlist render_pass must be 0 (none), "
                            "1 (opaque) or 2 (mostly opaque)";
            return false;
        }
        out.ambientBoost = mods->GetBool("ambient_boost", false);
        out.doNotCastShadows = mods->GetBool("do_not_cast_shadows", false);
        out.forcePhonemeCrossfade = mods->GetBool("force_phoneme_crossfade", false);

        // morph_cull_type: 0 aggressive, 1 duplicates, 2 none,
        // 3 rules_only (default)
        const int morphCull = mods->GetInt("morph_cull_type", 3);
        if (morphCull < 0 || morphCull > 3) {
            if (err) *err = "modelmodifierlist morph_cull_type must be 0 (aggressive), "
                            "1 (duplicates), 2 (none) or 3 (rules_only)";
            return false;
        }
        out.flexCullMethod = static_cast<cm::FlexCullMethod>(morphCull);

        // animation_cull_type: 0 aggressive (default), 1 duplicates, 2 none
        const int animCull = mods->GetInt("animation_cull_type", 0);
        if (animCull < 0 || animCull > 2) {
            if (err) *err = "modelmodifierlist animation_cull_type must be 0 (aggressive), "
                            "1 (duplicates) or 2 (none)";
            return false;
        }
        out.animCullMethod = static_cast<cm::AnimCullMethod>(animCull);

        // these moved to bonemarkuplist - reject rather than silently ignore,
        // since ignoring them changes the skeleton without saying so
        for (const char* moved : {"unlock_definebones", "realign_bones", "bone_cull_type"}) {
            if (mods->Get(moved)) {
                if (err) *err = std::string("modelmodifierlist \"") + moved +
                                "\" moved to bonemarkuplist";
                return false;
            }
        }

        // translatemodel/rotatemodel = $origin. rotatemodel is degrees;
        // z composes with the built-in +90 yaw exactly like $origin's 4th arg.
        pm::Vector3 t;
        if (GetVec3(mods, "translatemodel", t))
            out.adjust = t;
        pm::Vector3 r;
        if (GetVec3(mods, "rotatemodel", r)) {
            out.rotation = {r.x * pm::kDeg2Rad, r.y * pm::kDeg2Rad,
                            (r.z + 90.0f) * pm::kDeg2Rad};
            out.rotationSet = true;
        }
    }
    if (out.scale == 0.0f)
        out.scale = 1.0f;

    // defaultweightlist ($defaultweightlist): authored entries for weightlist
    // slot 0 - the list every animation without an explicit weightlist uses
    if (auto dws = root->GetElementArray("defaultweightlist")) {
        for (const dmx::Element* w : *dws) {
            if (!w) continue;
            cm::WeightList::Entry entry;
            if (auto b = w->GetString("bone"))
                entry.bone = *b;
            entry.weight = w->GetFloat("weight", 0.0f);
            entry.posweight = w->GetFloat("posweight", entry.weight);
            if (entry.weight == 0.0f && entry.posweight > 0.0f) {
                if (err) *err = "defaultweightlist bone \"" + entry.bone +
                                "\": posweight > 0 needs weight > 0";
                return false;
            }
            out.defaultWeights.push_back(std::move(entry));
        }
    }

    // weightlistlist ($weightlist): named per-bone weight masks
    if (auto wls = root->GetElementArray("weightlistlist")) {
        for (const dmx::Element* e : *wls) {
            if (!e) continue;
            cm::WeightList wl;
            wl.name = e->name;
            if (auto n = e->GetString("name"))
                wl.name = *n;
            if (wl.name.empty()) {
                if (err) *err = "WeightList has no name";
                return false;
            }
            for (const auto& existing : out.weightlists) {
                if (_stricmp(existing.name.c_str(), wl.name.c_str()) == 0) {
                    if (err) *err = "duplicate weightlist \"" + wl.name + "\"";
                    return false;
                }
            }
            if (auto ws = e->GetElementArray("weights")) {
                for (const dmx::Element* w : *ws) {
                    if (!w) continue;
                    cm::WeightList::Entry entry;
                    if (auto b = w->GetString("bone"))
                        entry.bone = *b;
                    entry.weight = w->GetFloat("weight", 0.0f);
                    entry.posweight = w->GetFloat("posweight", entry.weight);
                    if (entry.weight == 0.0f && entry.posweight > 0.0f) {
                        // reference Option_Weightlist: posweight needs a
                        // nonzero rotation weight
                        if (err) *err = "weightlist \"" + wl.name + "\" bone \"" +
                                        entry.bone + "\": posweight > 0 needs weight > 0";
                        return false;
                    }
                    wl.entries.push_back(std::move(entry));
                }
            }
            out.weightlists.push_back(std::move(wl));
        }
        if (out.weightlists.size() > static_cast<size_t>(pulse::limits::kMaxWeightlists) - 1) {
            if (err) *err = "too many weightlists";
            return false;
        }
    }

    // poseparameterlist ($poseparameter)
    if (auto pps = root->GetElementArray("poseparameterlist")) {
        for (const dmx::Element* e : *pps) {
            if (!e) continue;
            cm::PoseParam pp;
            pp.name = e->name;
            if (auto n = e->GetString("name"))
                pp.name = *n;
            pp.min = e->GetFloat("min", 0.0f);
            pp.max = e->GetFloat("max", 0.0f);
            if (e->GetBool("wrap", false)) {
                pp.flags |= 0x0001; // STUDIO_LOOPING
                pp.loop = pp.max - pp.min;
            }
            float loop = e->GetFloat("loop", 0.0f);
            if (loop != 0.0f) {
                pp.flags |= 0x0001;
                pp.loop = loop;
            }
            out.poseparams.push_back(std::move(pp));
        }
        if (out.poseparams.size() > static_cast<size_t>(pulse::limits::kMaxPoseParam)) {
            if (err) *err = "too many pose parameters";
            return false;
        }
    }

    // ikdatalist ($ikchain / $ikautoplaylock)
    if (auto iks = root->GetElementArray("ikdatalist")) {
        for (const dmx::Element* e : *iks) {
            if (!e) continue;
            if (e->className == "IKChain") {
                cm::IkChain chain;
                chain.name = e->name;
                if (auto n = e->GetString("name"))
                    chain.name = *n;
                bool dup = false;
                for (const auto& c : out.ikchains)
                    if (_stricmp(c.name.c_str(), chain.name.c_str()) == 0)
                        dup = true;
                if (dup) {
                    // reference: duplicate ikchain is warned about and ignored
                    fprintf(stderr, "WARNING: duplicate ikchain \"%s\" ignored\n",
                            chain.name.c_str());
                    continue;
                }
                auto endBone = e->GetString("end_bone");
                if (!endBone || endBone->empty()) {
                    if (err) *err = "IKChain \"" + chain.name + "\" has no end_bone";
                    return false;
                }
                chain.bonename = *endBone;
                chain.height = e->GetFloat("height", 18.0f);
                chain.radius = e->GetFloat("pad", 0.0f) / 2.0f;
                chain.floor = e->GetFloat("floor", 0.0f);
                pm::Vector3 v;
                if (GetVec3(e, "knee", v))
                    chain.link[0].kneeDir = v;
                if (GetVec3(e, "center", v))
                    chain.center = v;
                out.ikchains.push_back(std::move(chain));
            } else if (e->className == "IKAutoplayLock") {
                cm::IkLock lock;
                if (auto c = e->GetString("chain"))
                    lock.name = *c;
                lock.flPosWeight = e->GetFloat("position_weight", 0.0f);
                lock.flLocalQWeight = e->GetFloat("rotation_weight", 0.0f);
                out.ikautoplaylocks.push_back(std::move(lock));
            } else {
                if (err) *err = "unknown ikdatalist element class \"" + e->className + "\"";
                return false;
            }
        }
        if (out.ikchains.size() > static_cast<size_t>(pulse::limits::kMaxIkChains) ||
            out.ikautoplaylocks.size() >
                static_cast<size_t>(pulse::limits::kMaxIkAutoplayLocks)) {
            if (err) *err = "too many ik chains / autoplay locks";
            return false;
        }
    }

    // bonemarkuplist (top level): a container element carrying the skeleton-wide
    // bone_cull_type / primary_root_bone, with the per-bone entries under
    // `children`. A bare element_array of entries is still accepted for scripts
    // that have no container properties to set.
    const dmx::Element* bmuList = root->GetElement("bonemarkuplist");
    const std::vector<dmx::ElementPtr>* bmus =
        bmuList ? bmuList->GetElementArray("children")
                : root->GetElementArray("bonemarkuplist");

    if (bmuList) {
        int cull = bmuList->GetInt("bone_cull_type", 0);
        if (cull < 0 || cull > 2) {
            if (err) *err = "bone_cull_type " + std::to_string(cull) +
                            " out of range (0 = aggressive, 1 = leaf only, 2 = none)";
            return false;
        }
        out.boneCullType = static_cast<cm::BoneCullType>(cull);

        unlockDefineBones = bmuList->GetBool("unlock_definebones", false);
        out.realignBones = bmuList->GetBool("realign_bones", false);

        // primary_root_bone (QC $root) - parsed and validated only; nothing
        // consumes it yet.
        if (auto rootBone = bmuList->GetString("primary_root_bone"))
            out.primaryRootBone = *rootBone;
    }

    if (bmus) {
        for (const dmx::Element* e : *bmus) {
            if (!e) continue;
            cm::BoneMarkup bm;
            bm.name = e->name;
            if (auto n = e->GetString("bone"))
                bm.name = *n;
            if (bm.name.empty()) {
                if (err) *err = "BoneMarkup has no bone name";
                return false;
            }
            bm.doNotCollapse = e->GetBool("do_not_collapse", false);
            bm.isBonemerge = e->GetBool("is_bonemerge", false);
            out.bonemarkups.push_back(std::move(bm));
        }
    }

    // jigglebonelist (top level, $jigglebone): bones the engine simulates.
    // `type` picks the sim; a constraint turns on when the script gives its
    // attributes, mirroring how the QC subsections set the flags. Angles are
    // authored in degrees and stored in radians.
    if (auto jbs = root->GetElementArray("jigglebonelist")) {
        for (const dmx::Element* e : *jbs) {
            if (!e) continue;
            cm::JiggleBone jb;
            jb.bonename = e->name;
            if (auto n = e->GetString("bone"))
                jb.bonename = *n;
            if (jb.bonename.empty()) {
                if (err) *err = "JiggleBone has no bone name";
                return false;
            }

            // last definition wins, in place (reference Cmd_JiggleBone)
            cm::JiggleBone* slot = nullptr;
            for (cm::JiggleBone& prev : out.jigglebones)
                if (_stricmp(prev.bonename.c_str(), jb.bonename.c_str()) == 0) {
                    slot = &prev;
                    break;
                }

            // flex_type: 0 none, 1 flexible (default), 2 rigid. Exclusive - the
            // SDK forbids is_flexible together with is_rigid.
            int type = e->GetInt("flex_type", 1);
            if (type == 1) {
                jb.flags |= cm::kJiggleIsFlexible | cm::kJiggleHasLengthConstraint;
                // allow_length_flex lets the bone stretch (clears the constraint)
                if (e->GetBool("allow_length_flex", false))
                    jb.flags &= ~cm::kJiggleHasLengthConstraint;
            } else if (type == 2) {
                jb.flags |= cm::kJiggleIsRigid | cm::kJiggleHasLengthConstraint;
            } else if (type != 0) {
                if (err) *err = "JiggleBone \"" + jb.bonename + "\": invalid flex_type " +
                                std::to_string(type) +
                                " (0 = none, 1 = flexible, 2 = rigid)";
                return false;
            }

            jb.length = e->GetFloat("length", jb.length);
            jb.tipMass = e->GetFloat("tip_mass", jb.tipMass);

            // reference ParseJiggleStiffness: both stiffness AND damping clamp
            // to [0,1000] (ParseJiggleDamping's tighter range is never used)
            auto spring = [&](const char* key, float fallback) {
                float v = e->GetFloat(key, fallback);
                return v < 0.0f ? 0.0f : (v > 1000.0f ? 1000.0f : v);
            };
            jb.yawStiffness = spring("yaw_stiffness", jb.yawStiffness);
            jb.yawDamping = spring("yaw_damping", jb.yawDamping);
            jb.pitchStiffness = spring("pitch_stiffness", jb.pitchStiffness);
            jb.pitchDamping = spring("pitch_damping", jb.pitchDamping);
            jb.alongStiffness = spring("along_stiffness", jb.alongStiffness);
            jb.alongDamping = spring("along_damping", jb.alongDamping);

            if (e->Get("angle_limit")) {
                jb.flags |= cm::kJiggleHasAngleConstraint;
                jb.angleLimit = DegToRad(e->GetFloat("angle_limit", 0.0f));
            }
            if (e->Get("yaw_min") || e->Get("yaw_max")) {
                jb.flags |= cm::kJiggleHasYawConstraint;
                jb.minYaw = DegToRad(e->GetFloat("yaw_min", 0.0f));
                jb.maxYaw = DegToRad(e->GetFloat("yaw_max", 0.0f));
            }
            jb.yawFriction = e->GetFloat("yaw_friction", jb.yawFriction);
            jb.yawBounce = e->GetFloat("yaw_bounce", jb.yawBounce);

            if (e->Get("pitch_min") || e->Get("pitch_max")) {
                jb.flags |= cm::kJiggleHasPitchConstraint;
                jb.minPitch = DegToRad(e->GetFloat("pitch_min", 0.0f));
                jb.maxPitch = DegToRad(e->GetFloat("pitch_max", 0.0f));
            }
            jb.pitchFriction = e->GetFloat("pitch_friction", jb.pitchFriction);
            jb.pitchBounce = e->GetFloat("pitch_bounce", jb.pitchBounce);

            // effect_type: 0 none (default), 1 base spring, 2 boing. Exclusive -
            // the SDK forbids has_base_spring together with is_boing. The
            // fields of the mode that is NOT selected are ignored outright.
            int effect = e->GetInt("effect_type", 0);
            if (effect == 1) {
                jb.flags |= cm::kJiggleHasBaseSpring;
                jb.baseMass = e->GetFloat("base_mass", jb.baseMass);
                jb.baseStiffness = spring("base_stiffness", jb.baseStiffness);
                jb.baseDamping = spring("base_damping", jb.baseDamping);
                jb.baseMinLeft = e->GetFloat("base_left_min", jb.baseMinLeft);
                jb.baseMaxLeft = e->GetFloat("base_left_max", jb.baseMaxLeft);
                jb.baseLeftFriction = e->GetFloat("base_left_friction", jb.baseLeftFriction);
                jb.baseMinUp = e->GetFloat("base_up_min", jb.baseMinUp);
                jb.baseMaxUp = e->GetFloat("base_up_max", jb.baseMaxUp);
                jb.baseUpFriction = e->GetFloat("base_up_friction", jb.baseUpFriction);
                jb.baseMinForward = e->GetFloat("base_forward_min", jb.baseMinForward);
                jb.baseMaxForward = e->GetFloat("base_forward_max", jb.baseMaxForward);
                jb.baseForwardFriction =
                    e->GetFloat("base_forward_friction", jb.baseForwardFriction);
            } else if (effect == 2) {
                jb.flags |= cm::kJiggleIsBoing;
                // these five default to 0 on the struct and are seeded only
                // here, matching where Cmd_JiggleBone seeds them
                jb.boingImpactSpeed = e->GetFloat("boing_impact_speed", 100.0f);
                // the reference's default is the literal 0.7071f, which is NOT
                // cos(45 deg) to the bit - only the authored path takes a cosine
                jb.boingImpactAngle =
                    e->Get("boing_impact_angle")
                        ? std::cos(DegToRad(e->GetFloat("boing_impact_angle", 45.0f)))
                        : 0.7071f;
                jb.boingDampingRate = e->GetFloat("boing_damping_rate", 0.25f);
                jb.boingFrequency = e->GetFloat("boing_frequency", 30.0f);
                jb.boingAmplitude = e->GetFloat("boing_amplitude", 0.35f);
            } else if (effect != 0) {
                if (err) *err = "JiggleBone \"" + jb.bonename + "\": invalid effect_type " +
                                std::to_string(effect) +
                                " (0 = none, 1 = base spring, 2 = boing)";
                return false;
            }

            if (slot)
                *slot = std::move(jb);
            else
                out.jigglebones.push_back(std::move(jb));
        }

        if (out.jigglebones.size() > static_cast<size_t>(pulse::limits::kMaxJiggleBones)) {
            if (err) *err = "too many jigglebones";
            return false;
        }
    }

    // animconstraintlist (top level, $driverbone / VRD quatinterp helpers): the
    // helper bone's pose is interpolated between triggers by how closely the
    // driver bone matches each trigger orientation.
    //
    // Rotations are authored in degrees, translations in model units, and BOTH
    // pose fields are deltas from the helper's bind pose - MapProceduralBones
    // folds the bind pose in once the skeleton is final. The reference reaches
    // the same place via `$driverbone unlockbones`; here it is unconditional,
    // which is why there is no `basepos`.
    if (auto pbs = root->GetElementArray("animconstraintlist")) {
        for (const dmx::Element* e : *pbs) {
            if (!e) continue;
            cm::ProceduralBone pb;
            pb.helpername = e->name;
            if (auto h = e->GetString("helper"))
                pb.helpername = *h;
            if (pb.helpername.empty()) {
                if (err) *err = "ProceduralBone has no helper bone name";
                return false;
            }
            if (auto d = e->GetString("driver"))
                pb.drivername = *d;
            if (pb.drivername.empty()) {
                if (err) *err = "ProceduralBone \"" + pb.helpername +
                                "\" has no driver bone name";
                return false;
            }
            // optional - empty means "resolve from the final skeleton"
            if (auto p = e->GetString("helper_parent"))
                pb.helperparentname = *p;
            if (auto p = e->GetString("driver_parent"))
                pb.driverparentname = *p;

            if (auto trigs = e->GetElementArray("triggerlist")) {
                for (const dmx::Element* t : *trigs) {
                    if (!t) continue;
                    cm::ProceduralBoneTrigger tr;

                    // The writer stores 1/tolerance, so zero would put an inf in
                    // the .mdl. The reference divides unguarded; refuse instead -
                    // no real trigger has zero angular width.
                    float tolDeg = t->GetFloat("tolerance", 0.0f);
                    if (tolDeg <= 0.0f) {
                        if (err) *err = "ProceduralBone \"" + pb.helpername +
                                        "\": trigger tolerance must be > 0";
                        return false;
                    }
                    tr.tolerance = DegToRad(tolDeg);

                    pm::Vector3 driverRotDeg, helperRotDeg;
                    GetVec3(t, "driver_rotation", driverRotDeg);
                    GetVec3(t, "helper_rotation", helperRotDeg);
                    pm::AngleQuaternion({DegToRad(driverRotDeg.x), DegToRad(driverRotDeg.y),
                                         DegToRad(driverRotDeg.z)},
                                        tr.trigger);
                    pm::AngleQuaternion({DegToRad(helperRotDeg.x), DegToRad(helperRotDeg.y),
                                         DegToRad(helperRotDeg.z)},
                                        tr.quat);

                    // scaled at parse time, matching where the reference applies
                    // g_currentscale to the VRD trigger positions
                    pm::Vector3 pos;
                    GetVec3(t, "helper_translation", pos);
                    tr.pos = {pos.x * out.scale, pos.y * out.scale, pos.z * out.scale};

                    pb.triggers.push_back(tr);
                }
            }

            // hard cap - the .mdl reader walks exactly numtriggers entries and
            // the reference's arrays are [32]. It truncates with a warning.
            if (pb.triggers.size() > static_cast<size_t>(pulse::limits::kMaxProceduralTriggers)) {
                std::fprintf(stderr,
                             "animconstraint \"%s\": %zu triggers exceed limit of %d; "
                             "truncating\n",
                             pb.helpername.c_str(), pb.triggers.size(),
                             pulse::limits::kMaxProceduralTriggers);
                pb.triggers.resize(pulse::limits::kMaxProceduralTriggers);
            }
            if (pb.triggers.empty()) {
                std::fprintf(stderr, "animconstraint \"%s\": no triggers, ignoring\n",
                             pb.helpername.c_str());
                continue;
            }

            out.proceduralbones.push_back(std::move(pb));
        }

        if (out.proceduralbones.size() >
            static_cast<size_t>(pulse::limits::kMaxProceduralBones)) {
            if (err) *err = "too many animconstraints";
            return false;
        }
    }

    // skeleton: container for bone-structure commands ($definebone; later
    // $hierarchy and friends).
    if (const dmx::Element* skel = root->GetElement("skeleton")) {
        // definebonelist ($definebone). translation/rotation build the
        // bind-pose rawLocal; the optional realign pair marks the bone
        // pre-aligned.
        if (auto dbs = skel->GetElementArray("definebonelist")) {
            for (const dmx::Element* e : *dbs) {
                if (!e) continue;
                cm::ImportBone ib;
                ib.name = e->name;
                if (auto n = e->GetString("name"))
                    ib.name = *n;
                if (auto p = e->GetString("parent"))
                    ib.parent = *p;
                ib.bUnlocked = e->GetBool("unlocked", unlockDefineBones);

                pm::Vector3 pos, rotDeg;
                GetVec3(e, "translation", pos);
                GetVec3(e, "rotation", rotDeg); // (pitch, yaw, roll) degrees
                pm::AngleMatrixDeg(rotDeg, ib.rawLocal);
                ib.rawLocal.m[0][3] = pos.x;
                ib.rawLocal.m[1][3] = pos.y;
                ib.rawLocal.m[2][3] = pos.z;

                pm::Vector3 rpos, rrotDeg;
                bool hasRPos = GetVec3(e, "realign_translation", rpos);
                bool hasRRot = GetVec3(e, "realign_rotation", rrotDeg);
                if (hasRPos || hasRRot) {
                    ib.bPreAligned = true;
                    pm::AngleMatrixDeg(rrotDeg, ib.srcRealign);
                    ib.srcRealign.m[0][3] = rpos.x;
                    ib.srcRealign.m[1][3] = rpos.y;
                    ib.srcRealign.m[2][3] = rpos.z;
                }
                out.importbones.push_back(std::move(ib));
            }
        }

        // bonemorphdriverlist ($boneflexdriver): a bone translation component
        // drives a morph controller. Reference Cmd_BoneFlexDriver merges by
        // (bone, controller) via FindOrCreate*, so repeated entries UPDATE the
        // existing control instead of appending - mirrored here.
        if (auto bfds = skel->GetElementArray("bonemorphdriverlist")) {
            for (const dmx::Element* bfd : *bfds) {
                if (!bfd) continue;
                std::string bonename = bfd->name;
                if (auto b = bfd->GetString("bone"))
                    bonename = *b;
                if (bonename.empty()) {
                    if (err) *err = "BoneMorphDriver has no bone name";
                    return false;
                }

                cm::BoneFlexDriver* driver = nullptr;
                for (cm::BoneFlexDriver& d : out.boneflexdrivers)
                    if (_stricmp(d.bonename.c_str(), bonename.c_str()) == 0) {
                        driver = &d;
                        break;
                    }
                if (!driver) {
                    out.boneflexdrivers.push_back(cm::BoneFlexDriver{});
                    driver = &out.boneflexdrivers.back();
                    driver->bonename = bonename;
                }

                auto ctrls = bfd->GetElementArray("controls");
                if (!ctrls) {
                    if (err) *err = "BoneMorphDriver \"" + bonename + "\" has no controls";
                    return false;
                }
                for (const dmx::Element* c : *ctrls) {
                    if (!c) continue;
                    std::string ctrlname = c->name;
                    if (auto n = c->GetString("controller"))
                        ctrlname = *n;
                    if (ctrlname.empty()) {
                        if (err) *err = "BoneMorphDriver \"" + bonename +
                                        "\" control has no controller name";
                        return false;
                    }

                    // <tx|ty|tz>; reference matches by PREFIX (StringHasPrefix)
                    std::string comp = "tx";
                    if (auto s = c->GetString("component"))
                        comp = *s;
                    int component = -1; // STUDIO_BONE_FLEX_INVALID
                    static const char* kComponents[] = {"tx", "ty", "tz"};
                    for (int i = 0; i < 3; ++i)
                        if (comp.rfind(kComponents[i], 0) == 0) {
                            component = i;
                            break;
                        }
                    if (component < 0) {
                        if (err) *err = "BoneMorphDriver \"" + bonename + "\" control \"" +
                                        ctrlname + "\": invalid component \"" + comp +
                                        "\" (expected tx/ty/tz)";
                        return false;
                    }

                    cm::BoneFlexDriverControl* ctrl = nullptr;
                    for (cm::BoneFlexDriverControl& e : driver->controls)
                        if (_stricmp(e.controllername.c_str(), ctrlname.c_str()) == 0) {
                            ctrl = &e;
                            break;
                        }
                    if (!ctrl) {
                        driver->controls.push_back(cm::BoneFlexDriverControl{});
                        ctrl = &driver->controls.back();
                        ctrl->controllername = ctrlname;
                    }
                    ctrl->component = component;
                    ctrl->min = c->GetFloat("min", 0.0f);
                    ctrl->max = c->GetFloat("max", 1.0f);
                }
            }
        }
    }

    // rendermeshlist: name -> loaded source
    std::map<std::string, source::Source*> renderMeshes;
    if (auto rms = root->GetElementArray("rendermeshlist")) {
        for (const dmx::Element* rm : *rms) {
            if (!rm) continue;
            auto fn = rm->GetString("filename");
            if (!fn || fn->empty()) continue;
            float importScale = rm->GetFloat("import_scale", 1.0f);
            (void)importScale; // phase-1: script-level $scale only
            source::Source* src = LoadSource(out, scriptDir, *fn, err, /*morphSource=*/true);
            if (!src)
                return false;
            renderMeshes[rm->name] = src;
            // remember the script string for the mstudiomodel_t name
            src->filename = *fn;
        }
    }

    // cdmaterialslist (optional; SetSkinValues appends the empty relative
    // entry either way)
    // NOTE: entries go BEFORE the auto empty entry, matching parse order.
    if (auto cds = root->GetStringArray("cdmaterialslist")) {
        // stored on the input; Compile() copies + appends ""
        // (phase 1: keep it simple - copy into a temp on CompileInput)
        // Using outname field of sources is wrong; add to CompileInput later
        // if needed. For testcube parity the list is empty.
        (void)cds;
    }

    // bodygrouplist
    if (auto bgs = root->GetElementArray("bodygrouplist")) {
        for (const dmx::Element* bg : *bgs) {
            if (!bg) continue;
            cm::CompileInput::InBodyPart part;
            part.name = bg->name;
            if (auto n = bg->GetString("name"))
                part.name = *n;
            if (auto choices = bg->GetElementArray("choices")) {
                for (const dmx::Element* choice : *choices) {
                    if (!choice) continue;
                    cm::CompileInput::InModel model;
                    const dmx::Attribute* mesh = choice->Get("mesh");
                    std::string meshRef;
                    if (mesh) {
                        if (auto s = std::get_if<std::string>(&mesh->value))
                            meshRef = *s;
                    }
                    if (meshRef.empty()) {
                        model.name = "blank";
                        model.source = nullptr;
                    } else {
                        auto it = renderMeshes.find(meshRef);
                        if (it == renderMeshes.end()) {
                            if (err) *err = "bodygroup references unknown rendermesh \"" +
                                            meshRef + "\"";
                            return false;
                        }
                        model.source = it->second;
                        model.name = cm::ChoiceName({meshRef});
                    }
                    part.models.push_back(model);
                }
            }
            out.bodyparts.push_back(std::move(part));
        }
    }
    if (out.bodyparts.empty()) {
        if (err) *err = "no bodygroups";
        return false;
    }
    // bodypart base values (product of previous choice counts, starting at 1)
    // (stored via compile stage below)

    // ---- physics (Phase 4) ----
    // Four sibling top-level lists. QC's $collisionmodel / $collisionjoints
    // split is gone: BuildCollisionModel auto-detects a single body vs a
    // ragdoll from how many bones the collision geometry resolves to.
    // Placed after rendermeshlist so "mesh" "rendermesh" refs resolve, and
    // after bodygrouplist so a shape can be validated against loaded sources.

    // physicsshapelist - the collision geometry. Two element classes:
    // PhysicsShapeFromFile takes an authored collision mesh, PhysicsShapeFromRender
    // generates hulls off the render geometry (QC's $generatemodel/$generatejoint).
    if (auto shapes = root->GetElementArray("physicsshapelist")) {
        for (const dmx::Element* e : *shapes) {
            if (!e) continue;
            cm::PhysicsShape shape;
            shape.name = e->name;
            if (auto n = e->GetString("name"))
                shape.name = *n;

            const bool fromRender =
                (_stricmp(e->className.c_str(), "PhysicsShapeFromRender") == 0);
            if (!fromRender && _stricmp(e->className.c_str(), "PhysicsShapeFromFile") != 0) {
                if (err) *err = "physicsshapelist entry \"" + shape.name + "\" has unknown type \"" +
                                e->className + "\" - expected PhysicsShapeFromFile or "
                                "PhysicsShapeFromRender";
                return false;
            }
            shape.kind = fromRender ? cm::PhysicsShapeKind::FromRender
                                    : cm::PhysicsShapeKind::FromFile;

            if (auto pb = e->GetString("parent_bone"))
                shape.parentBone = *pb;

            // shared framing
            GetVec3(e, "offset_origin", shape.offsetOrigin);
            shape.offsetAnglesSet = GetVec3(e, "offset_angles", shape.offsetAngles);
            shape.importScale = e->GetFloat("import_scale", 1.0f);
            if (shape.importScale <= 0.0f) {
                if (err) *err = "physics shape \"" + shape.name +
                                "\" has import_scale " + std::to_string(shape.importScale) +
                                " - must be greater than 0";
                return false;
            }

            if (fromRender) {
                if (shape.parentBone.empty()) {
                    if (err) *err = "PhysicsShapeFromRender \"" + shape.name +
                                    "\" has no parent_bone";
                    return false;
                }
                shape.decimationFactor = e->GetFloat("decimation_factor", 0.22f);
                if (shape.decimationFactor > 1.0f) shape.decimationFactor = 1.0f;
                if (shape.decimationFactor > 0.0f && shape.decimationFactor < 0.1f)
                    shape.decimationFactor = 0.1f;
                shape.concavity = e->GetFloat("concavity", 0.04f);
                // 0 = "however many the split produced", which still means the
                // safety ceiling - VHACD merges down to whatever it is given.
                shape.maxHulls = e->GetInt("max_hulls", 0);
                if (shape.maxHulls <= 0) {
                    shape.maxHulls = lim::kMaxGeneratedHulls;
                } else if (shape.maxHulls > lim::kMaxGeneratedHulls) {
                    std::printf("WARNING: PhysicsShapeFromRender \"%s\" asks for %d hulls, "
                                "clamped to %d\n",
                                shape.name.c_str(), shape.maxHulls, lim::kMaxGeneratedHulls);
                    shape.maxHulls = lim::kMaxGeneratedHulls;
                }
                shape.cullWeight = e->GetFloat("cull_weight", 0.42f);
                if (shape.cullWeight < 0.0f) shape.cullWeight = 0.0f;
                if (shape.cullWeight > 1.0f) shape.cullWeight = 1.0f;
                shape.maxConvex = e->GetInt("max_convex", lim::kMaxConvexPieces);

                if (auto ex = e->GetStringArray("extra_skinned_bones"))
                    shape.extraSkinnedBones = *ex;
                if (shape.extraSkinnedBones.size() >
                    static_cast<size_t>(lim::kMaxExtraSkinnedBones)) {
                    if (err) *err = "PhysicsShapeFromRender \"" + shape.name +
                                    "\" lists too many extra_skinned_bones (max " +
                                    std::to_string(lim::kMaxExtraSkinnedBones) + ")";
                    return false;
                }

                // The filter excludes rendermeshes rather than selecting them,
                // so every name has to resolve or the exclusion silently misses.
                if (auto ef = e->GetStringArray("exception_render_mesh_filter"))
                    shape.exceptionMeshNames = *ef;
                for (const std::string& meshName : shape.exceptionMeshNames) {
                    auto it = renderMeshes.find(meshName);
                    if (it == renderMeshes.end()) {
                        if (err) *err = "PhysicsShapeFromRender \"" + shape.name +
                                        "\" filters unknown rendermesh \"" + meshName + "\"";
                        return false;
                    }
                    shape.exceptionSources.push_back(it->second);
                }
            } else {
                const dmx::Attribute* mesh = e->Get("mesh");
                std::string meshRef;
                if (mesh) {
                    if (auto s = std::get_if<std::string>(&mesh->value))
                        meshRef = *s;
                }
                if (meshRef.empty()) {
                    if (err) *err = "PhysicsShapeFromFile \"" + shape.name + "\" has no mesh";
                    return false;
                }
                auto it = renderMeshes.find(meshRef);
                if (it == renderMeshes.end()) {
                    if (err) *err = "PhysicsShapeFromFile \"" + shape.name +
                                    "\" references unknown rendermesh \"" + meshRef + "\"";
                    return false;
                }
                shape.source = it->second;

                const int importType = e->GetInt("import_type", 0);
                if (importType < 0 || importType > 2) {
                    if (err) *err = "PhysicsShapeFromFile \"" + shape.name +
                                    "\" has import_type " + std::to_string(importType) +
                                    " - must be 0, 1 or 2";
                    return false;
                }
                shape.importType = static_cast<cm::PhysicsImportType>(importType);
                if (shape.importType != cm::PhysicsImportType::Skinned &&
                    shape.parentBone.empty()) {
                    if (err) *err = "PhysicsShapeFromFile \"" + shape.name +
                                    "\" has import_type " + std::to_string(importType) +
                                    " but no parent_bone";
                    return false;
                }

                shape.concave = e->GetBool("concave", false);
                shape.maxConvex = e->GetInt("max_convex", lim::kMaxConvexPieces);
                shape.remove2d = e->GetBool("remove_2d", false);
            }
            out.physShapes.push_back(std::move(shape));
        }
        if (out.physShapes.size() > static_cast<size_t>(lim::kMaxPhysShapes)) {
            if (err) *err = "too many physics shapes (max " +
                            std::to_string(lim::kMaxPhysShapes) + ")";
            return false;
        }
    }

    // physicsjointlist - ragdoll constraints, keyed by BONE (not mesh)
    if (auto joints = root->GetElementArray("physicsjointlist")) {
        for (const dmx::Element* e : *joints) {
            if (!e) continue;
            cm::PhysicsJoint joint;
            if (auto b = e->GetString("bone"))
                joint.bonename = *b;
            if (joint.bonename.empty()) {
                if (err) *err = "PhysicsJoint \"" + e->name + "\" has no bone";
                return false;
            }
            if (auto axes = e->GetElementArray("axes")) {
                for (const dmx::Element* ax : *axes) {
                    if (!ax) continue;
                    cm::PhysicsJointAxis a;
                    std::string axisName;
                    if (auto s = ax->GetString("axis")) axisName = *s;
                    if      (axisName == "x") a.axis = 0;
                    else if (axisName == "y") a.axis = 1;
                    else if (axisName == "z") a.axis = 2;
                    else {
                        if (err) *err = "PhysicsJoint \"" + joint.bonename +
                                        "\" has unknown axis \"" + axisName +
                                        "\" (expected x/y/z)";
                        return false;
                    }
                    std::string typeName = "free";
                    if (auto s = ax->GetString("type")) typeName = *s;
                    if      (typeName == "free")  a.type = 0;
                    else if (typeName == "limit") a.type = 1;
                    else if (typeName == "fixed") a.type = 2;
                    else {
                        if (err) *err = "PhysicsJoint \"" + joint.bonename +
                                        "\" axis " + axisName + " has unknown type \"" +
                                        typeName + "\" (expected free/limit/fixed)";
                        return false;
                    }
                    a.min = ax->GetFloat("min", 0.0f);
                    a.max = ax->GetFloat("max", 0.0f);
                    a.friction = ax->GetFloat("friction", 0.0f);
                    joint.axes.push_back(a);
                }
            }
            out.physJoints.push_back(std::move(joint));
        }
    }

    // physicsmarkuplist - a container: the model-level settings are its own
    // attributes, and `children` holds the per-body overrides keyed by BONE.
    // Container and child share spellings on purpose (`damping` on the
    // container is the default a child's `damping` overrides). A body is
    // always identified by its bone, so this works whether one collision mesh
    // spans many bones or each body gets its own mesh.
    // A bare element_array is also accepted, like bonemarkuplist.
    const dmx::Element* pmarkList = root->GetElement("physicsmarkuplist");
    const std::vector<dmx::ElementPtr>* markups =
        pmarkList ? pmarkList->GetElementArray("children")
                  : root->GetElementArray("physicsmarkuplist");

    if (pmarkList) {
        // physicsmodifierlist was folded in here; reject the old spelling
        // rather than silently ignoring a script's physics settings
        if (root->Get("physicsmodifierlist")) {
            if (err) *err = "physicsmodifierlist was merged into physicsmarkuplist "
                            "(its settings are now attributes on that element)";
            return false;
        }
        out.physMass = pmarkList->GetFloat("mass", 1.0f);
        out.physAutoMass = pmarkList->GetBool("automass", false);
        out.physMassCenterSet = GetVec3(pmarkList, "mass_center", out.physMassCenter);
        if (auto rb = pmarkList->GetString("root_bone"))
            out.physRootBone = *rb;
        out.physNoSelfCollisions = pmarkList->GetBool("no_self_collisions", false);
        out.physDamping = pmarkList->GetFloat("damping", 0.0f);
        out.physRotdamping = pmarkList->GetFloat("rotdamping", 0.0f);
        out.physInertia = pmarkList->GetFloat("inertia", 1.0f);
        out.physDrag = pmarkList->GetFloat("drag", -1.0f);
        out.physWeldPosition = pmarkList->GetFloat("weld_position", 0.0f);
        out.physWeldNormal = pmarkList->GetFloat("weld_normal", 0.999f);
        if (auto pn = pmarkList->GetString("phy_name"))
            out.physName = *pn;
        // animatedfriction: a nested element, so its presence is the enable -
        // there is no sensible "no ramp" value for the five numbers.
        if (const dmx::Element* af = pmarkList->GetElement("animatedfriction")) {
            out.physHasAnimatedFriction = true;
            out.physAnimFrictionMin = af->GetInt("min", 0);
            out.physAnimFrictionMax = af->GetInt("max", 0);
            out.physAnimFrictionTimeIn = af->GetFloat("time_in", 0.0f);
            out.physAnimFrictionTimeOut = af->GetFloat("time_out", 0.0f);
            out.physAnimFrictionTimeHold = af->GetFloat("time_hold", 0.0f);
        }
        if (out.physMass <= 0.0f && !out.physAutoMass) {
            if (err) *err = "physicsmarkuplist mass must be positive";
            return false;
        }
    } else if (root->Get("physicsmodifierlist")) {
        if (err) *err = "physicsmodifierlist was merged into physicsmarkuplist "
                        "(its settings are now attributes on that element)";
        return false;
    }

    if (markups) {
        for (const dmx::Element* e : *markups) {
            if (!e) continue;
            cm::PhysicsMarkup mk;
            mk.name = e->name;
            if (auto n = e->GetString("name"))
                mk.name = *n;
            if (auto b = e->GetString("bone"))
                mk.bonename = *b;
            if (mk.bonename.empty()) {
                if (err) *err = "PhysicsMarkup \"" + mk.name + "\" has no bone";
                return false;
            }
            // presence-checked: an authored 0 must beat the list default
            if (e->Get("mass_bias"))  { mk.massBias   = e->GetFloat("mass_bias", 1.0f);  mk.massBiasSet = true; }
            if (e->Get("inertia"))    { mk.inertia    = e->GetFloat("inertia", 1.0f);    mk.inertiaSet = true; }
            if (e->Get("damping"))    { mk.damping    = e->GetFloat("damping", 0.0f);    mk.dampingSet = true; }
            if (e->Get("rotdamping")) { mk.rotdamping = e->GetFloat("rotdamping", 0.0f); mk.rotdampingSet = true; }
            // ragdoll body partitioning (phase 4.1). Inert on a single body.
            mk.skip = e->GetBool("skip", false);
            if (auto mi = e->GetString("merge_into")) mk.mergeInto = *mi;
            if (mk.skip && !mk.mergeInto.empty()) {
                if (err) *err = "PhysicsMarkup \"" + mk.name +
                                "\" sets both skip and merge_into - pick one";
                return false;
            }
            // .pulsemdl still authors the pair on one bone; flatten it into the
            // shared flat list in declaration order ($physicscollide's order)
            if (auto cw = e->GetStringArray("collide_with"))
                for (const std::string& partner : *cw)
                    out.physCollidePairs.push_back({mk.bonename, partner});
            out.physMarkups.push_back(std::move(mk));
        }
    }

    // gamedatalist - model-level game data ($surfaceprop, $contents).
    if (const dmx::Element* gd = root->GetElement("gamedatalist")) {
        if (auto sp = gd->GetString("surfaceprop"))
            out.surfaceprop = *sp;

        // $contents. Each token adds bits and may clear others; the word starts
        // at CONTENTS_SOLID, so "grate" (which removes solid) yields grate
        // alone. A bare number sets raw bits.
        if (auto ct = gd->GetStringArray("contents")) {
            int add = 0, remove = 0;
            for (const std::string& tok : *ct) {
                if (_stricmp(tok.c_str(), "solid") == 0) {
                    add |= kContentsSolid;
                } else if (_stricmp(tok.c_str(), "grate") == 0) {
                    add |= kContentsGrate;
                    remove |= kContentsSolid;
                } else if (_stricmp(tok.c_str(), "ladder") == 0) {
                    add |= kContentsLadder;
                } else if (_stricmp(tok.c_str(), "monster") == 0) {
                    add |= kContentsMonster;
                } else if (_stricmp(tok.c_str(), "debris") == 0) {
                    add |= kContentsDebris;
                } else if (_stricmp(tok.c_str(), "notsolid") == 0) {
                    remove |= kContentsSolid;
                } else {
                    char* end = nullptr;
                    long v = strtol(tok.c_str(), &end, 0);
                    if (end == tok.c_str() || *end != '\0') {
                        if (err) *err = "gamedatalist: unknown contents value \"" +
                                        tok + "\"";
                        return false;
                    }
                    add |= static_cast<int>(v);
                }
            }
            out.contents = (out.contents | add) & ~remove;
        }
    }

    // ---- flex / morph registration (Phase 3) ----
    // Gather the top-level morphcontrollerlist / morphrulelist into the shared
    // ManualFlex block, then let flexreg.cpp walk the bodies. Morph controllers
    // and rules are GLOBAL (Source-2-style authoring); the block attaches to the
    // last morphed body, which only fixes the on-disk registration order.
    {
        ManualFlex manual;

        if (auto mcs = root->GetElementArray("morphcontrollerlist")) {
            for (const dmx::Element* mc : *mcs) {
                if (!mc) continue;
                ManualFlex::Controller ctrl;
                ctrl.name = mc->name;
                if (auto n = mc->GetString("name"))
                    ctrl.name = *n;
                if (auto g = mc->GetString("group"))
                    if (!g->empty()) ctrl.group = *g;
                ctrl.min = mc->GetFloat("min", 0.0f);
                ctrl.max = mc->GetFloat("max", 1.0f);
                manual.controllers.push_back(std::move(ctrl));
            }
        }

        if (auto mrs = root->GetElementArray("morphrulelist")) {
            for (const dmx::Element* mr : *mrs) {
                if (!mr) continue;
                std::string name = mr->name;
                if (auto n = mr->GetString("name"))
                    name = *n;

                if (mr->className == "MorphSplitStereo") {
                    manual.stereoSplits.push_back({name, mr->GetFloat("splitfactor", 0.0f)});
                    continue;
                }
                if (mr->className == "MorphDominationRule") {
                    ManualFlex::Domination dom;
                    dom.name = name;
                    if (auto ds = mr->GetStringArray("dominators"))
                        for (const std::string& d : *ds)
                            if (!d.empty())
                                dom.dominators.push_back(d);
                    if (dom.dominators.empty()) {
                        if (err) *err = "MorphDominationRule \"" + name +
                                        "\" has no dominators";
                        return false;
                    }
                    manual.dominations.push_back(std::move(dom));
                    continue;
                }

                ManualFlex::Rule rule;
                rule.name = name;
                if (mr->className == "MorphLocalVar") {
                    rule.localvar = true;
                } else if (mr->className == "MorphRule") {
                    if (auto e = mr->GetString("expr"))
                        rule.expr = *e;
                    if (rule.expr.empty()) {
                        if (err) *err = "MorphRule \"" + rule.name + "\" has no expr";
                        return false;
                    }
                } else {
                    if (err) *err = "unknown morphrulelist element class \"" + mr->className +
                                    "\" (expected MorphRule/MorphLocalVar/"
                                    "MorphDominationRule/MorphSplitStereo)";
                    return false;
                }
                manual.rules.push_back(std::move(rule));
            }
        }

        if (!RegisterFlex(out, manual, err))
            return false;
    }

    // animationlist: Declares first (they pin indices), then Animations
    // (named, reusable), then Sequences
    std::map<std::string, int> namedAnims; // Animation element name -> anim idx
    if (auto anims = root->GetElementArray("animationlist")) {
        // pass 0: forward declarations ($declaresequence / $declareanimation)
        for (const dmx::Element* e : *anims) {
            if (!e) continue;
            if (e->className == "DeclareAnimation") {
                cm::CompileInput::InAnim a;
                a.name = e->name;
                if (auto n = e->GetString("name"))
                    a.name = *n;
                a.isDeclare = true;
                a.flags |= kStudioOverride;
                namedAnims[a.name] = static_cast<int>(out.anims.size());
                out.anims.push_back(std::move(a));
            } else if (e->className == "DeclareSequence") {
                cm::CompileInput::InSequence seq;
                seq.name = e->name;
                if (auto n = e->GetString("name"))
                    seq.name = *n;
                seq.isDeclare = true;
                seq.flags |= kStudioOverride;
                out.sequences.push_back(std::move(seq));
            } else if (e->className == "IncludeModel") {
                // $includemodel: borrow another compiled model's sequences.
                // The link happens at runtime, so all we store is the name.
                auto fn = e->GetString("filename");
                if (!fn || fn->empty()) {
                    if (err) *err = "IncludeModel \"" + e->name +
                                    "\" has no filename";
                    return false;
                }
                // must name a compiled .mdl - a source .dmx or a bare name
                // would silently produce a reference the engine cannot resolve
                if (fn->size() < 4 ||
                    _stricmp(fn->c_str() + fn->size() - 4, ".mdl") != 0) {
                    if (err) *err = "IncludeModel filename \"" + *fn +
                                    "\" must end in .mdl";
                    return false;
                }
                if (out.includeModels.size() >=
                    static_cast<size_t>(lim::kMaxIncludeModels)) {
                    if (err) *err = "too many IncludeModel entries";
                    return false;
                }
                out.includeModels.push_back("models/" + *fn);
            }
        }

        // shared Animation-field reader (used by Animation elements and the
        // implied animation of a Sequence)
        auto readAnimFields = [&](const dmx::Element* e,
                                  cm::CompileInput::InAnim& a) -> bool {
            // InAnim carries an ordered command list; this format has no
            // ordering of its own, so emit the fixed weightlist -> subtract ->
            // reverse order the elements always implied.
            using InCmd = cm::CompileInput::InAnim::InCmd;
            a.fps = e->GetFloat("fps", 30.0f);
            if (e->GetBool("delta", false))
                a.flags |= kStudioDelta;

            // per-animation transform, replacing the modelmodifierlist one for
            // this animation only. Same vocabulary as translatemodel/
            // rotatemodel/scale, including rotate's +90 roll composition.
            pm::Vector3 t;
            if (GetVec3(e, "translate", t)) {
                a.adjust = t;
                a.adjustSet = true;
            }
            pm::Vector3 r;
            if (GetVec3(e, "rotate", r)) {
                a.rotation = {r.x * pm::kDeg2Rad, r.y * pm::kDeg2Rad,
                              (r.z + 90.0f) * pm::kDeg2Rad};
                a.rotationSet = true;
            }
            a.scale = e->GetFloat("scale", 1.0f);
            a.ignorescale = e->GetBool("ignorescale", false);
            // keep this clip whatever animation_cull_type says
            a.nocull = e->GetBool("nocull", false);

            // clip trim. endframe -1 keeps the source clip's end; the compile
            // stage clamps both ends to whichever clip is sampled.
            a.startframe = e->GetInt("startframe", 0);
            a.endframe = e->GetInt("endframe", -1);
            if (a.startframe < 0) {
                if (err) *err = "Animation \"" + a.name + "\" startframe must be >= 0";
                return false;
            }
            if (a.endframe >= 0 && a.endframe < a.startframe) {
                if (err) *err = "Animation \"" + a.name + "\" endframe before startframe";
                return false;
            }
            if (auto wl = e->GetString("weightlist")) {
                InCmd cmd;
                cmd.kind = InCmd::Weights;
                cmd.name = *wl;
                a.cmds.push_back(std::move(cmd));
            }
            if (auto sub = e->GetString("subtract")) {
                InCmd cmd;
                cmd.kind = InCmd::Subtract;
                cmd.name = *sub;
                cmd.frame = e->GetInt("subtract_frame", 0);
                cmd.presubtract = e->GetBool("presubtract", false);
                a.cmds.push_back(std::move(cmd));
            }
            if (e->GetBool("reverse", false))
                a.cmds.push_back({InCmd::Reverse});
            return true;
        };

        // pass 1: standalone Animation elements
        for (const dmx::Element* e : *anims) {
            if (!e || e->className != "Animation") continue;
            cm::CompileInput::InAnim a;
            a.name = e->name;
            if (auto n = e->GetString("name"))
                a.name = *n;
            auto fn = e->GetString("filename");
            if (!fn || fn->empty()) {
                if (err) *err = "Animation \"" + a.name + "\" has no filename";
                return false;
            }
            a.source = LoadSource(out, scriptDir, *fn, err, /*morphSource=*/false,
                                  source::LoadKind::Animation);
            if (!a.source)
                return false;
            if (!readAnimFields(e, a))
                return false;
            namedAnims[a.name] = static_cast<int>(out.anims.size());
            out.anims.push_back(std::move(a));
        }

        // resolve one blend entry: Animation-element name -> reuse, else load
        // the file as an implied animation (settings copied from the element)
        auto resolveAnimRef = [&](const dmx::Element* e, const std::string& seqName,
                                  const std::string& ref, int blendIdx,
                                  int& outIdx) -> bool {
            auto named = namedAnims.find(ref);
            if (named != namedAnims.end()) {
                outIdx = named->second;
                return true;
            }
            cm::CompileInput::InAnim a;
            // implied name: ALWAYS "@<seq>" - the reference names every
            // implied blend anim identically (ProcessImpliedAnimation);
            // the string table dedups the duplicates
            (void)blendIdx;
            a.name = "@" + seqName;
            a.source = LoadSource(out, scriptDir, ref, err, /*morphSource=*/false,
                                  source::LoadKind::Animation);
            if (!a.source)
                return false;
            if (!readAnimFields(e, a))
                return false;
            outIdx = static_cast<int>(out.anims.size());
            out.anims.push_back(std::move(a));
            return true;
        };

        // pass 2: Sequences
        for (const dmx::Element* e : *anims) {
            if (!e || e->className != "Sequence") continue;
            cm::CompileInput::InSequence seq;
            seq.name = e->name;
            if (auto n = e->GetString("name"))
                seq.name = *n;

            // sequence flags / options ("loop" is an ANIMATION option in QC -
            // it lands on blend anim 0's flags below; the sequence inherits it
            // through the flags union in the compile stage)
            bool seqLoop = e->GetBool("loop", false);
            if (e->GetBool("snap", false)) seq.flags |= kStudioSnap;
            if (e->GetBool("autoplay", false)) seq.flags |= kStudioAutoplay;
            if (e->GetBool("realtime", false)) seq.flags |= kStudioRealtime;
            if (e->GetBool("hidden", false)) seq.flags |= kStudioHidden;
            // sequence-level delta = STUDIO_DELTA|STUDIO_POST; predelta = DELTA
            if (e->GetBool("delta", false)) seq.flags |= kStudioDelta | kStudioPost;
            if (e->GetBool("predelta", false)) seq.flags |= kStudioDelta;
            if (auto act = e->GetString("activity")) {
                seq.activityname = *act;
                seq.actweight = e->GetInt("activity_weight", -1);
            }
            seq.fadeintime = e->GetFloat("fadein", 0.2f);
            seq.fadeouttime = e->GetFloat("fadeout", 0.2f);
            if (auto wl = e->GetString("weightlist"))
                seq.weightlist = *wl;
            seq.blendwidth = e->GetInt("blendwidth", 0);

            // animations: blendlist (multi-anim grid) or single filename
            if (auto bl = e->GetStringArray("blendlist"); bl && !bl->empty()) {
                int idx = 0;
                for (const std::string& ref : *bl) {
                    int animIdx = -1;
                    if (!resolveAnimRef(e, seq.name, ref, idx, animIdx))
                        return false;
                    seq.blendAnims.push_back(animIdx);
                    idx++;
                }
                seq.animIndex = seq.blendAnims[0];
            } else {
                auto fn = e->GetString("filename");
                if (!fn || fn->empty()) {
                    if (err) *err = "Sequence \"" + seq.name + "\" has no filename";
                    return false;
                }
                int animIdx = -1;
                if (!resolveAnimRef(e, seq.name, *fn, -1, animIdx))
                    return false;
                seq.animIndex = animIdx;
            }

            // loop mutates blend anim 0 (QC ParseAnimationToken semantics -
            // named Animation elements included)
            if (seqLoop && seq.animIndex >= 0)
                out.anims[seq.animIndex].flags |= kStudioLooping;

            // children: BlendParameter / AddLayer / BlendLayer / IKLock / IKRule
            if (auto children = e->GetElementArray("children")) {
                for (const dmx::Element* c : *children) {
                    if (!c) continue;
                    if (c->className == "BlendParameter") {
                        if (seq.numblendparams >= 2) {
                            if (err) *err = "Sequence \"" + seq.name +
                                            "\": more than 2 BlendParameters";
                            return false;
                        }
                        auto& bp = seq.blendparams[seq.numblendparams++];
                        if (auto p = c->GetString("parameter"))
                            bp.parameter = *p;
                        bp.min = c->GetFloat("min", 0.0f);
                        bp.max = c->GetFloat("max", 0.0f);
                    } else if (c->className == "AddLayer") {
                        cm::CompileInput::InAutoLayer al;
                        if (auto s = c->GetString("sequence"))
                            al.sequence = *s;
                        if (c->GetBool("local", false)) {
                            al.flags |= kStudioAlLocal;
                            seq.flags |= kStudioLocal;
                        }
                        seq.autolayers.push_back(std::move(al));
                    } else if (c->className == "BlendLayer") {
                        cm::CompileInput::InAutoLayer al;
                        if (auto s = c->GetString("sequence"))
                            al.sequence = *s;
                        al.start = c->GetFloat("startframe", 0.0f);
                        al.peak = c->GetFloat("peakframe", 0.0f);
                        al.tail = c->GetFloat("tailframe", 0.0f);
                        al.end = c->GetFloat("endframe", 0.0f);
                        if (c->GetBool("spline", false)) al.flags |= kStudioAlSpline;
                        if (c->GetBool("xfade", false)) al.flags |= kStudioAlXfade;
                        if (c->GetBool("noblend", false)) al.flags |= kStudioAlNoblend;
                        if (c->GetBool("local", false)) {
                            al.flags |= kStudioAlLocal;
                            seq.flags |= kStudioLocal;
                        }
                        if (auto pp = c->GetString("poseparameter")) {
                            al.flags |= kStudioAlPose;
                            al.poseparameter = *pp;
                        }
                        seq.autolayers.push_back(std::move(al));
                    } else if (c->className == "IKLock") {
                        cm::IkLock lock;
                        if (auto ch = c->GetString("chain"))
                            lock.name = *ch;
                        lock.flPosWeight = c->GetFloat("position_weight", 0.0f);
                        lock.flLocalQWeight = c->GetFloat("rotation_weight", 0.0f);
                        seq.iklocks.push_back(std::move(lock));
                    } else if (c->className == "IKRule") {
                        cm::CompileInput::InIkRule rule;
                        if (auto ch = c->GetString("chain"))
                            rule.chain = *ch;
                        if (auto t = c->GetString("type"))
                            rule.type = *t;
                        if (auto tb = c->GetString("touch_bone"))
                            rule.touchBone = *tb;
                        if (auto at = c->GetString("attachment"))
                            rule.attachment = *at;
                        if (const dmx::Attribute* h = c->Get("height")) {
                            rule.height = c->GetFloat("height", 0.0f);
                            rule.heightSet = true;
                        }
                        if (const dmx::Attribute* f = c->Get("floor")) {
                            rule.floor = c->GetFloat("floor", 0.0f);
                            rule.floorSet = true;
                        }
                        if (const dmx::Attribute* p = c->Get("pad")) {
                            rule.radius = c->GetFloat("pad", 0.0f) / 2.0f;
                            rule.radiusSet = true;
                        }
                        if (const dmx::Attribute* r = c->Get("radius")) {
                            rule.radius = c->GetFloat("radius", 0.0f);
                            rule.radiusSet = true;
                        }
                        // QC defaults: ranges 0 (the "all zero" cascade fires
                        // when no range is given); -1 = the '.' placeholder
                        rule.contact = c->GetInt("contact", -1);
                        rule.startframe = c->GetInt("startframe", 0);
                        rule.peakframe = c->GetInt("peakframe", 0);
                        rule.tailframe = c->GetInt("tailframe", 0);
                        rule.endframe = c->GetInt("endframe", 0);
                        rule.usesequence = c->GetBool("usesequence", false);
                        if (c->GetBool("usesource", false)) {
                            rule.usesource = true;
                            rule.usesequence = false;
                        }
                        pm::Vector3 v;
                        if (GetVec3(c, "fakeorigin", v)) {
                            rule.fakeorigin = v;
                            rule.fakeoriginSet = true;
                        }
                        if (GetVec3(c, "fakerotate", v)) {
                            rule.fakerotate = v;
                            rule.fakerotateSet = true;
                        }
                        seq.ikrules.push_back(std::move(rule));
                    } else {
                        if (err) *err = "Sequence \"" + seq.name +
                                        "\": unknown child element class \"" +
                                        c->className + "\"";
                        return false;
                    }
                }
            }

            out.sequences.push_back(std::move(seq));
        }
    }
    out.upAxisY = source::DmxUpAxisY();

    // attachmentlist ($attachment). Parsed after the sources so absolute
    // attachments can bake the FINAL model rotation (QC evaluates
    // g_defaultrotation at $attachment parse time; the declarative schema
    // always uses the final value).
    // facemarkuplist: ordered, mixed-class (Eyeball / Mouth / Eyelid). Order
    // matters - it fixes the eyeball index an Eyelid later names, and Mouth and
    // Eyelid both append to the global flexdesc table. Gathered into the shared
    // FaceMarkup block; facemarkup.cpp does the registering.
    if (auto fms = root->GetElementArray("facemarkuplist")) {
        FaceMarkup markup;
        for (const dmx::Element* fm : *fms) {
            if (!fm) continue;
            FaceMarkup::Entry entry;

            if (fm->className == "Eyeball") {
                entry.kind = FaceMarkup::Kind::Eyeball;
                entry.eyeball.name = fm->name;
                if (auto n = fm->GetString("name"))
                    entry.eyeball.name = *n;
                if (auto b = fm->GetString("bone"))
                    entry.eyeball.bonename = *b;
                if (auto m = fm->GetString("material"))
                    entry.eyeball.material = *m;
                GetVec3(fm, "origin", entry.eyeball.origin);
                entry.eyeball.diameter = fm->GetFloat("diameter", 0.0f);
                entry.eyeball.angle = fm->GetFloat("angle", 0.0f);
                entry.eyeball.pupilscale = fm->GetFloat("pupilscale", 0.0f);
                markup.entries.push_back(std::move(entry));
                continue;
            }

            if (fm->className == "Mouth") {
                entry.kind = FaceMarkup::Kind::Mouth;
                if (auto c = fm->GetString("controller"))
                    entry.mouth.controller = *c;
                if (auto b = fm->GetString("bone"))
                    entry.mouth.bonename = *b;
                GetVec3(fm, "forward", entry.mouth.forward);
                markup.entries.push_back(std::move(entry));
                continue;
            }

            if (fm->className == "Eyelid") {
                entry.kind = FaceMarkup::Kind::Eyelid;
                std::string type;
                if (auto t = fm->GetString("type"))
                    type = *t;
                if (_stricmp(type.c_str(), "upper") != 0 &&
                    _stricmp(type.c_str(), "lower") != 0) {
                    if (err) *err = "Eyelid \"type\" must be \"upper\" or \"lower\"";
                    return false;
                }
                entry.eyelid.upper = _stricmp(type.c_str(), "upper") == 0;

                static const char* kSuffix[3] = {"lowerer", "neutral", "raiser"};
                for (int i = 0; i < 3; ++i) {
                    if (auto d = fm->GetString(kSuffix[i]))
                        entry.eyelid.delta[i] = *d;
                    // targets are scaled at parse time (`* g_currentscale`)
                    entry.eyelid.target[i] =
                        fm->GetFloat(std::string(kSuffix[i]) + "_target", 0.0f) * out.scale;
                }
                if (auto r = fm->GetString("righteyeball"))
                    entry.eyelid.righteyeball = *r;
                if (auto l = fm->GetString("lefteyeball"))
                    entry.eyelid.lefteyeball = *l;
                markup.entries.push_back(std::move(entry));
                continue;
            }

            if (err) *err = "unknown facemarkuplist element class \"" + fm->className +
                            "\" (expected Eyeball/Mouth/Eyelid)";
            return false;
        }
        if (!RegisterFaceMarkup(out, markup, err))
            return false;
    }

    if (auto atts = root->GetElementArray("attachmentlist")) {
        pm::RadianEuler defaultRot =
            out.rotationSet
                ? out.rotation
                : (out.upAxisY ? pm::RadianEuler{static_cast<float>(kPiD / 2.0), 0.0f,
                                                 static_cast<float>(kPiD / 2.0)}
                               : pm::RadianEuler{0.0f, 0.0f, static_cast<float>(kPiD / 2.0)});

        for (const dmx::Element* e : *atts) {
            if (!e) continue;
            cm::Attachment att;
            att.name = e->name;
            if (auto n = e->GetString("name"))
                att.name = *n;
            auto bone = e->GetString("parent_bone");
            if (!bone || bone->empty()) {
                if (err) *err = "attachment \"" + att.name + "\" has no parent_bone";
                return false;
            }
            att.bonename = *bone;

            // position is scaled like a vertex (scale_vertex)
            pm::Vector3 t;
            GetVec3(e, "translation", t);
            t = {t.x * out.scale, t.y * out.scale, t.z * out.scale};

            // local matrix: identity; "absolute" bases it on the inverse model
            // rotation; an explicit rotation (degrees) overrides the rotation
            // part (QC option order: last one wins - rotation is authoritative
            // here).
            if (e->GetBool("is_absolute", false)) {
                att.type |= cm::kAttachIsAbsolute;
                pm::AngleIMatrix(defaultRot, att.local);
            }
            if (e->GetBool("is_rigid", false))
                att.type |= cm::kAttachIsRigid;
            if (e->GetBool("world_align", false))
                att.flags |= cm::kAttachFlagWorldAlign;

            pm::Vector3 rotDeg;
            if (GetVec3(e, "rotation", rotDeg))
                pm::AngleMatrixDeg(rotDeg, att.local); // (pitch, yaw, roll)

            // position stuffed in AFTER the rotation options
            att.local.m[0][3] = t.x;
            att.local.m[1][3] = t.y;
            att.local.m[2][3] = t.z;

            out.attachments.push_back(std::move(att));
        }
    }
    return true;
}

} // namespace pulse::loader
