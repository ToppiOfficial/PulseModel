// mdlasmload.cpp - see mdlasmload.h.
//
// One pass per document category, in the order the commands depend on each other: scale is baked
// into a source as it loads, $collapsematerial must precede that load, $renamematerial follows it
// and attachments finish last (FinishScript). A new node is one decoder, one pass and one table row.

#include "mdlasmload.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>

#include "dependencies.h"
#include "diagnostic.h"
#include "mdlasmanim.h"
#include "mdlasmdoc.h"
#include "mdlasmflex.h"
#include "pathcompat.h"
#include "scriptapply.h"
#include "strcompat.h"

namespace pulse::loader {
namespace {

using namespace detail;
namespace md = pulse::mdlasm;

constexpr const char* kPrefix = "mdlasm:";

// What a pass works with: the document, the loader state and the labels diagnostics use.
struct Load {
    Ctx& c;
    const md::Document& d;

    // Registers a node so a failure names it; the returned value is the "line" every Apply takes.
    int Node(const std::string& label) {
        c.nodeLabels.push_back(label);
        return static_cast<int>(c.nodeLabels.size()) - 1;
    }

    // A warning that names the node it came from.
    void Warn(int line, const std::string& text) {
        const std::string& label = c.nodeLabels[static_cast<size_t>(line)];
        pulse::diag::WarnAt({}, label.empty() ? text : label + ": " + text);
    }
};

using Pass = bool (*)(Load&);

// The turn a twist trigger appends to the driver and helper about their own axes.
// Converts through double like DegToRad.
pm::Quaternion TwistRotation(md::TwistAxis axis, float degrees) {
    const float half = static_cast<float>(degrees * 3.14159265358979323846 / 180.0) * 0.5f, s = std::sin(half);
    return {axis == md::TwistAxis::X ? s : 0.0f, axis == md::TwistAxis::Y ? s : 0.0f,
            axis == md::TwistAxis::Z ? s : 0.0f, std::cos(half)};
}

// Position plus (pitch, yaw, roll) degrees as the bind-pose matrix $definebone reads.
pm::matrix3x4 Sextet(const pm::Vector3& origin, const pm::Vector3& angles) {
    pm::matrix3x4 out;
    pm::AngleMatrixDeg(angles, out);
    out.m[0][3] = origin.x;
    out.m[1][3] = origin.y;
    out.m[2][3] = origin.z;
    return out;
}

std::string Quoted(const std::string& name) { return "'" + name + "'"; }

// The name a command refers to an embedded source by. It keeps the original extension.
std::string VirtualName(const md::SourceFile& source) {
    std::string file = pulse::FilePath(source.path).filename().string();
    const std::string extension = pulse::FilePath(file).extension().string();
    if (extension.empty() || (_stricmp(extension.c_str(), ".dmx") != 0 && _stricmp(extension.c_str(), ".smd") != 0 &&
                              _stricmp(extension.c_str(), ".fbx") != 0))
        file += ".dmx";
    return std::string(kPrefix) + std::to_string(source.uid) + "/" + file;
}

// ---------------------------------------------------------------------------
// Model Definition and Render Definition
// ---------------------------------------------------------------------------

bool Definition(Load& l) {
    Ctx& c = l.c;
    const md::ModelDefinition& def = l.d.definition;
    const int line = l.Node("Model Definition");

    if (!ApplyModelName(c, line, def.name))
        return false;

    if (!def.surfaceProp.empty())
        c.in.surfaceprop = def.surfaceProp;
    c.in.scale = def.scale;
    c.in.adjust = def.origin;
    c.in.archetype = def.archetype;
    c.in.fitBoneWeightPoses = def.weightFitPoses;
    c.in.boneCullType = def.boneCullMethod;

    // Auto leaves the rotation to the DMX up axis, like a script with no rotation command.
    constexpr float quarter = pm::kPiF / 2.0f;
    pm::RadianEuler up{0.0f, 0.0f, quarter};
    switch (def.rotationMode) {
    case md::RotationMode::Auto:
        break;
    case md::RotationMode::Angles:
        c.in.rotation = {def.angles.x * pm::kDeg2Rad, def.angles.y * pm::kDeg2Rad,
                         (def.angles.z + 90.0f) * pm::kDeg2Rad};
        c.in.rotationSet = true;
        break;
    case md::RotationMode::UpX: up.y = quarter; break;
    case md::RotationMode::UpNegX: up.y = -quarter; break;
    case md::RotationMode::UpY: up.x = quarter; break;
    case md::RotationMode::UpNegY: up.x = -quarter; break;
    case md::RotationMode::UpZ: break;
    }
    if (def.rotationMode != md::RotationMode::Auto && def.rotationMode != md::RotationMode::Angles) {
        c.in.rotation = up;
        c.in.rotationSet = true;
    }

    if (def.bboxSet) {
        c.in.bbox[0] = def.bbox[0];
        c.in.bbox[1] = def.bbox[1];
        c.in.bboxSet = true;
    }
    if (def.cboxSet) {
        c.in.cbox[0] = def.cbox[0];
        c.in.cbox[1] = def.cbox[1];
        c.in.cboxSet = true;
    }
    if (def.illumSet && !ApplyIllumPosition(c, line, def.illum, def.illumBone))
        return false;

    // The eye position is already in model axes; the compile stage applies $scale. A deflection of 30
    // is the engine's own default, which a script leaves unwritten.
    if (def.eyeEnabled && def.eyeActive) {
        c.in.eyeposition = def.eyePosition;
        c.in.eyepositionAutoHeight = def.eyeAutoHeight;
        if (def.eyeMaxDeflection != 30.0f)
            c.in.maxEyeDeflection = std::cos(def.eyeMaxDeflection * pm::kDeg2Rad);
    }
    return true;
}

bool RenderDefinition(Load& l) {
    const md::RenderDefinition& render = l.d.definition.render;
    if (!render.present || !render.active)
        return true;

    Ctx& c = l.c;
    c.in.renderPass = static_cast<int>(render.pass);
    c.in.ambientBoost = render.ambientBoost;
    c.in.doNotCastShadows = render.doNotCastShadows;
    c.in.castTextureShadows = render.castTextureShadows;
    c.in.noForcedFade = render.noForcedFade;
    c.in.forcePhonemeCrossfade = render.forcePhonemeCrossfade;
    if (render.constantDirectionalLight) {
        c.in.constDirLight = true;
        c.in.constDirLightDot = static_cast<uint8_t>(render.directionalLightScale * 255.0f);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

// Before any source loads: a collapse is resolved as faces load.
bool MaterialCollapses(Load& l) {
    for (const md::MaterialRule& rule : l.d.materials.collapses) {
        if (!rule.complete()) continue;

        const int line = l.Node("Material Markup: collapse " + Quoted(rule.from));
        if (!ApplyCollapseMaterial(l.c, line, rule.from, rule.to))
            return false;
    }
    return true;
}

bool MaterialDirectories(Load& l) {
    for (const std::string& directory : l.d.materials.directories)
        l.c.in.cdmaterials.push_back(directory);
    return true;
}

// After every source has loaded: a rename rewrites a loaded texture.
bool MaterialRenames(Load& l) {
    for (const md::MaterialRule& rule : l.d.materials.renames) {
        if (!rule.complete()) continue;

        const int line = l.Node("Material Markup: rename " + Quoted(rule.from));
        if (!ApplyRenameMaterial(l.c, line, rule.from, rule.to))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Bones
// ---------------------------------------------------------------------------

bool BoneDefinitions(Load& l) {
    Ctx& c = l.c;
    for (const md::BoneDefinition& bone : l.d.bones) {
        if (!bone.targetsBone()) continue;

        const int line = l.Node("Bone Definition: " + Quoted(bone.bone));
        if (bone.bind.active) {
            cm::ImportBone ib;
            ib.name = bone.bone;
            ib.parent = bone.bind.parent;
            ib.bUnlocked = bone.bind.unlocked;
            ib.rawLocal = Sextet(bone.bind.origin, bone.bind.angles);
            if (bone.bind.preAligned) {
                ib.bPreAligned = true;
                ib.srcRealign = Sextet(bone.bind.realignOrigin, bone.bind.realignAngles);
            }
            if (!ApplyDefineBone(c, line, std::move(ib)))
                return false;
        }

        if (bone.boneMerge) FindOrAddMarkup(c, bone.bone).isBonemerge = true;
        if (bone.doNotCollapse) FindOrAddMarkup(c, bone.bone).doNotCollapse = true;
        if (bone.alwaysCollapse) c.in.alwaysCollapse.push_back(bone.bone);
    }
    return true;
}

// The Realign Bone nodes in list order, each from the result of the one before on its bone.
bool RealignBones(Load& l) {
    for (const md::RealignNode& node : l.d.realigns) {
        if (!node.targetsBone()) continue;

        const int line = l.Node("Realign Bone: " + Quoted(node.name.empty() ? node.bone : node.name));
        cm::BoneRealignEdit edit = node.edit;
        edit.line = line;
        if (!ApplyRealignBone(l.c, line, node.bone, edit))
            return false;
    }
    return true;
}

// After every realign, so a Bone Definition's No realign drops the realigns of its bone.
bool NoRealign(Load& l) {
    for (const md::BoneDefinition& bone : l.d.bones) {
        if (!bone.targetsBone() || !bone.noRealign) continue;

        cm::BoneMarkup& markup = FindOrAddMarkup(l.c, bone.bone);
        markup.noRealign = true;
        markup.realigns.clear();
    }
    return true;
}

// The Bone Hierarchy nodes in list order; EnforceHierarchy applies them once the bone table exists.
bool Hierarchies(Load& l) {
    for (const md::HierarchyNode& node : l.d.hierarchies) {
        if (!node.targetsBone()) continue;

        l.c.in.forcedHierarchy.push_back({node.bone, node.parent});
    }
    return true;
}

// ---------------------------------------------------------------------------
// Render meshes and model groups
// ---------------------------------------------------------------------------

// The stream requests the enabled $meshedit blocks make of the loader, as ParseEditMesh records them.
void RequestStreams(source::MeshFilter& filter, const std::vector<std::vector<source::EditMeshOperation>>& blocks) {
    filter.keepMeshIdentity = false;
    filter.selectMeshStreams = false;
    filter.editStreams.clear();

    const auto request = [&filter](const std::string& name, source::EditStreamKind kind, int component) {
        for (const auto& stream : filter.editStreams)
            if (stream.kind == kind && stream.component == component &&
                _stricmp(stream.name.c_str(), name.c_str()) == 0)
                return;

        filter.editStreams.push_back({name, component, false, kind});
    };

    using source::EditStreamKind;
    using source::MeshOperation;
    for (const auto& block : blocks) {
        for (const auto& op : block) {
            switch (op.kind) {
            case MeshOperation::SelectMeshByStream:
                filter.selectMeshStreams = true;
                [[fallthrough]];
            case MeshOperation::SelectMesh:
                filter.keepMeshIdentity = true;
                break;
            case MeshOperation::SelectStream:
                request(op.name, op.channel < 0 ? EditStreamKind::Float : EditStreamKind::Channel, op.channel);
                break;
            case MeshOperation::SwitchUV:
                request(op.name, EditStreamKind::UV, 0);
                request(op.name, EditStreamKind::UV, 1);
                break;
            case MeshOperation::Inflate:
                if (!op.directionStream.empty())
                    for (int component = 0; component < 3; ++component)
                        request(op.directionStream, EditStreamKind::Channel, component);
                if (!op.weightStream.empty())
                    request(op.weightStream,
                            op.weightChannel < 0 ? EditStreamKind::Float : EditStreamKind::Channel,
                            op.weightChannel);
                break;
            default:
                break;
            }
        }
    }
}

bool Meshes(Load& l) {
    Ctx& c = l.c;
    for (const md::MeshEntry& entry : l.d.meshes) {
        if (!entry.enabled) continue;

        const md::RenderMeshEdit& source = entry.edit;
        const int line = l.Node("Meshes: " + Quoted(source.name));
        const md::SourceFile* file = l.d.findSource(entry.source);
        if (!file || !file->bytes)
            return c.Fail(line, file ? "the source has no data" : "the mesh has no source");
        if (c.rendermeshes.count(source.name))
            return c.Fail(line, "duplicate $rendermesh \"" + source.name + "\"");

        MeshEdit edit;
        edit.name = source.name;
        edit.filter = source.filter;
        edit.noMorph = source.noMorph;
        edit.skeletalAwareDecimation = source.skeletalAwareDecimation;
        edit.decimates = source.decimates;
        for (const md::MeshEditBlock& block : source.editBlocks) {
            if (!block.enabled) continue;

            if (block.skeleton) {
                edit.skeletonBlocks.push_back(block.skeletonOperations);
                for (auto& op : edit.skeletonBlocks.back()) op.line = line;
                edit.editOrder.emplace_back(true, edit.skeletonBlocks.size() - 1);
            } else {
                edit.editBlocks.push_back(block.operations);
                for (auto& op : edit.editBlocks.back()) op.line = line;
                edit.editOrder.emplace_back(false, edit.editBlocks.size() - 1);
            }
        }
        RequestStreams(edit.filter, edit.editBlocks);

        if (!ApplyRenderMesh(c, line, source.name, VirtualName(*file), edit))
            return false;
    }
    return true;
}

bool Groups(Load& l) {
    Ctx& c = l.c;
    for (const md::ModelGroup& group : l.d.groups) {
        if (!group.enabled) continue;

        const int line = l.Node("Model Group: " + Quoted(group.name));
        const std::string where = "$modelgroup \"" + group.name + "\"";
        cm::CompileInput::InBodyPart part;
        part.name = group.name;

        for (const md::ModelGroupChoice& choice : group.choices) {
            std::vector<source::Source*> refs;
            std::vector<std::string> refNames;
            for (const md::Uid uid : choice.meshes) {
                const md::MeshEntry* mesh = l.d.findMesh(uid);
                if (!mesh) return c.Fail(line, where + ": a choice names a mesh that no longer exists");
                if (!mesh->enabled) {
                    l.Warn(line, "mesh \"" + mesh->edit.name + "\" is disabled and is skipped");
                    continue;
                }

                const auto loaded = c.rendermeshes.find(mesh->edit.name);
                if (loaded == c.rendermeshes.end())
                    return c.Fail(line, where + ": mesh \"" + mesh->edit.name + "\" was not loaded");
                refs.push_back(loaded->second);
                refNames.push_back(mesh->edit.name);
            }

            if (choice.meshes.empty()) {
                cm::CompileInput::InModel blank;
                blank.name = "blank";
                part.models.push_back(std::move(blank));
                continue;
            }
            if (!ApplyModelChoice(c, line, where, refs, refNames, {}, !choice.name.empty(), choice.name, part))
                return false;
        }
        c.in.bodyparts.push_back(std::move(part));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Attachments, hitboxes and the face
// ---------------------------------------------------------------------------

// The origin stays as authored; the compile stage scales it when attachments finish.
bool Attachments(Load& l) {
    for (const md::Attachment& attachment : l.d.attachments) {
        if (!attachment.enabled) continue;

        const int line = l.Node("Attachment: " + Quoted(attachment.name));
        Ctx::PendingAttachment a;
        a.name = attachment.name;
        a.bone = attachment.bone;
        a.origin = attachment.origin;
        a.hasAngles = attachment.anglesSet;
        if (attachment.anglesSet) a.anglesDeg = attachment.angles;
        if (attachment.rigid) a.type |= cm::kAttachIsRigid;
        if (attachment.absolute) a.type |= cm::kAttachIsAbsolute;
        if (attachment.worldAlign) a.flags |= cm::kAttachFlagWorldAlign;
        if (!ApplyAttachment(l.c, std::move(a)))
            return false;
    }
    return true;
}

// An enabled set is written even when empty: declaring any set turns automatic hitboxes off.
bool Hitboxes(Load& l) {
    for (const md::HitboxSet& set : l.d.hitboxSets) {
        if (!set.enabled) continue;

        const int line = l.Node("Hitbox Set: " + Quoted(set.name));
        cm::HitboxSet out;
        out.name = set.name;
        for (const md::Hitbox& box : set.boxes) {
            if (!box.enabled) continue;

            cm::HitBox hb;
            hb.bonename = box.bone;
            hb.group = box.group;
            hb.bmin = box.min;
            hb.bmax = box.max;
            hb.angOffset = box.angles;
            if (box.capsule) hb.capsuleRadius = box.radius;
            hb.name = box.name;
            if (!ApplyHbox(l.c, line, out, std::move(hb)))
                return false;
        }
        if (!ApplyHitboxSet(l.c, line, std::move(out)))
            return false;
    }
    return true;
}

// Eyeballs first: their order is the eyeball index an eyelid names.
bool Face(Load& l) {
    Ctx& c = l.c;
    for (const md::Eyeball& eyeball : l.d.eyeballs) {
        if (!eyeball.enabled) continue;

        l.Node("Eyeball: " + Quoted(eyeball.name));
        FaceMarkup::Entry entry;
        entry.kind = FaceMarkup::Kind::Eyeball;
        entry.eyeball.name = eyeball.name;
        entry.eyeball.bonename = eyeball.bone;
        entry.eyeball.material = eyeball.material;
        entry.eyeball.origin = eyeball.origin;
        entry.eyeball.center = eyeball.center;
        entry.eyeball.diameter = eyeball.diameter;
        entry.eyeball.angle = eyeball.angle;
        entry.eyeball.pupilscale = eyeball.pupilScale;
        c.face.entries.push_back(std::move(entry));
    }

    // A lid on both eyes is two per-eye entries, named like the Flex Graph names its lid descs.
    for (const md::EyelidNode& lid : l.d.eyelids) {
        if (!lid.enabled || lid.name.empty() || !lid.authored()) continue;

        l.Node("Eyelid: " + Quoted(lid.name));
        const bool stereo = !lid.rightEye.empty() && !lid.leftEye.empty();
        for (const bool left : {false, true}) {
            const std::string& eyeball = left ? lid.leftEye : lid.rightEye;
            if (eyeball.empty()) continue;

            FaceMarkup::Entry entry;
            entry.kind = FaceMarkup::Kind::Eyelid;
            entry.eyelid.upper = lid.type == md::EyelidType::Upper;
            for (std::size_t i = 0; i < 3; ++i) {
                entry.eyelid.delta[i] = lid.poses[i].morph;
                entry.eyelid.target[i] = lid.poses[i].height * c.in.scale;
            }
            entry.eyelid.split = lid.split;
            entry.eyelid.eyeball = eyeball;
            entry.eyelid.basedesc = stereo ? lid.name + (left ? "_left" : "_right") : lid.name;
            c.face.entries.push_back(std::move(entry));
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Bone markup: jigglebone, driverbone, driveraimat and ikchain
// ---------------------------------------------------------------------------

// Degrees to radians through a double, like the script reader.
float Radians(float degrees) { return static_cast<float>(degrees * 3.14159265358979323846 / 180.0); }

float Stiffness(float value) { return std::clamp(value, 0.0f, 1000.0f); }

cm::JiggleBone JiggleOf(const md::JiggleSettings& s, const std::string& bone) {
    cm::JiggleBone jb;
    jb.bonename = bone;

    // Both tip sections also set the length constraint, which allow_length_flex clears.
    if (s.style == md::JiggleStyle::Flexible) jb.flags |= cm::kJiggleIsFlexible | cm::kJiggleHasLengthConstraint;
    else if (s.style == md::JiggleStyle::Rigid) jb.flags |= cm::kJiggleIsRigid | cm::kJiggleHasLengthConstraint;

    if (s.style != md::JiggleStyle::None) {
        jb.length = s.length;
        jb.tipMass = s.tipMass;
    }

    if (s.style == md::JiggleStyle::Flexible) {
        jb.yawStiffness = Stiffness(s.yawStiffness);
        jb.yawDamping = Stiffness(s.yawDamping);
        jb.pitchStiffness = Stiffness(s.pitchStiffness);
        jb.pitchDamping = Stiffness(s.pitchDamping);
        if (s.lengthFlex) {
            jb.flags &= ~cm::kJiggleHasLengthConstraint;
            jb.alongStiffness = Stiffness(s.alongStiffness);
            jb.alongDamping = Stiffness(s.alongDamping);
        }
        if (s.angleConstraint) {
            jb.flags |= cm::kJiggleHasAngleConstraint;
            jb.angleLimit = Radians(s.angleLimit);
        }
        if (s.yawConstraint) {
            jb.flags |= cm::kJiggleHasYawConstraint;
            jb.minYaw = Radians(s.minYaw);
            jb.maxYaw = Radians(s.maxYaw);
            jb.yawFriction = s.yawFriction;
            jb.yawBounce = s.yawBounce;
        }
        if (s.pitchConstraint) {
            jb.flags |= cm::kJiggleHasPitchConstraint;
            jb.minPitch = Radians(s.minPitch);
            jb.maxPitch = Radians(s.maxPitch);
            jb.pitchFriction = s.pitchFriction;
            jb.pitchBounce = s.pitchBounce;
        }
    }

    if (s.base == md::JiggleBase::BaseSpring) {
        jb.flags |= cm::kJiggleHasBaseSpring;
        jb.baseStiffness = Stiffness(s.baseStiffness);
        jb.baseDamping = Stiffness(s.baseDamping);
        jb.baseMass = s.baseMass;
        if (s.baseLeftConstraint) {
            jb.baseMinLeft = s.baseMinLeft;
            jb.baseMaxLeft = s.baseMaxLeft;
            jb.baseLeftFriction = s.baseLeftFriction;
        }
        if (s.baseUpConstraint) {
            jb.baseMinUp = s.baseMinUp;
            jb.baseMaxUp = s.baseMaxUp;
            jb.baseUpFriction = s.baseUpFriction;
        }
        if (s.baseForwardConstraint) {
            jb.baseMinForward = s.baseMinForward;
            jb.baseMaxForward = s.baseMaxForward;
            jb.baseForwardFriction = s.baseForwardFriction;
        }
    } else if (s.base == md::JiggleBase::Boing) {
        // The 0.7071 literal is not cos(45 degrees), and the two differ in the .mdl bytes.
        jb.flags |= cm::kJiggleIsBoing;
        jb.boingImpactSpeed = s.boingImpactSpeed;
        jb.boingImpactAngle = s.boingImpactAngle == 45.0f ? 0.7071f : std::cos(Radians(s.boingImpactAngle));
        jb.boingDampingRate = s.boingDampingRate;
        jb.boingFrequency = s.boingFrequency;
        jb.boingAmplitude = s.boingAmplitude;
    }
    return jb;
}

bool BoneMarkups(Load& l) {
    Ctx& c = l.c;
    for (const md::BoneMarkupNode& node : l.d.boneMarkups) {
        if (!node.targetsBone()) continue;

        const int line = l.Node("Bone Markup: " + Quoted(node.bone));
        bool ok = true;
        if (const auto* jiggle = std::get_if<md::JiggleSettings>(&node.value)) {
            ok = ApplyJiggleBone(c, line, JiggleOf(*jiggle, node.bone));
        } else if (const auto* procedural = std::get_if<md::ProceduralSettings>(&node.value)) {
            if (procedural->mode == md::ProceduralMode::Driver) {
                if (procedural->driver.empty()) {
                    l.Warn(line, "no driver bone, skipped");
                    continue;
                }
                if (std::ranges::none_of(procedural->triggers, &md::ProceduralTrigger::enabled))
                    return c.Fail(line, "$driverbone \"" + node.bone + "\": no enabled triggers");

                // `helper` -1 is the entry's own bone; each bone reads its own pose from every trigger.
                const Token at{"$driverbone trigger", line, false};
                const auto build = [&](const std::string& name, float influenceOutput, int helper,
                                       cm::ProceduralBone& pb) {
                    pb.helpername = name;
                    pb.drivername = procedural->driver;
                    if (procedural->influenceInput > 0.0f) pb.influenceInput = procedural->influenceInput;
                    if (influenceOutput >= 0.0f) pb.influenceOutput = influenceOutput;
                    for (const md::ProceduralTrigger& trigger : procedural->triggers) {
                        if (!trigger.enabled) continue;

                        // The base position joins the trigger position before the model scale.
                        const md::HelperPose helperPose = trigger.helperPose(helper);
                        const pm::Vector3 pos{procedural->basePosition.x + helperPose.position.x,
                                              procedural->basePosition.y + helperPose.position.y,
                                              procedural->basePosition.z + helperPose.position.z};
                        if (!AddTrigger(c, at, pb, trigger.tolerance, trigger.driverAngles, helperPose.angles, pos,
                                        !procedural->local))
                            return false;
                        if (trigger.kind != md::TriggerKind::Twist) continue;

                        // AddTrigger drops a trigger past the cap, so only a pushed one is twisted.
                        for (const float sign : {1.0f, -1.0f}) {
                            const size_t before = pb.triggers.size();
                            if (!AddTrigger(c, at, pb, trigger.tolerance, trigger.driverAngles, helperPose.angles, pos,
                                            !procedural->local))
                                return false;
                            if (pb.triggers.size() == before) break;

                            auto& twisted = pb.triggers.back();
                            const float helperSign = trigger.invertHelperTwist ? -sign : sign;
                            const pm::Quaternion turn = TwistRotation(trigger.twistAxis, sign * trigger.twistAngle);
                            const pm::Quaternion helperTurn =
                                TwistRotation(trigger.twistAxis, helperSign * trigger.twistAngle);
                            const pm::Quaternion pose = twisted.trigger, turnedHelper = twisted.quat;
                            pm::QuaternionMult(pose, turn, twisted.trigger);
                            if (trigger.twistHelpers) pm::QuaternionMult(turnedHelper, helperTurn, twisted.quat);
                        }
                    }
                    return true;
                };

                // The entry's bone, then each additional bone with its own pose and influence out.
                cm::ProceduralBone pb;
                if (!build(node.bone, procedural->influenceOutput, -1, pb)) return false;
                ok = ApplyProceduralBone(c, line, std::move(pb));
                for (size_t i = 0; ok && i < procedural->helpers.size(); ++i) {
                    const md::ProceduralHelper& helper = procedural->helpers[i];
                    if (helper.bone.empty()) continue;

                    cm::ProceduralBone extra;
                    if (!build(helper.bone, helper.influenceOutput, int(i), extra)) return false;
                    ok = ApplyProceduralBone(c, line, std::move(extra));
                }
            } else {
                if (procedural->target.empty()) {
                    l.Warn(line, "no aim target, skipped");
                    continue;
                }
                cm::AimAtBone ab;
                ab.autobasepos = true;
                ab.bonename = node.bone;
                ab.aimname = procedural->target;
                ab.upvector = procedural->up;
                ab.aimvector = procedural->aim;
                pm::VectorNormalize(ab.upvector);
                pm::VectorNormalize(ab.aimvector);
                ok = ApplyAimAtBone(c, line, std::move(ab));
            }
        } else if (const auto* ik = std::get_if<md::IkSettings>(&node.value)) {
            if (ik->chain.empty())
                return c.Fail(line, "$ikchain: the chain needs a name");
            const bool duplicate = std::ranges::any_of(c.in.ikchains, [&](const cm::IkChain& chain) {
                return _stricmp(chain.name.c_str(), ik->chain.c_str()) == 0;
            });
            if (duplicate) {
                l.Warn(line, "duplicate ikchain \"" + ik->chain + "\" ignored");
                continue;
            }

            cm::IkChain chain;
            chain.name = ik->chain;
            chain.bonename = node.bone;
            chain.link[0].kneeDir = ik->knee;
            chain.height = ik->height;
            chain.radius = ik->pad / 2.0f;
            chain.floor = ik->floor;
            chain.center = ik->center;
            ok = ApplyIkChain(c, line, std::move(chain));
            if (ok && ik->autoplayLock) {
                cm::IkLock lock;
                lock.name = ik->chain;
                lock.flPosWeight = ik->lockPosition;
                lock.flLocalQWeight = ik->lockRotation;
                ok = ApplyIkAutoplayLock(c, line, std::move(lock));
            }
        }
        if (!ok)
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Level of detail
// ---------------------------------------------------------------------------

// A render mesh by its own name (matched without case, as the document does), else a source file
// name or a fused choice by its name.
source::Source* ModelNamed(Ctx& c, const std::string& name) {
    for (const auto& [alias, source] : c.rendermeshes)
        if (_stricmp(alias.c_str(), name.c_str()) == 0)
            return source;
    return FindModelSource(c, name);
}

bool Lods(Load& l) {
    Ctx& c = l.c;
    const md::LodSettings& settings = l.d.lodSettings;
    if (settings.minLod) c.in.minLod = settings.minLod;
    if (settings.allowRootLods) c.in.allowRootLods = settings.allowRootLods;

    // The shadow LOD goes last wherever it sits in the list.
    std::vector<const md::LodNode*> order;
    for (const md::LodNode& node : l.d.lods)
        if (node.enabled && !node.shadow) order.push_back(&node);
    for (const md::LodNode& node : l.d.lods)
        if (node.enabled && node.shadow) { order.push_back(&node); break; }

    for (const md::LodNode* node : order) {
        const int line = l.Node(node->shadow ? "Shadow LOD" : "LOD: " + Quoted(node->name));
        cm::ScriptLod lod;
        lod.switchValue = node->shadow ? -1.0f : node->switchValue;
        lod.facialAnimation = !node->shadow && node->facial;
        lod.useShadowLodMaterials = node->shadow && node->shadowMaterials;
        lod.skeletalAwareDecimation = node->skeletalAware;
        if (!node->shadow && lod.switchValue < 0.0f)
            return c.Fail(line, "$lod: negative switch values are reserved for $shadowlod");

        for (const md::LodModelRule& rule : node->models) {
            cm::LodReplacement r;
            r.src = rule.target;
            r.srcSource = ModelNamed(c, rule.target);
            const char* verb = rule.action == md::LodAction::Replace ? "replacemodel"
                : rule.action == md::LodAction::Remove ? "removemodel" : "decimatemodel";
            if (!r.srcSource)
                return c.Fail(line, std::string(verb) + ": unknown model \"" + rule.target + "\"");

            if (rule.action == md::LodAction::Replace) {
                r.dst = rule.replacement;
                r.source = ModelNamed(c, rule.replacement);
                if (!r.source)
                    return c.Fail(line, "replacemodel: unknown replacement \"" + rule.replacement + "\"");
                lod.modelReplacements.push_back(std::move(r));
            } else if (rule.action == md::LodAction::Remove) {
                lod.modelReplacements.push_back(std::move(r));
            } else {
                r.decimation = rule.factor;
                if (r.decimation <= 0.0f || r.decimation > 1.0f)
                    return c.Fail(line, "decimatemodel: factor must be in (0, 1.0]");
                lod.generateLods.push_back(std::move(r));
            }
        }

        if (node->decimateAll) {
            lod.decimateAllFactor = node->decimateAllFactor;
            if (lod.decimateAllFactor <= 0.0f || lod.decimateAllFactor > 1.0f)
                return c.Fail(line, "decimateallmodel: factor must be in (0, 1.0]");
        }
        if (node->removeSmall) {
            lod.smallMeshLimit = node->smallSize;
            if (lod.smallMeshLimit <= 0.0f)
                return c.Fail(line, "removesmallmeshes: size must be positive");
            for (const std::string& name : node->smallMeshes) {
                source::Source* mesh = ModelNamed(c, name);
                if (!mesh)
                    return c.Fail(line, "removesmallmeshes: unknown mesh \"" + name + "\"");
                lod.smallMeshSources.push_back(mesh);
            }
        }

        for (const md::LodBoneRule& rule : node->boneReplacements) {
            if (!rule.complete()) continue;
            cm::LodReplacement r;
            r.src = rule.bone;
            r.dst = rule.replacement;
            lod.boneReplacements.push_back(std::move(r));
        }
        for (const std::string& bone : node->boneTreeCollapses) {
            cm::LodReplacement r;
            r.src = bone;
            lod.boneTreeCollapses.push_back(std::move(r));
        }
        lod.collapseJiggleBones = node->collapseJiggle;
        lod.collapseProceduralBones = node->collapseProcedural;

        for (const md::MaterialRule& rule : node->materialReplacements) {
            if (!rule.complete()) continue;
            cm::LodReplacement r;
            r.src = rule.from;
            r.dst = rule.to;
            lod.materialReplacements.push_back(std::move(r));
        }
        for (const md::MaterialRule& rule : node->materialSubstringReplacements) {
            if (!rule.complete()) continue;
            cm::LodReplacement r;
            r.src = rule.from;
            r.dst = rule.to;
            lod.materialSubstringReplacements.push_back(std::move(r));
        }
        for (const std::string& name : node->meshRemovals) {
            cm::LodReplacement r;
            r.src = name;
            lod.meshRemovals.push_back(std::move(r));
        }
        for (const std::string& name : node->meshSubstringRemovals) {
            cm::LodReplacement r;
            r.src = name;
            lod.meshSubstringRemovals.push_back(std::move(r));
        }

        if (!ApplyLod(c, line, std::move(lod)))
            return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Physics and prefabs
// ---------------------------------------------------------------------------

pm::Vector3 Rotated(const pm::Vector3& euler, const pm::Vector3& v) {
    // Euler angles in degrees as one quaternion (x, then y, then z), the way a capsule is spun.
    const float hx = euler.x * pm::kDeg2Rad * 0.5f, hy = euler.y * pm::kDeg2Rad * 0.5f,
                hz = euler.z * pm::kDeg2Rad * 0.5f;
    const float cx = std::cos(hx), sx = std::sin(hx), cy = std::cos(hy), sy = std::sin(hy),
                cz = std::cos(hz), sz = std::sin(hz);
    const float w = cx * cy * cz + sx * sy * sz, x = sx * cy * cz - cx * sy * sz,
                y = cx * sy * cz + sx * cy * sz, z = cx * cy * sz - sx * sy * cz;
    const pm::Vector3 u{x, y, z};
    const pm::Vector3 t{2 * (u.y * v.z - u.z * v.y), 2 * (u.z * v.x - u.x * v.z), 2 * (u.x * v.y - u.y * v.x)};
    return {v.x + w * t.x + (u.y * t.z - u.z * t.y), v.y + w * t.y + (u.z * t.x - u.x * t.z),
            v.z + w * t.z + (u.x * t.y - u.y * t.x)};
}

const char* ImportTypeWord(cm::PhysicsImportType type) {
    return type == cm::PhysicsImportType::ToOneBone ? "singlejoint"
         : type == cm::PhysicsImportType::OneBoneOnly ? "singlehull" : "perjoint";
}

// A render mesh by its own name, as the script spells it.
std::string MeshAlias(Ctx& c, const std::string& name) {
    for (const auto& entry : c.rendermeshes)
        if (_stricmp(entry.first.c_str(), name.c_str()) == 0)
            return entry.first;
    return name;
}

bool PhysicsShapeOf(Load& l, int line, const md::PhysicsShape& shape) {
    Ctx& c = l.c;
    using Type = md::PhysicsShapeType;
    cm::PhysicsShape sh;
    sh.parentBone = shape.bone;

    if (shape.type == Type::Box || shape.type == Type::Sphere || shape.type == Type::Capsule) {
        sh.kind = cm::PhysicsShapeKind::Primitive;
        sh.name = shape.bone;
        const std::string where = "$physicsshape primitive";
        if (shape.type == Type::Box) {
            if (shape.min.x == shape.max.x || shape.min.y == shape.max.y || shape.min.z == shape.max.z)
                return c.Fail(line, where + " box: box is flat");
            source::TessellateBox(shape.min, shape.max, shape.angles, sh.primitivePoints);
        } else if (shape.type == Type::Sphere) {
            if (!(shape.radius > 0.0f))
                return c.Fail(line, where + " sphere: radius must be greater than 0");
            source::TessellateSphere(shape.center, shape.radius, shape.segments, sh.primitivePoints);
        } else {
            if (!(shape.radius > 0.0f) && !(shape.endRadius > 0.0f))
                return c.Fail(line, where + " capsule: needs a radius greater than 0");
            // The segment is spun about its midpoint.
            const pm::Vector3 center{(shape.min.x + shape.max.x) * 0.5f, (shape.min.y + shape.max.y) * 0.5f,
                                     (shape.min.z + shape.max.z) * 0.5f};
            const auto end = [&](const pm::Vector3& p) {
                const pm::Vector3 r = Rotated(shape.angles, {p.x - center.x, p.y - center.y, p.z - center.z});
                return pm::Vector3{center.x + r.x, center.y + r.y, center.z + r.z};
            };
            source::TessellateCapsule(end(shape.min), end(shape.max), shape.radius, shape.endRadius,
                                      shape.segments, sh.primitivePoints);
        }
        for (pm::Vector3& p : sh.primitivePoints)
            p = {p.x * c.in.scale, p.y * c.in.scale, p.z * c.in.scale};
        return ApplyPhysShapeAdd(c, line, std::move(sh));
    }

    sh.offsetOrigin = shape.offsetOrigin;
    sh.offsetAngles = shape.offsetAngles;
    sh.offsetAnglesSet = shape.offsetAngles.x != 0 || shape.offsetAngles.y != 0 || shape.offsetAngles.z != 0;
    sh.importScale = shape.importScale;
    sh.maxConvex = shape.maxConvex;

    std::string mode, ref;
    if (shape.type == Type::FromFile) {
        const md::SourceFile* file = l.d.findSource(shape.source);
        if (!file) return c.Fail(line, "$physicsshape fromfile: the shape has no source");
        mode = "fromfile";
        ref = VirtualName(*file);
        sh.kind = cm::PhysicsShapeKind::FromFile;
        sh.importType = shape.importType;
        sh.concave = shape.concave;
        sh.remove2d = shape.remove2d;
    } else {
        mode = shape.type == Type::FromRenderMesh ? "fromrendermesh" : "fromrender";
        ref = shape.type == Type::FromRenderMesh ? MeshAlias(c, shape.renderMesh) : std::string();
        sh.kind = cm::PhysicsShapeKind::FromRender;
        sh.decimationFactor = shape.decimationFactor;
        sh.concavity = shape.concavity;
        sh.maxHulls = shape.maxHulls;
        sh.maxDepth = shape.maxDepth;
        sh.cullWeight = shape.cullWeight;
        sh.extraSkinnedBones = shape.extraBones;
        if (shape.type == Type::FromRender)
            for (const std::string& name : shape.excludedMeshes) sh.exceptionMeshNames.push_back(MeshAlias(c, name));
        if (shape.maxHulls < 0 || shape.maxHulls > limits::kMaxConvexPieces)
            return c.Fail(line, "$physicsshape " + mode + ": maxhulls must be between 0 and " +
                                    std::to_string(limits::kMaxConvexPieces));
        if (shape.maxDepth < 0 || shape.maxDepth > 10)
            return c.Fail(line, "$physicsshape " + mode + ": maxdepth must be between 0 and 10");
    }
    if (!ApplyPhysShape(c, line, mode, ref, ImportTypeWord(sh.importType), sh))
        return false;
    return ApplyPhysShapeAdd(c, line, std::move(sh));
}

bool Physics(Load& l) {
    Ctx& c = l.c;
    const md::PhysicsSettings& s = l.d.physics;
    const bool any = std::ranges::any_of(l.d.physicsNodes, [](const md::PhysicsNode& node) { return node.enabled; });
    if (!any) return true;

    c.in.physMass = s.mass;
    c.in.physAutoMass = s.autoMass;
    c.in.physMassCenter = s.massCenter;
    c.in.physMassCenterSet = s.massCenterSet;
    c.in.physRootBone = s.rootBone;
    c.in.physNoSelfCollisions = s.noSelfCollisions;
    c.in.physAssumeWorldspace = s.assumeWorldSpace;
    c.in.physDamping = s.damping;
    c.in.physRotdamping = s.rotationDamping;
    c.in.physInertia = s.inertia;
    c.in.physDrag = s.drag;
    c.in.physWeldPosition = s.weldPosition;
    c.in.physWeldNormal = s.weldNormal;
    c.in.physName = s.fileName;
    if (s.animatedFriction) {
        c.in.physHasAnimatedFriction = true;
        c.in.physAnimFrictionMin = s.frictionMin;
        c.in.physAnimFrictionMax = s.frictionMax;
        c.in.physAnimFrictionTimeIn = s.frictionIn;
        c.in.physAnimFrictionTimeOut = s.frictionOut;
        c.in.physAnimFrictionTimeHold = s.frictionHold;
    }

    const size_t shapesBefore = c.in.physShapes.size();
    int firstLine = 0;
    for (const md::PhysicsNode& node : l.d.physicsNodes) {
        if (!node.enabled) continue;

        const int line = l.Node("Physics: " + Quoted(node.name));
        if (!firstLine) firstLine = line;

        if (const auto* shape = std::get_if<md::PhysicsShape>(&node.value)) {
            if (!PhysicsShapeOf(l, line, *shape))
                return false;
        } else if (const auto* joint = std::get_if<md::PhysicsJoint>(&node.value)) {
            if (joint->bone.empty()) continue;

            cm::PhysicsJoint* target = nullptr;
            for (auto& entry : c.in.physJoints)
                if (_stricmp(entry.bonename.c_str(), joint->bone.c_str()) == 0) { target = &entry; break; }
            if (!target) {
                c.in.physJoints.emplace_back();
                target = &c.in.physJoints.back();
                target->bonename = joint->bone;
            }
            for (int axis = 0; axis < 3; ++axis) {
                const md::PhysicsAxis& a = joint->axes[static_cast<size_t>(axis)];
                if (a.mode == md::PhysicsAxisMode::Unset) continue;
                for (const cm::PhysicsJointAxis& e : target->axes)
                    if (e.axis == axis)
                        return c.Fail(line, "$physicsjoint \"" + joint->bone + "\": axis " +
                                                std::string(1, "xyz"[axis]) + " is written more than once");

                cm::PhysicsJointAxis out;
                out.axis = axis;
                out.type = a.mode == md::PhysicsAxisMode::Free ? 0 : a.mode == md::PhysicsAxisMode::Limit ? 1 : 2;
                if (out.type == 1) {
                    out.min = a.min;
                    out.max = a.max;
                }
                out.friction = a.friction;
                target->axes.push_back(out);
            }
        } else if (const auto* body = std::get_if<md::PhysicsBody>(&node.value)) {
            if (body->bone.empty()) continue;

            for (const cm::PhysicsMarkup& e : c.in.physMarkups)
                if (_stricmp(e.bonename.c_str(), body->bone.c_str()) == 0)
                    return c.Fail(line, "$physicsmarkup \"" + body->bone + "\": two nodes name this body");

            cm::PhysicsMarkup mk;
            mk.name = mk.bonename = body->bone;
            if (body->mode == md::PhysicsBodyMode::Skip) mk.skip = true;
            if (body->mode == md::PhysicsBodyMode::Merge) mk.mergeInto = body->mergeInto;
            if (body->massMode == md::PhysicsMassMode::Fixed) {
                if (body->mass <= 0.0f)
                    return c.Fail(line, "$physicsmarkup \"" + body->bone + "\": mass must be greater than 0");
                mk.mass = body->mass;
                mk.massSet = true;
            } else if (body->massMode == md::PhysicsMassMode::Bias) {
                mk.massBias = body->massBias;
                mk.massBiasSet = true;
            }
            if (body->inertiaSet) { mk.inertia = body->inertia; mk.inertiaSet = true; }
            if (body->dampingSet) { mk.damping = body->damping; mk.dampingSet = true; }
            if (body->rotationDampingSet) { mk.rotdamping = body->rotationDamping; mk.rotdampingSet = true; }
            c.in.physMarkups.push_back(std::move(mk));
        } else if (const auto* pair = std::get_if<md::PhysicsPair>(&node.value)) {
            if (pair->first.empty() || pair->second.empty()) continue;

            if (pair->collide) {
                cm::PhysicsCollidePair entry;
                entry.a = pair->first;
                entry.b = pair->second;
                c.in.physCollidePairs.push_back(std::move(entry));
                continue;
            }
            auto& pairs = c.in.physCollidePairs;
            const size_t before = pairs.size();
            std::erase_if(pairs, [&](const cm::PhysicsCollidePair& p) {
                return (_stricmp(p.a.c_str(), pair->first.c_str()) == 0 && _stricmp(p.b.c_str(), pair->second.c_str()) == 0) ||
                       (_stricmp(p.a.c_str(), pair->second.c_str()) == 0 && _stricmp(p.b.c_str(), pair->first.c_str()) == 0);
            });
            if (pairs.size() == before)
                l.Warn(line, "no collide pair of \"" + pair->first + "\" and \"" + pair->second + "\" above it to remove");
        }
    }
    if (c.in.physShapes.size() == shapesBefore)
        c.shapelessPhysicsModelLine = firstLine;
    return true;
}

// After every authored node: an authored name beats a prefab's, and any authored hitbox set blocks
// the imported ones.
bool Prefabs(Load& l) {
    Ctx& c = l.c;
    for (const md::PrefabNode& node : l.d.prefabs) {
        if (!node.imports()) continue;

        const md::SourceFile* file = l.d.findSource(node.source);
        if (!file) continue;

        const int line = l.Node("Prefab: " + Quoted(node.name));
        if (node.kind == md::PrefabKind::Joints) {
            if (!node.jigglebones && !node.proceduralbones && !node.hitboxes && !node.attachments)
                continue;
            HitboxImport hitboxes;
            for (const md::HitboxSetRename& rename : node.hitboxSetRenames)
                if (rename.enabled && rename.complete()) hitboxes.renames.emplace_back(rename.from, rename.to);
            if (!ApplyDataModelJoints(c, line, VirtualName(*file), node.jigglebones, node.proceduralbones,
                                      node.hitboxes, node.attachments, &hitboxes))
                return false;
        } else if (!ApplyDataModelPhysics(c, line, VirtualName(*file))) {
            return false;
        }
    }
    return true;
}
// ---------------------------------------------------------------------------
// Flex graph, wrinkle scales and morph splits
// ---------------------------------------------------------------------------

// A wrinkle or split that names no morph of the model is ignored with a warning, as the Model
// Assembler ignores it; the graph itself registers once the bodies' morph descs exist.
bool Flex(Load& l) {
    Ctx& c = l.c;
    const md::Document& d = l.d;
    if (d.flexNodes.empty() && d.flexWrinkles.empty() && d.flexSplits.empty() && d.flexLocalVars.empty())
        return true;

    std::map<std::string, std::string> morphs;
    for (const auto& entry : c.rendermeshes)
        for (const source::SrcFlexKey& key : entry.second->flexkeys) {
            std::string lower = key.name;
            std::ranges::transform(lower, lower.begin(), [](unsigned char ch) { return char(std::tolower(ch)); });
            morphs.emplace(std::move(lower), key.name);
        }
    const auto spelled = [&](const std::string& name) -> const std::string* {
        std::string lower = name;
        std::ranges::transform(lower, lower.begin(), [](unsigned char ch) { return char(std::tolower(ch)); });
        const auto found = morphs.find(lower);
        return found == morphs.end() ? nullptr : &found->second;
    };

    for (const md::FlexWrinkle& wrinkle : d.flexWrinkles) {
        const std::string* morph = spelled(wrinkle.morph);
        if (!morph) {
            l.Warn(l.Node("Wrinkle Scale: " + Quoted(wrinkle.morph)), "names no morph of the model and is ignored");
            continue;
        }
        source::WrinkleScaleOption option;
        option.line = l.Node("Wrinkle Scale: " + Quoted(wrinkle.morph));
        option.shape = *morph;
        option.scale = wrinkle.scale;
        c.wrinkleScales.push_back(std::move(option));
    }
    for (const md::FlexSplit& split : d.flexSplits) {
        const std::string* morph = spelled(split.morph);
        if (!morph) {
            l.Warn(l.Node("Morph Split: " + Quoted(split.morph)), "names no morph of the model and is ignored");
            continue;
        }
        const bool listed = std::ranges::any_of(c.manual.stereoSplits, [&](const auto& entry) {
            return _stricmp(entry.name.c_str(), morph->c_str()) == 0;
        });
        if (!listed) c.manual.stereoSplits.push_back({*morph, split.factor});
    }

    const md::Document* document = &d;
    c.manual.custom = [document](cm::CompileInput& in, std::string* err) {
        return md::RegisterFlexGraph(*document, in, err);
    };
    return true;
}


// ---------------------------------------------------------------------------
// Animation Graph
// ---------------------------------------------------------------------------

// The name a diagnostic gives a graph node.
std::string AnimNodeLabel(const md::Document& d, md::Uid uid) {
    const md::AnimNode* node = d.findAnimNode(uid);
    if (!node || node->name.empty())
        return "Animation Graph";
    return "Animation Graph " + Quoted(node->name);
}

bool Animations(Load& l) {
    Ctx& c = l.c;
    if (l.d.animNodes.empty() && l.d.poseParameters.empty())
        return true;

    const md::AnimScript script = md::emitAnimScript(l.d);
    if (!script.problems.empty()) {
        const md::Problem& first = script.problems.front();
        return c.Fail(l.Node(AnimNodeLabel(l.d, first.node)), first.message);
    }

    for (const cm::WeightList& list : script.weightLists)
        if (!ApplyWeightList(c, l.Node("Weight List " + Quoted(list.name)), list))
            return false;
    for (const cm::PoseParam& param : script.poseParams)
        if (!ApplyPoseParameter(c, l.Node("Pose Parameter " + Quoted(param.name)), param))
            return false;
    c.in.xnodes = script.xnodes;

    // Animations first, so a sequence can name them; `at` maps an emitted animation to its place in the list.
    std::vector<int> at(script.anims.size(), -1);
    for (size_t i = 0; i < script.anims.size(); ++i) {
        cm::CompileInput::InAnim anim = script.anims[i].anim;
        const int node = l.Node(AnimNodeLabel(l.d, script.anims[i].node));

        if (script.anims[i].source) {
            const md::SourceFile* source = l.d.findSource(script.anims[i].source);
            if (!source)
                return c.Fail(node, "the animation source is missing");
            anim.source = LoadAnimationSource(c, node, VirtualName(*source));
            if (!anim.source)
                return false;
        }

        if (!anim.name.empty() && anim.name[0] == '@') {
            at[i] = AddImpliedAnimation(c, std::move(anim));
        } else {
            const std::string name = anim.name;
            if (!ApplyAnimation(c, node, std::move(anim)))
                return false;
            at[i] = c.namedAnims[name];
        }
    }

    for (const md::AnimScriptInclude& include : script.includeModels)
        if (!ApplyIncludeModel(c, l.Node(AnimNodeLabel(l.d, include.node)), "Include Model", include.path))
            return false;

    for (const md::AnimScriptSequence& entry : script.sequences) {
        cm::CompileInput::InSequence seq = entry.sequence;
        if (seq.isDeclare) {
            if (seq.name.empty())
                return c.Fail(l.Node(AnimNodeLabel(l.d, entry.node)), "the declared sequence has no name");
            ApplyDeclareSequence(c, std::move(seq.name));
            continue;
        }

        std::vector<int> blends;
        if (seq.blendAnims.empty())
            blends.push_back(at[static_cast<size_t>(seq.animIndex)]);
        for (const int index : seq.blendAnims)
            blends.push_back(at[static_cast<size_t>(index)]);
        seq.blendAnims.clear();

        if (!ApplySequence(c, l.Node(AnimNodeLabel(l.d, entry.node)), std::move(seq), std::move(blends), /*inherit=*/false))
            return false;
    }
    return true;
}

// The passes, in the order the commands need.
constexpr Pass kPasses[] = {
    Definition,
    MaterialCollapses,
    MaterialDirectories,
    BoneDefinitions,
    RealignBones,
    NoRealign,
    Hierarchies,
    Meshes,
    Groups,
    Lods,
    MaterialRenames,
    Face,
    Attachments,
    Hitboxes,
    BoneMarkups,
    Animations,
    Physics,
    Prefabs,
    Flex,
    RenderDefinition,
};

} // namespace

bool IsMdlAsmPath(const char* path) {
    return _stricmp(pulse::FilePath(path).extension().string().c_str(), ".mdlasm") == 0;
}

bool LoadMdlAsm(const char* rawPath, compile::CompileInput& out, std::string* err) {
    const std::filesystem::path path = std::filesystem::absolute(pulse::FilePath(rawPath)).lexically_normal();

    md::Document document;
    try {
        document = md::LoadDocument(path.string());
    } catch (const std::exception& error) {
        if (err) *err = path.filename().string() + ": " + error.what();
        return false;
    }
    for (const std::string& tag : document.unknownChunks)
        pulse::diag::WarnAt({}, "the model assembly has a section \"" + tag + "\" this compiler does not know; it was ignored");

    pulse::dependencies::Note(path.string(), pulse::dependencies::Kind::Script);

    Ctx c{out};
    c.err = err;
    c.file = path.filename().string();
    c.rootFile = c.file;
    c.scriptDir = path.parent_path();
    c.curDir = c.scriptDir;
    c.sourceDirStack.push_back(c.scriptDir);
    c.nodeLabels.push_back(std::string());
    for (const md::SourceFile& source : document.sources)
        if (source.bytes) c.memorySources[VirtualName(source)] = source.bytes;
    out.mdlPath = rawPath;

    Load load{c, document};
    for (const Pass pass : kPasses) {
        const bool ok = pass(load);
        pulse::diag::Current() = {};
        if (!ok)
            return false;
    }
    return FinishScript(c, out, err);
}

} // namespace pulse::loader
