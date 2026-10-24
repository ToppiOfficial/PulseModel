// pulsemodel.cpp - .pulsemodel (JSON) front end. See pulsemodel.h.
//
// Load order: parse + validate each file against the schema, substitute
// variables, expand prefabs, merge `shared` and prefab content into the chosen
// model, then lower the result to script tokens in a fixed section order.

#include "strcompat.h"
#include "pathcompat.h"

#include "pulsemodel.h"
#include "api.h"
#include "dependencies.h"
#include "diagnostic.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace pulse::pulsemodel {

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;

namespace {

constexpr int kFormatVersion = 1;
// Added after validation, so no file can write them: an object's [file index,
// JSON pointer], and per key the same for keys merged in from another file.
constexpr const char* kOrigin = "@origin";
constexpr const char* kKeyOrigins = "@keys";

bool IsInternal(const std::string& key) { return !key.empty() && key[0] == '@'; }

// ---------------------------------------------------------------------------
// Schema - every key a .pulsemodel may hold. "editor" is allowed in any object
// and is never read by the compiler.
// ---------------------------------------------------------------------------

enum class Type {
    String, Number, Integer, Bool, Vec3, Enum, File,
    Strings, Files, Variables, Prefabs, Object, List, Any
};

struct Item;

struct Field {
    const char* key;
    Type type;
    const char* doc;
    const Item* item = nullptr;
    const char* const* values = nullptr; // Enum, null-terminated
    const char* ref = nullptr;           // what a name refers to: bone, mesh, ...
    bool required = false;

    constexpr Field Of(const Item* i) const { Field f = *this; f.item = i; return f; }
    constexpr Field Values(const char* const* v) const { Field f = *this; f.values = v; return f; }
    constexpr Field Ref(const char* r) const { Field f = *this; f.ref = r; return f; }
    constexpr Field Required() const { Field f = *this; f.required = true; return f; }
};

struct Item {
    const char* name;
    const Field* fields;
    size_t count;
    const Item* base = nullptr; // fields shared with another item
};

constexpr Field F(const char* key, Type type, const char* doc) { return {key, type, doc}; }

template <size_t N>
constexpr Item MakeItem(const char* name, const Field (&fields)[N], const Item* base = nullptr) {
    return {name, fields, N, base};
}

const char* const kArchetypes[] = {"general", "character", "static", "simple", nullptr};
const char* const kRenderPasses[] = {"none", "opaque", "mostlyopaque", nullptr};
const char* const kUpAxes[] = {"x", "-x", "y", "-y", "z", "-z", nullptr};
const char* const kBoneCulls[] = {"aggressive", "leafonly", "none", nullptr};
const char* const kSkinnedBoneCulls[] = {"aggressive", "tree", nullptr};
const char* const kAxes[] = {"x", "y", "z", nullptr};

// --- model settings ---

const Field kTransform[] = {
    F("origin", Type::Vec3, "Model translation."),
    F("angles", Type::Vec3, "Model rotation in degrees (pitch yaw roll)."),
    F("scale", Type::Number, "Uniform model scale."),
    F("upAxis", Type::Enum, "Up axis of the source art; replaces angles.").Values(kUpAxes),
};
const Item kTransformItem = MakeItem("transform", kTransform);

const Field kBudget[] = {
    F("bones", Type::Integer, "Bone ceiling."),
    F("materials", Type::Integer, "Material ceiling."),
};
const Item kBudgetItem = MakeItem("budget", kBudget);

const Field kBox[] = {
    F("min", Type::Vec3, "Minimum corner.").Required(),
    F("max", Type::Vec3, "Maximum corner.").Required(),
};
const Item kBoxItem = MakeItem("box", kBox);

const Field kBBox[] = {
    F("min", Type::Vec3, "Minimum corner."),
    F("max", Type::Vec3, "Maximum corner."),
    F("meshes", Type::Strings, "Fit the box to these meshes instead.").Ref("mesh"),
};
const Item kBBoxItem = MakeItem("bbox", kBBox);

const Field kIllumPosition[] = {
    F("position", Type::Vec3, "Lighting sample point.").Required(),
    F("bone", Type::String, "Bone the point follows; omitted = static.").Ref("bone"),
};
const Item kIllumPositionItem = MakeItem("illumPosition", kIllumPosition);

const Field kEyePosition[] = {
    F("position", Type::Vec3, "Ideal eye point.").Required(),
    F("autoHeight", Type::Bool, "Z comes from the eyeballs; position.z is an offset."),
};
const Item kEyePositionItem = MakeItem("eyePosition", kEyePosition);

const Field kSettings[] = {
    F("name", Type::String, "Model path under models/, e.g. player/foo."),
    F("archetype", Type::Enum, "Model archetype.").Values(kArchetypes),
    F("transform", Type::Object, "Whole-model transform.").Of(&kTransformItem),
    F("renderPass", Type::Enum, "Render pass.").Values(kRenderPasses),
    F("budget", Type::Object, "Compile ceilings.").Of(&kBudgetItem),
    F("surfaceProp", Type::String, "Surface property."),
    F("contents", Type::Strings, "Trace contents: solid, grate, ladder, monster, debris, notsolid or a number."),
    F("bbox", Type::Object, "Hull box.").Of(&kBBoxItem),
    F("cbox", Type::Object, "View clipping box.").Of(&kBoxItem),
    F("illumPosition", Type::Object, "Lighting sample point.").Of(&kIllumPositionItem),
    F("eyePosition", Type::Object, "Ideal eye point.").Of(&kEyePositionItem),
    F("maxEyeDeflection", Type::Number, "Eye turn limit in degrees."),
    F("vtxFormat", Type::Integer, ".vtx layout: 0 legacy (TF2/L4D2/GMod/HL2), 1 full (SFM/CS:GO)."),
    F("noDx80", Type::Bool, "Skip the .dx80.vtx."),
    F("autoCenter", Type::Bool, "Center the model."),
    F("forceWriteVertexData", Type::Bool, "Write .vvd/.vtx even without geometry."),
    F("ambientBoost", Type::Bool, "Ambient boost flag."),
    F("doNotCastShadows", Type::Bool, "Cast no shadows."),
    F("forcePhonemeCrossfade", Type::Bool, "Force phoneme crossfade."),
    F("noForcedFade", Type::Bool, "No forced fade."),
    F("castTextureShadows", Type::Bool, "Cast texture shadows."),
    F("constantDirectionalLight", Type::Number, "Constant directional light scale (0-1)."),
    F("skipBoneInBBox", Type::Bool, "Auto hitboxes grow from vertices only."),
    F("allowRootLods", Type::Integer, "Allowed root LOD count."),
    F("minLod", Type::Integer, "Promote this LOD to root."),
};
const Item kSettingsItem = MakeItem("model", kSettings);

// --- materials ---

const Field kReplace[] = {
    F("material", Type::String, "Material to replace.").Ref("material").Required(),
    F("with", Type::String, "Replacement material.").Required(),
};
const Item kReplaceItem = MakeItem("replace", kReplace);

const Field kSkin[] = {
    F("replace", Type::List, "Materials this skin swaps.").Of(&kReplaceItem),
};
const Item kSkinItem = MakeItem("skin", kSkin);

const Field kMaterials[] = {
    F("paths", Type::Strings, "Material search paths ($cdmaterials)."),
    F("skins", Type::List, "Skin families after the base skin.").Of(&kSkinItem),
    F("renames", Type::List, "Rename a loaded material.").Of(&kReplaceItem),
    F("collapse", Type::List, "Faces of a material take another one as they load.").Of(&kReplaceItem),
    F("sortOrder", Type::Strings, "Mesh draw order by material.").Ref("material"),
};
const Item kMaterialsItem = MakeItem("materials", kMaterials);

// --- meshes and model groups ---

const Field kWeld[] = {
    F("seams", Type::Bool, "Also weld across UV seams."),
    F("sharp", Type::Number, "Keep edges sharper than this many degrees."),
};
const Item kWeldItem = MakeItem("weld", kWeld);

const Field kScaleBone[] = {
    F("bone", Type::String, "Bone to scale.").Ref("bone").Required(),
    F("factor", Type::Number, "Scale factor.").Required(),
    F("axes", Type::Strings, "Axes to scale (x, y, z); omitted = all."),
};
const Item kScaleBoneItem = MakeItem("scaleBone", kScaleBone);

const Field kMesh[] = {
    F("name", Type::String, "Mesh name model groups refer to.").Required(),
    F("file", Type::File, "Source file (.dmx, .smd, .fbx).").Required(),
    F("noMorph", Type::Bool, "Load without morphs."),
    F("flipNormals", Type::Bool, "Flip normals."),
    F("removeMeshes", Type::Strings, "Drop meshes by material.").Ref("material"),
    F("removeMeshWords", Type::Strings, "Drop meshes whose material contains a word."),
    F("weld", Type::Object, "Weld vertices.").Of(&kWeldItem),
    F("decimate", Type::Number, "Keep this share of faces (0-1]."),
    F("skinnedBoneCull", Type::Enum, "Drop unweighted bones.").Values(kSkinnedBoneCulls),
    F("scaleBones", Type::List, "Scale bones as the mesh loads.").Of(&kScaleBoneItem),
};
const Item kMeshItem = MakeItem("mesh", kMesh);

const Field kChoice[] = {
    F("name", Type::String, "Choice name; omitted = named after its meshes."),
    F("meshes", Type::Strings, "Meshes drawn together by this choice.").Ref("mesh"),
    F("blank", Type::Bool, "An empty choice."),
};
const Item kChoiceItem = MakeItem("choice", kChoice);

const Field kModelGroup[] = {
    F("name", Type::String, "Group name.").Required(),
    F("choices", Type::List, "Choices, in index order.").Of(&kChoiceItem).Required(),
};
const Item kModelGroupItem = MakeItem("modelGroup", kModelGroup);

// --- LODs ---

const Field kLodModel[] = {
    F("model", Type::String, "Mesh this LOD replaces.").Ref("mesh").Required(),
    F("mesh", Type::String, "Replacement mesh.").Ref("mesh"),
    F("file", Type::File, "Replacement source file."),
};
const Item kLodModelItem = MakeItem("lodModel", kLodModel);

const Field kLodDecimate[] = {
    F("model", Type::String, "Mesh to decimate.").Ref("mesh").Required(),
    F("factor", Type::Number, "Share of faces kept (0-1].").Required(),
};
const Item kLodDecimateItem = MakeItem("lodDecimate", kLodDecimate);

const Field kSmallMeshes[] = {
    F("size", Type::Number, "Minimum unit size.").Required(),
    F("meshes", Type::Strings, "Limit to these meshes.").Ref("mesh"),
};
const Item kSmallMeshesItem = MakeItem("removeSmallMeshes", kSmallMeshes);

const Field kLodBone[] = {
    F("bone", Type::String, "Bone to replace.").Ref("bone").Required(),
    F("with", Type::String, "Replacement bone.").Ref("bone").Required(),
};
const Item kLodBoneItem = MakeItem("lodBone", kLodBone);

const Field kLodWord[] = {
    F("word", Type::String, "Word to match in material names.").Required(),
    F("with", Type::String, "Replacement material.").Required(),
};
const Item kLodWordItem = MakeItem("lodWord", kLodWord);

const Field kLod[] = {
    F("switch", Type::Number, "Switch value; omit for the shadow LOD."),
    F("shadow", Type::Bool, "The shadow LOD (last, at most one)."),
    F("replaceModels", Type::List, "Draw other geometry.").Of(&kLodModelItem),
    F("removeModels", Type::Strings, "Stop drawing meshes.").Ref("mesh"),
    F("decimateModels", Type::List, "Generate geometry by decimation.").Of(&kLodDecimateItem),
    F("decimateAll", Type::Number, "Decimate every mesh to this share."),
    F("removeSmallMeshes", Type::Object, "Drop small meshes.").Of(&kSmallMeshesItem),
    F("replaceBones", Type::List, "Bone replacements.").Of(&kLodBoneItem),
    F("collapseBoneTrees", Type::Strings, "Collapse whole bone subtrees.").Ref("bone"),
    F("collapseJiggleBones", Type::Bool, "Collapse jiggle bones."),
    F("collapseProceduralBones", Type::Bool, "Collapse procedural bones."),
    F("replaceMaterials", Type::List, "Material replacements.").Of(&kReplaceItem),
    F("replaceMaterialWords", Type::List, "Material replacements by word.").Of(&kLodWordItem),
    F("removeMeshes", Type::Strings, "Drop meshes by material.").Ref("material"),
    F("removeMeshWords", Type::Strings, "Drop meshes whose material contains a word."),
    F("facial", Type::Bool, "Facial animation on this LOD."),
    F("skeletalAwareDecimation", Type::Bool, "Decimate with the skeleton in mind."),
    F("useShadowLodMaterials", Type::Bool, "Shadow LOD keeps its own materials."),
};
const Item kLodItem = MakeItem("lod", kLod);

// --- bones ---

const Field kPose[] = {
    F("position", Type::Vec3, "Position.").Required(),
    F("angles", Type::Vec3, "Rotation in degrees.").Required(),
};
const Item kPoseItem = MakeItem("pose", kPose);

const Field kDefineBone[] = {
    F("name", Type::String, "Bone name.").Required(),
    F("parent", Type::String, "Parent bone; omitted = root bone.").Ref("bone"),
    F("unlocked", Type::Bool, "Appended after the source bones instead of replacing one."),
    F("position", Type::Vec3, "Bind position.").Required(),
    F("angles", Type::Vec3, "Bind rotation in degrees.").Required(),
    F("realign", Type::Object, "Realign transform; marks the bone pre-aligned.").Of(&kPoseItem),
};
const Item kDefineBoneItem = MakeItem("defineBone", kDefineBone);

const Field kRename[] = {
    F("bone", Type::String, "Bone to rename.").Ref("bone").Required(),
    F("to", Type::String, "New name.").Required(),
};
const Item kRenameItem = MakeItem("renameBone", kRename);

const Field kTransformWeights[] = {
    F("residualBone", Type::String, "Bone the weights ramp onto.").Ref("bone").Required(),
    F("factor", Type::Number, "Ramp band width (0-1)."),
    F("smoothing", Type::Number, "S-curve easing (0-1)."),
};
const Item kTransformWeightsItem = MakeItem("transformWeights", kTransformWeights);

const Field kTransformBone[] = {
    F("bone", Type::String, "Bone to edit.").Ref("bone").Required(),
    F("angles", Type::Vec3, "Rotate the bind orientation."),
    F("position", Type::Vec3, "Translate the bind position."),
    F("worldAngles", Type::Bool, "Angles are in world axes."),
    F("worldPosition", Type::Bool, "Position is in world axes."),
    F("transformWeights", Type::Object, "Re-skin ramp.").Of(&kTransformWeightsItem),
    F("offset", Type::Vec3, "Shift the transformWeights ramp end."),
    F("transformVerts", Type::Bool, "Carry the rigged vertices."),
    F("transformChildren", Type::Bool, "Move the whole subtree."),
    F("ignoreAnimation", Type::Bool, "Never apply to animation."),
    F("ignoreHitbox", Type::Bool, "Leave this bone's hitbox in place."),
};
const Item kTransformBoneItem = MakeItem("transformBone", kTransformBone);

const Field kBones[] = {
    F("define", Type::List, "Bind-pose bones.").Of(&kDefineBoneItem),
    F("boneMerge", Type::Strings, "Bones kept for bonemerge.").Ref("bone"),
    F("doNotCollapse", Type::Strings, "Bones never collapsed.").Ref("bone"),
    F("alwaysCollapse", Type::Strings, "Bones always collapsed.").Ref("bone"),
    F("renames", Type::List, "Rename bones in the output.").Of(&kRenameItem),
    F("transforms", Type::List, "Bind-pose edits.").Of(&kTransformBoneItem),
    F("root", Type::String, "Motion root bone.").Ref("bone"),
    F("cullMethod", Type::Enum, "Bone cull method.").Values(kBoneCulls),
    F("realign", Type::Bool, "Realign single-child chains."),
    F("lockLengths", Type::Bool, "Pin bone lengths to the bind pose."),
};
const Item kBonesItem = MakeItem("bones", kBones);

// --- attachments, hitboxes, pose parameters, IK ---

const Field kAttachment[] = {
    F("name", Type::String, "Attachment name.").Required(),
    F("bone", Type::String, "Parent bone; omitted = model space.").Ref("bone"),
    F("position", Type::Vec3, "Position."),
    F("angles", Type::Vec3, "Rotation in degrees."),
    F("rigid", Type::Bool, "Rigid."),
    F("absolute", Type::Bool, "Position is in model space."),
    F("worldAlign", Type::Bool, "World aligned."),
    F("flexGroups", Type::Strings, "Place at the average of these flex groups."),
    F("flexMorphs", Type::Strings, "Place at the average of these morphs."),
    F("materials", Type::Strings, "Place at the average of these materials.").Ref("material"),
};
const Item kAttachmentItem = MakeItem("attachment", kAttachment);

const Field kHitbox[] = {
    F("group", Type::Integer, "Hit group.").Required(),
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("min", Type::Vec3, "Minimum corner.").Required(),
    F("max", Type::Vec3, "Maximum corner.").Required(),
    F("angles", Type::Vec3, "Rotation in degrees."),
    F("radius", Type::Number, "Capsule radius."),
    F("name", Type::String, "Hitbox name."),
};
const Item kHitboxItem = MakeItem("hitbox", kHitbox);

const Field kHitboxSet[] = {
    F("name", Type::String, "Set name.").Required(),
    F("boxes", Type::List, "Hitboxes.").Of(&kHitboxItem),
};
const Item kHitboxSetItem = MakeItem("hitboxSet", kHitboxSet);

const Field kPoseParameter[] = {
    F("name", Type::String, "Parameter name.").Required(),
    F("min", Type::Number, "Start value.").Required(),
    F("max", Type::Number, "End value.").Required(),
    F("wrap", Type::Bool, "Wraps over the full range."),
    F("loop", Type::Number, "Loops at this value."),
};
const Item kPoseParameterItem = MakeItem("poseParameter", kPoseParameter);

const Field kIkChain[] = {
    F("name", Type::String, "Chain name.").Required(),
    F("bone", Type::String, "End bone.").Ref("bone").Required(),
    F("knee", Type::Vec3, "Knee direction."),
    F("height", Type::Number, "Height."),
    F("pad", Type::Number, "Pad."),
    F("floor", Type::Number, "Floor."),
    F("center", Type::Vec3, "Center."),
};
const Item kIkChainItem = MakeItem("ikChain", kIkChain);

// --- animations and sequences ---

const Field kFrames[] = {
    F("start", Type::Integer, "First frame.").Required(),
    F("end", Type::Integer, "Last frame.").Required(),
};
const Item kFramesItem = MakeItem("frames", kFrames);

// options shared by animations and a sequence's first clip
const Field kClip[] = {
    F("fps", Type::Number, "Frame rate."),
    F("loop", Type::Bool, "Looping."),
    F("noForceLoop", Type::Bool, "Do not force the loop."),
    F("startLoop", Type::Integer, "Loop restart frame."),
    F("frames", Type::Object, "Frame range.").Of(&kFramesItem),
    F("numFrames", Type::Integer, "Resample to this many frames."),
    F("reverse", Type::Bool, "Play backwards."),
    F("origin", Type::Vec3, "Clip translation."),
    F("rotate", Type::Number, "Clip yaw in degrees."),
    F("angles", Type::Vec3, "Clip rotation in degrees."),
    F("scale", Type::Number, "Clip scale."),
    F("motionRollback", Type::Number, "Motion rollback in seconds."),
    F("ignoreScale", Type::Bool, "Ignore $transformmodel scale."),
};
const Item kClipItem = MakeItem("clip", kClip);

const Field kAnimation[] = {
    F("name", Type::String, "Animation name.").Required(),
    F("file", Type::File, "Source file."),
    F("bindPose", Type::Bool, "The compile's own bind pose instead of a file."),
    F("declare", Type::Bool, "An empty slot an included model fills."),
    F("snap", Type::Bool, "Snap."),
    F("post", Type::Bool, "Post."),
};
const Item kAnimationItem = MakeItem("animation", kAnimation, &kClipItem);

const Field kActivity[] = {
    F("name", Type::String, "Activity, e.g. ACT_IDLE.").Required(),
    F("weight", Type::Integer, "Activity weight."),
};
const Item kActivityItem = MakeItem("activity", kActivity);

const Field kBlend[] = {
    F("parameter", Type::String, "Pose parameter.").Ref("poseParameter").Required(),
    F("min", Type::Number, "Start value.").Required(),
    F("max", Type::Number, "End value.").Required(),
};
const Item kBlendItem = MakeItem("blend", kBlend);

const Field kAddLayer[] = {
    F("sequence", Type::String, "Layered sequence.").Ref("sequence").Required(),
    F("local", Type::Bool, "Local layer."),
};
const Item kAddLayerItem = MakeItem("addLayer", kAddLayer);

const Field kBlendLayer[] = {
    F("sequence", Type::String, "Layered sequence.").Ref("sequence").Required(),
    F("start", Type::Number, "Start frame.").Required(),
    F("peak", Type::Number, "Peak frame.").Required(),
    F("tail", Type::Number, "Tail frame.").Required(),
    F("end", Type::Number, "End frame.").Required(),
    F("xfade", Type::Bool, "Crossfade."),
    F("spline", Type::Bool, "Spline."),
    F("noBlend", Type::Bool, "No blend."),
    F("local", Type::Bool, "Local layer."),
    F("poseParameter", Type::String, "Drive by a pose parameter.").Ref("poseParameter"),
};
const Item kBlendLayerItem = MakeItem("blendLayer", kBlendLayer);

const Field kIkLock[] = {
    F("chain", Type::String, "IK chain.").Ref("ikChain").Required(),
    F("position", Type::Number, "Position weight.").Required(),
    F("rotation", Type::Number, "Rotation weight.").Required(),
};
const Item kIkLockItem = MakeItem("ikLock", kIkLock);

const Field kEvent[] = {
    F("name", Type::String, "Event name or numeric id.").Required(),
    F("frame", Type::Integer, "Frame.").Required(),
    F("options", Type::String, "Option string the game reads."),
};
const Item kEventItem = MakeItem("event", kEvent);

const Field kAnimTag[] = {
    F("name", Type::String, "Tag name.").Required(),
    F("cycle", Type::Number, "Cycle fraction.").Required(),
};
const Item kAnimTagItem = MakeItem("animTag", kAnimTag);

const Field kTransition[] = {
    F("from", Type::String, "Entry node.").Required(),
    F("to", Type::String, "Exit node.").Required(),
    F("reversible", Type::Bool, "Reversible."),
};
const Item kTransitionItem = MakeItem("transition", kTransition);

const Field kSequence[] = {
    F("name", Type::String, "Sequence name.").Required(),
    F("file", Type::File, "Source file of the only clip."),
    F("files", Type::Files, "Source files of a blend grid."),
    F("animations", Type::Strings, "Animations of a blend grid.").Ref("animation"),
    F("bindPose", Type::Bool, "The compile's own bind pose instead of a file."),
    F("declare", Type::Bool, "An empty slot an included model fills."),
    F("activity", Type::Object, "Activity.").Of(&kActivityItem),
    F("snap", Type::Bool, "Snap."),
    F("autoplay", Type::Bool, "Autoplay."),
    F("hidden", Type::Bool, "Hidden."),
    F("realtime", Type::Bool, "Realtime."),
    F("post", Type::Bool, "Post."),
    F("delta", Type::Bool, "Delta."),
    F("preDelta", Type::Bool, "Pre-delta."),
    F("worldSpace", Type::Bool, "World space."),
    F("worldRelative", Type::Bool, "World relative."),
    F("rootDriver", Type::String, "Root transform bone.").Ref("bone"),
    F("exitPhase", Type::Number, "Exit phase."),
    F("poseCycle", Type::String, "Pose parameter driving the cycle.").Ref("poseParameter"),
    F("fadeIn", Type::Number, "Fade-in time."),
    F("fadeOut", Type::Number, "Fade-out time."),
    F("blendWidth", Type::Integer, "Blend grid width."),
    F("blends", Type::List, "Blend axes (at most 2).").Of(&kBlendItem),
    F("addLayers", Type::List, "Add layers.").Of(&kAddLayerItem),
    F("blendLayers", Type::List, "Blend layers.").Of(&kBlendLayerItem),
    F("ikLocks", Type::List, "IK locks.").Of(&kIkLockItem),
    F("events", Type::List, "Events.").Of(&kEventItem),
    F("animTags", Type::List, "Animation tags.").Of(&kAnimTagItem),
    F("node", Type::String, "Transition node."),
    F("transition", Type::Object, "Transition between nodes.").Of(&kTransitionItem),
    F("activityModifiers", Type::Strings, "Activity modifiers."),
};
const Item kSequenceItem = MakeItem("sequence", kSequence, &kClipItem);

// --- files ---

const Field kContent[] = {
    F("variables", Type::Variables, "New variables, written $name$ in any string."),
    F("redefineVariables", Type::Variables, "New values for existing variables."),
    F("includePrefabs", Type::Prefabs, "Other .pulsemodel files merged in."),
    F("searchDirs", Type::Files, "Extra directories for source files."),
    F("model", Type::Object, "Model settings.").Of(&kSettingsItem),
    F("materials", Type::Object, "Materials and skins.").Of(&kMaterialsItem),
    F("meshes", Type::List, "Render meshes.").Of(&kMeshItem),
    F("modelGroups", Type::List, "Model groups (bodygroups).").Of(&kModelGroupItem),
    F("lods", Type::List, "LODs, in switch order.").Of(&kLodItem),
    F("bones", Type::Object, "Skeleton edits.").Of(&kBonesItem),
    F("attachments", Type::List, "Attachments.").Of(&kAttachmentItem),
    F("hitboxSets", Type::List, "Hitbox sets.").Of(&kHitboxSetItem),
    F("poseParameters", Type::List, "Pose parameters.").Of(&kPoseParameterItem),
    F("ikChains", Type::List, "IK chains.").Of(&kIkChainItem),
    F("animations", Type::List, "Named animations.").Of(&kAnimationItem),
    F("sequences", Type::List, "Sequences, in index order.").Of(&kSequenceItem),
    F("includeModels", Type::Strings, "Compiled .mdl files whose sequences this model uses."),
};
const Item kContentItem = MakeItem("content", kContent);

const Field kModel[] = {
    F("id", Type::String, "Model id, unique in the file.").Required(),
    F("compile", Type::Bool, "Compiled by default (true when omitted)."),
};
const Item kModelItem = MakeItem("modelEntry", kModel, &kContentItem);

const Field kRoot[] = {
    F("format", Type::String, "Always \"pulsemodel\".").Required(),
    F("version", Type::Integer, "Format version.").Required(),
    F("shared", Type::Object, "Content merged into every model.").Of(&kContentItem),
    F("models", Type::List, "Models.").Of(&kModelItem).Required(),
};
const Item kRootItem = MakeItem("file", kRoot);

const Item* const kAllItems[] = {
    &kRootItem, &kModelItem, &kContentItem, &kSettingsItem, &kTransformItem, &kBudgetItem,
    &kBoxItem, &kBBoxItem, &kIllumPositionItem, &kEyePositionItem, &kMaterialsItem, &kSkinItem,
    &kReplaceItem, &kMeshItem, &kWeldItem, &kScaleBoneItem, &kModelGroupItem, &kChoiceItem,
    &kLodItem, &kLodModelItem, &kLodDecimateItem, &kSmallMeshesItem, &kLodBoneItem, &kLodWordItem,
    &kBonesItem, &kDefineBoneItem, &kPoseItem, &kRenameItem, &kTransformBoneItem,
    &kTransformWeightsItem, &kAttachmentItem, &kHitboxSetItem, &kHitboxItem, &kPoseParameterItem,
    &kIkChainItem, &kClipItem, &kFramesItem, &kAnimationItem, &kSequenceItem, &kActivityItem,
    &kBlendItem, &kAddLayerItem, &kBlendLayerItem, &kIkLockItem, &kEventItem, &kAnimTagItem,
    &kTransitionItem,
};

const Field* FindField(const Item& item, const std::string& key) {
    for (const Item* i = &item; i; i = i->base)
        for (size_t f = 0; f < i->count; ++f)
            if (key == i->fields[f].key) return &i->fields[f];
    return nullptr;
}

const char* TypeName(Type type) {
    switch (type) {
    case Type::String: return "string";
    case Type::Number: return "number";
    case Type::Integer: return "integer";
    case Type::Bool: return "bool";
    case Type::Vec3: return "vec3";
    case Type::Enum: return "enum";
    case Type::File: return "file";
    case Type::Strings: return "strings";
    case Type::Files: return "files";
    case Type::Variables: return "variables";
    case Type::Prefabs: return "prefabs";
    case Type::Object: return "object";
    case Type::List: return "list";
    case Type::Any: return "any";
    }
    return "any";
}

// ---------------------------------------------------------------------------
// Load context
// ---------------------------------------------------------------------------

std::string Pointer(const std::string& parent, const std::string& key) {
    std::string out = parent + "/";
    for (char ch : key) {
        if (ch == '~') out += "~0";
        else if (ch == '/') out += "~1";
        else out += ch;
    }
    return out;
}

std::string Pointer(const std::string& parent, size_t index) {
    return parent + "/" + std::to_string(index);
}

// [file index, JSON pointer] of `key` in `obj`, or of `obj` itself for an empty key.
Json KeyOrigin(const Json& obj, const std::string& key) {
    if (!key.empty()) {
        const auto merged = obj.find(kKeyOrigins);
        if (merged != obj.end() && merged->contains(key)) return merged->at(key);
    }
    const Json& o = obj.at(kOrigin);
    const std::string base = o[1].get<std::string>();
    return Json::array({o[0], key.empty() ? base : Pointer(base, key)});
}

struct SourceFile {
    std::string path; // absolute
    fs::path dir;
    bool prefab = false;
};

using Scope = std::vector<std::pair<std::string, std::string>>;

struct Loader {
    const LoadOptions& options;
    std::string* err;
    std::vector<SourceFile> files;
    std::vector<Json> docs;               // parallel to files
    std::vector<std::string> prefabStack; // cycle guard, canonical paths

    bool Fail(size_t file, const std::string& path, const std::string& msg) {
        if (!err) return false;
        const pulse::diag::Location at{files[file].path, 0, path.empty() ? "/" : path};
        *err = fs::path(at.file).filename().string() + ": " + at.path + ": " + msg;
        pulse::diag::NoteFailure(*err, at, msg);
        return false;
    }

    bool Fail(const Json& obj, const std::string& key, const std::string& msg) {
        const Json at = KeyOrigin(obj, key);
        return Fail(at[0].get<size_t>(), at[1].get<std::string>(), msg);
    }

    bool IsPinned(const std::string& name) const {
        for (const auto& [pinned, value] : options.defvars)
            if (pinned == name) return true;
        return false;
    }
};

// ---------------------------------------------------------------------------
// Validation - unknown keys, types, required fields. Runs before anything is
// substituted, so a number may still be a "$name$" string here.
// ---------------------------------------------------------------------------

bool IsVarRef(const Json& v) {
    if (!v.is_string()) return false;
    const std::string& s = v.get_ref<const std::string&>();
    return s.size() > 2 && s.front() == '$' && s.back() == '$' &&
           s.find('$', 1) == s.size() - 1;
}

bool CheckValue(Loader& l, size_t file, const std::string& path, const Field& field, Json& v);

bool CheckObject(Loader& l, size_t file, const std::string& path, const Item& item, Json& obj) {
    if (!obj.is_object())
        return l.Fail(file, path, std::string("expected an object (") + item.name + ")");

    for (auto& [key, value] : obj.items()) {
        if (key == "editor") continue;
        const Field* field = FindField(item, key);
        if (!field)
            return l.Fail(file, Pointer(path, key), "unknown key \"" + key + "\" in " + item.name);
        if (!CheckValue(l, file, Pointer(path, key), *field, value))
            return false;
    }

    for (const Item* i = &item; i; i = i->base)
        for (size_t f = 0; f < i->count; ++f)
            if (i->fields[f].required && !obj.contains(i->fields[f].key))
                return l.Fail(file, path, std::string("missing \"") + i->fields[f].key + "\"");
    obj[kOrigin] = Json::array({file, path});
    return true;
}

bool CheckValue(Loader& l, size_t file, const std::string& path, const Field& field, Json& v) {
    auto fail = [&](const char* what) { return l.Fail(file, path, std::string("expected ") + what); };
    auto stringList = [&]() {
        if (!v.is_array()) return fail("an array of strings");
        for (size_t i = 0; i < v.size(); ++i)
            if (!v[i].is_string()) return l.Fail(file, Pointer(path, i), "expected a string");
        return true;
    };

    switch (field.type) {
    case Type::String:
    case Type::File:
        return v.is_string() || fail("a string");
    case Type::Number:
        return v.is_number() || IsVarRef(v) || fail("a number");
    case Type::Integer:
        return v.is_number_integer() || IsVarRef(v) || fail("an integer");
    case Type::Bool:
        return v.is_boolean() || IsVarRef(v) || fail("true or false");
    case Type::Vec3:
        if (!v.is_array() || v.size() != 3) return fail("an array of 3 numbers");
        for (size_t i = 0; i < 3; ++i)
            if (!v[i].is_number() && !IsVarRef(v[i]))
                return l.Fail(file, Pointer(path, i), "expected a number");
        return true;
    case Type::Enum: {
        if (!v.is_string()) return fail("a string");
        if (IsVarRef(v)) return true;
        for (const char* const* e = field.values; *e; ++e)
            if (_stricmp(*e, v.get_ref<const std::string&>().c_str()) == 0) return true;
        std::string list;
        for (const char* const* e = field.values; *e; ++e) list += std::string(list.empty() ? "" : ", ") + *e;
        return l.Fail(file, path, "expected one of " + list);
    }
    case Type::Strings:
    case Type::Files:
        return stringList();
    case Type::Variables:
        if (!v.is_object()) return fail("an object of variable values");
        for (auto& [name, value] : v.items())
            if (!value.is_string() && !value.is_number() && !value.is_boolean())
                return l.Fail(file, Pointer(path, name), "expected a string, number or bool");
        return true;
    case Type::Prefabs:
        if (!v.is_array()) return fail("an array of prefabs");
        for (size_t i = 0; i < v.size(); ++i) {
            Json& p = v[i];
            const std::string at = Pointer(path, i);
            if (p.is_string()) continue;
            if (!p.is_object()) return l.Fail(file, at, "expected a file name or { file, variables }");
            for (auto& [key, value] : p.items()) {
                if (key == "file" && value.is_string()) continue;
                if (key == "variables" && value.is_object()) {
                    const Field vars{"variables", Type::Variables, ""};
                    if (!CheckValue(l, file, Pointer(at, key), vars, value)) return false;
                    continue;
                }
                return l.Fail(file, Pointer(at, key), "expected \"file\" or \"variables\"");
            }
            if (!p.contains("file")) return l.Fail(file, at, "missing \"file\"");
        }
        return true;
    case Type::Object:
        return CheckObject(l, file, path, *field.item, v);
    case Type::List:
        if (!v.is_array()) return fail("an array");
        for (size_t i = 0; i < v.size(); ++i)
            if (!CheckObject(l, file, Pointer(path, i), *field.item, v[i]))
                return false;
        return true;
    case Type::Any:
        return true;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Reading a file
// ---------------------------------------------------------------------------

// JSON with duplicate keys rejected: a second value for a key must not quietly win.
bool ParseJson(Loader& l, size_t file, const std::string& text, Json& out) {
    std::vector<std::set<std::string>> keys;
    std::string duplicate;
    auto callback = [&](int, Json::parse_event_t event, Json& parsed) {
        if (event == Json::parse_event_t::object_start) {
            keys.emplace_back();
        } else if (event == Json::parse_event_t::object_end) {
            keys.pop_back();
        } else if (event == Json::parse_event_t::key && duplicate.empty()) {
            const std::string key = parsed.get<std::string>();
            if (!keys.back().insert(key).second) duplicate = key;
        }
        return true;
    };

    try {
        out = Json::parse(text, callback);
    } catch (const Json::parse_error& e) {
        return l.Fail(file, "", std::string("invalid JSON: ") + e.what());
    }
    if (!duplicate.empty())
        return l.Fail(file, "", "key \"" + duplicate + "\" is written twice in one object");
    return true;
}

// Reads, parses and validates one .pulsemodel. Returns the file index, or -1.
int ReadFile(Loader& l, const fs::path& full, bool prefab, pulse::dependencies::Kind kind) {
    const std::string path = fs::absolute(full).lexically_normal().u8string();
    l.files.push_back({path, fs::path(path).parent_path(), prefab});
    const size_t file = l.files.size() - 1;

    const std::string* unsaved = loader::FindUnsavedText(l.options.unsaved, path);
    pulse::dependencies::Note(path, kind, unsaved);
    std::string text;
    if (unsaved) {
        text = *unsaved;
    } else {
        std::ifstream f(full, std::ios::binary);
        if (!f) return l.Fail(file, "", "cannot open \"" + path + "\""), -1;
        std::ostringstream buf;
        buf << f.rdbuf();
        text = buf.str();
    }
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        text.erase(0, 3);

    Json doc;
    if (!ParseJson(l, file, text, doc)) return -1;
    if (!doc.is_object()) return l.Fail(file, "", "expected a JSON object"), -1;

    // the header decides how the rest reads, so it is checked first
    const auto format = doc.find("format");
    if (format == doc.end() || *format != "pulsemodel")
        return l.Fail(file, "/format", "expected \"pulsemodel\""), -1;
    const auto version = doc.find("version");
    if (version == doc.end() || !version->is_number_integer() || version->get<int>() < 1)
        return l.Fail(file, "/version", "expected a format version"), -1;
    if (version->get<int>() > kFormatVersion)
        return l.Fail(file, "/version", "made by a newer PulseModel (format " +
                                        std::to_string(version->get<int>()) + ", this one reads up to " +
                                        std::to_string(kFormatVersion) + ")"), -1;

    if (!CheckObject(l, file, "", kRootItem, doc)) return -1;
    l.docs.resize(l.files.size());
    l.docs[file] = std::move(doc);
    return static_cast<int>(file);
}

// ---------------------------------------------------------------------------
// Variables - the .pulseqc rules: $name$ inside any string, case-sensitive,
// `variables` only defines and `redefineVariables` only changes.
// ---------------------------------------------------------------------------

const std::string* Lookup(const Scope& scope, const std::string& name) {
    for (const auto& [key, value] : scope)
        if (key == name) return &value;
    return nullptr;
}

bool Expand(Loader& l, size_t file, const std::string& path, const Scope& scope, std::string& s) {
    if (s.find('$') == std::string::npos) return true;

    std::string out;
    for (size_t i = 0; i < s.size();) {
        size_t end = i + 1;
        if (s[i] == '$') {
            while (end < s.size() && static_cast<unsigned char>(s[end]) > 32 && s[end] != '$') ++end;
        }
        if (s[i] != '$' || end >= s.size() || s[end] != '$' || end == i + 1) {
            out.push_back(s[i++]);
            continue;
        }

        const std::string name = s.substr(i + 1, end - i - 1);
        const std::string* value = Lookup(scope, name);
        if (!value) return l.Fail(file, path, "unknown variable \"$" + name + "$\"");
        out += *value;
        i = end + 1;
    }
    s = std::move(out);
    return true;
}

std::string ValueText(const Json& v) {
    if (v.is_string()) return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
    if (v.is_number_integer()) return std::to_string(v.get<long long>());

    // shortest text that reads back as the same double
    const double d = v.get<double>();
    char buf[64];
    for (int precision = 1; precision <= 17; ++precision) {
        std::snprintf(buf, sizeof(buf), "%.*g", precision, d);
        if (std::strtod(buf, nullptr) == d) break;
    }
    return buf;
}

// `fixed` names keep their value: -defvar pins, and the values a prefab is passed.
bool DefineVariables(Loader& l, const Json& owner, const char* key, bool redefine,
                     const std::set<std::string>& fixed, Scope& scope) {
    const auto vars = owner.find(key);
    if (vars == owner.end()) return true;

    const Json& origin = owner.at(kOrigin);
    const size_t file = origin[0].get<size_t>();
    const std::string base = Pointer(origin[1].get<std::string>(), key);
    for (const auto& [name, value] : vars->items()) {
        const std::string path = Pointer(base, name);
        if (fixed.count(name)) {
            if (l.IsPinned(name))
                std::printf("%s: %s ignored - pinned to \"%s\" by -defvar\n",
                            fs::path(l.files[file].path).filename().string().c_str(), path.c_str(),
                            Lookup(scope, name)->c_str());
            continue;
        }

        std::string text = ValueText(value);
        if (!Expand(l, file, path, scope, text)) return false;

        bool found = false;
        for (auto& [existing, current] : scope) {
            if (existing != name) continue;
            if (!redefine)
                return l.Fail(file, path, "variable \"" + name + "\" is already defined - "
                                          "use redefineVariables to change it");
            current = text;
            found = true;
        }
        if (redefine && !found)
            return l.Fail(file, path, "variable \"" + name + "\" is not defined - "
                                      "use variables to define it");
        if (!redefine) scope.emplace_back(name, std::move(text));
    }
    return true;
}

bool ParseNumber(const std::string& s, bool integer) {
    if (s.empty()) return false;
    char* end = nullptr;
    if (integer) std::strtoll(s.c_str(), &end, 10);
    else std::strtod(s.c_str(), &end);
    return end && *end == '\0';
}

// Substitutes one scalar in place; a substituted number or bool must still be one.
bool SubstituteValue(Loader& l, size_t file, const std::string& path, const Field& field,
                     const Scope& scope, Json& v) {
    if (!v.is_string()) return true;

    std::string s = v.get<std::string>();
    if (!Expand(l, file, path, scope, s)) return false;

    switch (field.type) {
    case Type::Number:
    case Type::Vec3:
        if (!ParseNumber(s, false)) return l.Fail(file, path, "\"" + s + "\" is not a number");
        v = std::strtod(s.c_str(), nullptr);
        return true;
    case Type::Integer:
        if (!ParseNumber(s, true)) return l.Fail(file, path, "\"" + s + "\" is not an integer");
        v = std::strtoll(s.c_str(), nullptr, 10);
        return true;
    case Type::Bool:
        if (s != "true" && s != "false") return l.Fail(file, path, "\"" + s + "\" is not true or false");
        v = s == "true";
        return true;
    case Type::Enum:
        for (const char* const* e = field.values; *e; ++e)
            if (_stricmp(*e, s.c_str()) == 0) { v = s; return true; }
        return l.Fail(file, path, "\"" + s + "\" is not a valid " + field.key);
    default:
        v = s;
        return true;
    }
}

// A prefab's relative source path is relative to the prefab, when the file is
// there; otherwise it is left to the root file's search directories.
std::string PrefabFile(const fs::path& dir, const std::string& value, bool directory) {
    const fs::path p = pulse::FilePath(value);
    if (p.is_absolute()) return value;

    const fs::path full = (dir / p).lexically_normal();
    if (directory) return full.u8string();

    std::error_code ec;
    if (fs::exists(full, ec)) return full.u8string();
    if (!p.has_extension())
        for (const char* ext : {".dmx", ".smd", ".fbx"})
            if (fs::exists(fs::path(full).concat(ext), ec)) return full.u8string();
    return value;
}

bool Substitute(Loader& l, const Item& item, const Scope& scope, Json& obj) {
    const Json origin = obj.at(kOrigin);
    const size_t file = origin[0].get<size_t>();
    const std::string base = origin[1].get<std::string>();
    const SourceFile& source = l.files[file];

    for (auto& [key, value] : obj.items()) {
        if (IsInternal(key) || key == "editor") continue;
        const Field* field = FindField(item, key);
        if (!field) continue;
        const std::string path = Pointer(base, key);

        switch (field->type) {
        case Type::Object:
            if (!Substitute(l, *field->item, scope, value)) return false;
            break;
        case Type::List:
            for (Json& entry : value)
                if (!Substitute(l, *field->item, scope, entry)) return false;
            break;
        case Type::Vec3:
        case Type::Strings:
        case Type::Files:
            for (size_t i = 0; i < value.size(); ++i) {
                if (!SubstituteValue(l, file, Pointer(path, i), *field, scope, value[i])) return false;
                if (field->type == Type::Files && source.prefab)
                    value[i] = PrefabFile(source.dir, value[i].get<std::string>(),
                                          key == "searchDirs");
            }
            break;
        case Type::Variables:
        case Type::Prefabs:
        case Type::Any:
            break;
        default:
            if (!SubstituteValue(l, file, path, *field, scope, value)) return false;
            if (field->type == Type::File && source.prefab)
                value = PrefabFile(source.dir, value.get<std::string>(), false);
            break;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Prefabs and merging
// ---------------------------------------------------------------------------

bool CollectContent(Loader& l, Json part, const Scope& scope, std::vector<Json>& out);

// Loads the prefab one includePrefabs entry names and appends its content.
bool IncludePrefab(Loader& l, const Json& owner, size_t index, const Scope& callerScope,
                   std::vector<Json>& out) {
    const Json& origin = owner.at(kOrigin);
    const size_t file = origin[0].get<size_t>();
    const std::string at = Pointer(Pointer(origin[1].get<std::string>(), "includePrefabs"), index);
    const Json& entry = owner.at("includePrefabs")[index];

    std::string name = entry.is_string() ? entry.get<std::string>() : entry.at("file").get<std::string>();
    if (!Expand(l, file, at, callerScope, name)) return false;
    fs::path full = pulse::FilePath(name);
    if (!full.is_absolute()) full = l.files[file].dir / full;

    std::error_code ec;
    const fs::path canonical = fs::weakly_canonical(full, ec);
    const std::string key = ec ? full.lexically_normal().string() : canonical.string();
    for (const std::string& open : l.prefabStack)
        if (_stricmp(open.c_str(), key.c_str()) == 0)
            return l.Fail(file, at, "prefab \"" + name + "\" includes itself");

    // values passed in are written down explicitly, so they win over the prefab's own
    Scope scope;
    std::set<std::string> fixed;
    for (const auto& [pinned, value] : l.options.defvars) {
        scope.emplace_back(pinned, value);
        fixed.insert(pinned);
    }
    if (entry.is_object() && entry.contains("variables")) {
        for (const auto& [var, value] : entry.at("variables").items()) {
            std::string text = ValueText(value);
            if (!Expand(l, file, Pointer(Pointer(at, "variables"), var), callerScope, text)) return false;
            if (fixed.count(var)) continue;
            scope.emplace_back(var, std::move(text));
            fixed.insert(var);
        }
    }

    const int prefab = ReadFile(l, full, true, pulse::dependencies::Kind::Include);
    if (prefab < 0) return false;
    Json doc = l.docs[static_cast<size_t>(prefab)];
    const Json& models = doc.at("models");
    if (models.size() > 1)
        return l.Fail(static_cast<size_t>(prefab), "/models", "a prefab holds at most one model");

    l.prefabStack.push_back(key);
    Json shared = doc.contains("shared") ? doc.at("shared") : Json();
    Json model = models.empty() ? Json() : models[0];
    if (!shared.is_null() && (!DefineVariables(l, shared, "variables", false, fixed, scope) ||
                              !CollectContent(l, shared, scope, out)))
        return false;
    if (!model.is_null()) {
        model.erase("id");
        model.erase("compile");
        if (!DefineVariables(l, model, "variables", false, fixed, scope) ||
            !DefineVariables(l, model, "redefineVariables", true, fixed, scope) ||
            !CollectContent(l, model, scope, out))
            return false;
    }
    l.prefabStack.pop_back();
    return true;
}

// Substitutes one content object and appends it, followed by the prefabs it includes.
bool CollectContent(Loader& l, Json part, const Scope& scope, std::vector<Json>& out) {
    part.erase("variables");
    part.erase("redefineVariables");
    if (!Substitute(l, kContentItem, scope, part)) return false;

    const Json prefabs = part.contains("includePrefabs") ? part.at("includePrefabs") : Json::array();
    const Json owner = part;
    part.erase("includePrefabs");
    out.push_back(std::move(part));

    for (size_t i = 0; i < prefabs.size(); ++i)
        if (!IncludePrefab(l, owner, i, scope, out)) return false;
    return true;
}

std::string OriginText(const Loader& l, const Json& obj, const std::string& key) {
    const Json at = KeyOrigin(obj, key);
    return fs::path(l.files[at[0].get<size_t>()].path).filename().string() + ": " +
           at[1].get<std::string>();
}

// Lists append; objects merge key by key; a value set by two contributors is an error.
bool Merge(Loader& l, const Item& item, Json& dst, const Json& src) {
    for (const auto& [key, value] : src.items()) {
        if (IsInternal(key)) continue;
        if (!dst.contains(key)) {
            dst[kKeyOrigins][key] = KeyOrigin(src, key);
            dst[key] = value;
            continue;
        }
        if (key == "editor") continue;

        const Field* field = FindField(item, key);
        switch (field->type) {
        case Type::List:
        case Type::Strings:
        case Type::Files:
            for (const Json& entry : value) dst[key].push_back(entry);
            break;
        case Type::Object:
            if (!Merge(l, *field->item, dst[key], value)) return false;
            break;
        default:
            return l.Fail(src, key, "\"" + key + "\" is also set by " + OriginText(l, dst, key));
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Lowering - each section becomes the .pulseqc commands it stands for, in a
// fixed order: settings and scale before any source loads, meshes before the
// groups, LODs and boxes that name them, animations before sequences.
// ---------------------------------------------------------------------------

struct Emitter {
    Loader& l;
    std::vector<loader::ScriptToken> tokens;
    std::vector<pulse::diag::Location> lines;

    // Starts a new script line, reported as `key` of `obj` (or `obj` itself).
    void Line(const Json& obj, const std::string& key = {}) {
        const Json at = KeyOrigin(obj, key);
        const std::string path = at[1].get<std::string>();
        lines.push_back({l.files[at[0].get<size_t>()].path, 0, path.empty() ? "/" : path});
    }

    int Current() const { return static_cast<int>(lines.size()); }
    void Word(const std::string& s) { tokens.push_back({s, Current(), false}); }
    void Text(const std::string& s) { tokens.push_back({s, Current(), true}); }
    void Num(const Json& v) { Word(ValueText(v)); }
    void Vec(const Json& v) { for (const Json& n : v) Num(n); }

    void Command(const char* name, const Json& obj, const std::string& key = {}) {
        Line(obj, key);
        Word(name);
    }
};

const Json* Get(const Json& obj, const char* key) {
    const auto it = obj.find(key);
    return it == obj.end() ? nullptr : &*it;
}

std::string Str(const Json& obj, const char* key) { return obj.at(key).get<std::string>(); }

bool True(const Json& obj, const char* key) {
    const Json* v = Get(obj, key);
    return v && v->get<bool>();
}

void EmitFlag(Emitter& e, const Json& obj, const char* key, const char* word) {
    if (True(obj, key)) e.Word(word);
}

void EmitClause(Emitter& e, const Json& obj, const char* key, const char* word) {
    const Json* v = Get(obj, key);
    if (!v) return;
    e.Word(word);
    if (v->is_array()) e.Vec(*v);
    else if (v->is_string()) e.Text(v->get<std::string>());
    else e.Num(*v);
}

void EmitNames(Emitter& e, const Json& list) {
    for (const Json& name : list) e.Text(name.get<std::string>());
}

bool LowerSettings(Emitter& e, const Json& m) {
    if (const Json* name = Get(m, "name")) {
        e.Command("$modelname", m, "name");
        e.Text(name->get<std::string>());
    }
    if (const Json* v = Get(m, "archetype")) {
        e.Command("$modelarchetype", m, "archetype");
        e.Word(v->get<std::string>());
    }
    if (const Json* t = Get(m, "transform")) {
        if (Get(*t, "upAxis") && Get(*t, "angles"))
            return e.l.Fail(*t, "upAxis", "upAxis and angles both set the model rotation - use one");
        if (const Json* up = Get(*t, "upAxis")) {
            e.Command("$upaxis", *t, "upAxis");
            e.Word(up->get<std::string>());
        }
        if (Get(*t, "origin") || Get(*t, "angles") || Get(*t, "scale")) {
            e.Command("$transformmodel", *t);
            EmitClause(e, *t, "origin", "origin");
            EmitClause(e, *t, "angles", "angles");
            EmitClause(e, *t, "scale", "scale");
        }
    }
    if (const Json* v = Get(m, "renderPass")) {
        e.Command("$renderpass", m, "renderPass");
        e.Word(v->get<std::string>());
    }
    if (const Json* b = Get(m, "budget")) {
        e.Command("$modelbudget", *b);
        e.Word("{");
        EmitClause(e, *b, "bones", "bones");
        EmitClause(e, *b, "materials", "materials");
        e.Word("}");
    }
    if (const Json* v = Get(m, "surfaceProp")) {
        e.Command("$surfaceprop", m, "surfaceProp");
        e.Text(v->get<std::string>());
    }
    if (const Json* v = Get(m, "contents"); v && !v->empty()) {
        e.Command("$contents", m, "contents");
        EmitNames(e, *v);
    }

    static const struct { const char* key; const char* command; } kFlags[] = {
        {"noDx80", "$nodx80"},
        {"autoCenter", "$autocenter"},
        {"forceWriteVertexData", "$forcewritevertexdata"},
        {"ambientBoost", "$ambientboost"},
        {"doNotCastShadows", "$donotcastshadows"},
        {"forcePhonemeCrossfade", "$forcephonemecrossfade"},
        {"noForcedFade", "$noforcedfade"},
        {"castTextureShadows", "$casttextureshadows"},
        {"skipBoneInBBox", "$skipboneinbbox"},
    };
    for (const auto& flag : kFlags)
        if (True(m, flag.key)) e.Command(flag.command, m, flag.key);

    static const struct { const char* key; const char* command; } kValues[] = {
        {"vtxFormat", "$vtxformat"},
        {"constantDirectionalLight", "$constantdirectionallight"},
        {"allowRootLods", "$allowrootlods"},
        {"minLod", "$minlod"},
        {"maxEyeDeflection", "$maxeyedeflection"},
    };
    for (const auto& value : kValues) {
        if (const Json* v = Get(m, value.key)) {
            e.Command(value.command, m, value.key);
            e.Num(*v);
        }
    }

    if (const Json* box = Get(m, "cbox")) {
        e.Command("$cbox", *box);
        e.Vec(box->at("min"));
        e.Vec(box->at("max"));
    }
    return true;
}

// $bbox may name meshes, so it and the two points run after the meshes load.
bool LowerPoints(Emitter& e, const Json& m) {
    if (const Json* box = Get(m, "bbox")) {
        const bool corners = Get(*box, "min") || Get(*box, "max");
        const Json* meshes = Get(*box, "meshes");
        if (corners == (meshes != nullptr) || (corners && !(Get(*box, "min") && Get(*box, "max"))))
            return e.l.Fail(*box, "", "bbox takes min and max, or meshes");
        e.Command("$bbox", *box);
        if (meshes) EmitNames(e, *meshes);
        else {
            e.Vec(box->at("min"));
            e.Vec(box->at("max"));
        }
    }
    if (const Json* p = Get(m, "illumPosition")) {
        e.Command("$illumposition", *p);
        e.Vec(p->at("position"));
        if (const Json* bone = Get(*p, "bone")) e.Text(bone->get<std::string>());
    }
    if (const Json* p = Get(m, "eyePosition")) {
        e.Command("$eyeposition", *p);
        e.Vec(p->at("position"));
        EmitFlag(e, *p, "autoHeight", "autoheight");
    }
    return true;
}

void LowerReplacements(Emitter& e, const Json* list, const char* command) {
    if (!list) return;
    for (const Json& r : *list) {
        e.Command(command, r);
        e.Text(Str(r, "material"));
        e.Text(Str(r, "with"));
    }
}

void LowerMesh(Emitter& e, const Json& mesh) {
    e.Command("$rendermesh", mesh);
    e.Text(Str(mesh, "name"));
    e.Text(Str(mesh, "file"));

    bool body = false;
    auto open = [&](const std::string& key) {
        if (!body) e.Word("{");
        body = true;
        e.Line(mesh, key);
    };

    if (True(mesh, "noMorph")) { open("noMorph"); e.Word("$nomorph"); }
    if (True(mesh, "flipNormals")) { open("flipNormals"); e.Word("$flipnormals"); }
    for (const char* key : {"removeMeshes", "removeMeshWords"}) {
        const Json* list = Get(mesh, key);
        if (!list) continue;
        for (const Json& name : *list) {
            open(key);
            e.Word(std::string(key) == "removeMeshes" ? "$removemesh" : "$removemeshword");
            e.Text(name.get<std::string>());
        }
    }
    if (const Json* weld = Get(mesh, "weld")) {
        open("weld");
        e.Word("$weld");
        EmitFlag(e, *weld, "seams", "seams");
        EmitClause(e, *weld, "sharp", "sharp");
    }
    if (const Json* v = Get(mesh, "decimate")) {
        open("decimate");
        e.Word("$decimate");
        e.Num(*v);
    }
    if (const Json* v = Get(mesh, "skinnedBoneCull")) {
        open("skinnedBoneCull");
        e.Word("$skinnedbonecull");
        e.Word(v->get<std::string>());
    }
    if (const Json* list = Get(mesh, "scaleBones")) {
        for (const Json& s : *list) {
            if (!body) e.Word("{");
            body = true;
            e.Line(s);
            e.Word("$scalebone");
            e.Text(Str(s, "bone"));
            e.Num(s.at("factor"));
            if (const Json* axes = Get(s, "axes"))
                for (const Json& axis : *axes) e.Word(axis.get<std::string>());
        }
    }
    if (body) e.Word("}");
}

bool LowerModelGroup(Emitter& e, const Json& group) {
    e.Command("$modelgroup", group);
    e.Text(Str(group, "name"));
    e.Word("{");
    for (const Json& choice : group.at("choices")) {
        const Json* meshes = Get(choice, "meshes");
        if (True(choice, "blank")) {
            if (meshes || Get(choice, "name"))
                return e.l.Fail(choice, "blank", "a blank choice has no name or meshes");
            e.Line(choice);
            e.Word("blank");
            continue;
        }
        if (!meshes || meshes->empty())
            return e.l.Fail(choice, "", "a choice needs meshes, or blank: true");
        e.Line(choice);
        e.Word("mesh");
        if (const Json* name = Get(choice, "name")) {
            e.Word("name");
            e.Text(name->get<std::string>());
        }
        EmitNames(e, *meshes);
    }
    e.Word("}");
    return true;
}

bool LowerLod(Emitter& e, const Json& lod) {
    const bool shadow = True(lod, "shadow");
    if (shadow == (Get(lod, "switch") != nullptr))
        return e.l.Fail(lod, "", "a LOD takes a switch value, or shadow: true");

    e.Command(shadow ? "$shadowlod" : "$lod", lod);
    if (!shadow) e.Num(lod.at("switch"));
    e.Word("{");

    if (const Json* list = Get(lod, "replaceModels")) {
        for (const Json& r : *list) {
            const Json* mesh = Get(r, "mesh");
            const Json* file = Get(r, "file");
            if ((mesh != nullptr) == (file != nullptr))
                return e.l.Fail(r, "", "replaceModels takes a mesh or a file");
            e.Line(r);
            e.Word("replacemodel");
            e.Text(Str(r, "model"));
            e.Text((mesh ? mesh : file)->get<std::string>());
        }
    }
    if (const Json* list = Get(lod, "removeModels")) {
        for (const Json& name : *list) {
            e.Line(lod, "removeModels");
            e.Word("removemodel");
            e.Text(name.get<std::string>());
        }
    }
    if (const Json* list = Get(lod, "decimateModels")) {
        for (const Json& d : *list) {
            e.Line(d);
            e.Word("decimatemodel");
            e.Text(Str(d, "model"));
            e.Num(d.at("factor"));
        }
    }
    if (const Json* v = Get(lod, "decimateAll")) {
        e.Line(lod, "decimateAll");
        e.Word("decimateallmodel");
        e.Num(*v);
    }
    if (const Json* small = Get(lod, "removeSmallMeshes")) {
        e.Line(*small);
        e.Word("removesmallmeshes");
        e.Num(small->at("size"));
        if (const Json* meshes = Get(*small, "meshes")) EmitNames(e, *meshes);
    }
    if (const Json* list = Get(lod, "replaceBones")) {
        for (const Json& r : *list) {
            e.Line(r);
            e.Word("replacebone");
            e.Text(Str(r, "bone"));
            e.Text(Str(r, "with"));
        }
    }
    if (const Json* list = Get(lod, "collapseBoneTrees")) {
        for (const Json& bone : *list) {
            e.Line(lod, "collapseBoneTrees");
            e.Word("bonetreecollapse");
            e.Text(bone.get<std::string>());
        }
    }
    if (const Json* list = Get(lod, "replaceMaterials")) {
        for (const Json& r : *list) {
            e.Line(r);
            e.Word("replacematerial");
            e.Text(Str(r, "material"));
            e.Text(Str(r, "with"));
        }
    }
    if (const Json* list = Get(lod, "replaceMaterialWords")) {
        for (const Json& r : *list) {
            e.Line(r);
            e.Word("replacematerialword");
            e.Text(Str(r, "word"));
            e.Text(Str(r, "with"));
        }
    }
    for (const char* key : {"removeMeshes", "removeMeshWords"}) {
        const Json* list = Get(lod, key);
        if (!list) continue;
        for (const Json& name : *list) {
            e.Line(lod, key);
            e.Word(std::string(key) == "removeMeshes" ? "removemesh" : "removemeshword");
            e.Text(name.get<std::string>());
        }
    }

    static const struct { const char* key; const char* word; } kFlags[] = {
        {"collapseJiggleBones", "collapsejigglebones"},
        {"collapseProceduralBones", "collapseproceduralbones"},
        {"skeletalAwareDecimation", "use_skeletalaware_decimation"},
        {"useShadowLodMaterials", "use_shadowlod_materials"},
    };
    for (const auto& flag : kFlags) {
        if (!True(lod, flag.key)) continue;
        e.Line(lod, flag.key);
        e.Word(flag.word);
    }
    if (const Json* facial = Get(lod, "facial")) {
        e.Line(lod, "facial");
        e.Word(facial->get<bool>() ? "facial" : "nofacial");
    }
    e.Word("}");
    return true;
}

void LowerBones(Emitter& e, const Json& b) {
    if (const Json* list = Get(b, "define")) {
        for (const Json& d : *list) {
            e.Command("$definebone", d);
            e.Text(Str(d, "name"));
            const Json* parent = Get(d, "parent");
            e.Text(parent ? parent->get<std::string>() : "");
            EmitFlag(e, d, "unlocked", "unlocked");
            e.Vec(d.at("position"));
            e.Vec(d.at("angles"));
            if (const Json* realign = Get(d, "realign")) {
                e.Vec(realign->at("position"));
                e.Vec(realign->at("angles"));
            }
        }
    }
    if (const Json* list = Get(b, "renames")) {
        for (const Json& r : *list) {
            e.Command("$renamebone", r);
            e.Text(Str(r, "bone"));
            e.Text(Str(r, "to"));
        }
    }
    if (const Json* list = Get(b, "transforms")) {
        for (const Json& t : *list) {
            e.Command("$transformbone", t);
            e.Text(Str(t, "bone"));
            EmitClause(e, t, "angles", "angles");
            EmitClause(e, t, "position", "position");
            EmitFlag(e, t, "worldAngles", "worldangles");
            EmitFlag(e, t, "worldPosition", "worldposition");
            if (const Json* w = Get(t, "transformWeights")) {
                e.Word("transformweights");
                e.Text(Str(*w, "residualBone"));
                if (const Json* factor = Get(*w, "factor")) {
                    e.Num(*factor);
                    if (const Json* smoothing = Get(*w, "smoothing")) e.Num(*smoothing);
                } else if (Get(*w, "smoothing")) {
                    e.Num(1);
                    e.Num(w->at("smoothing"));
                }
            }
            EmitClause(e, t, "offset", "offset");
            EmitFlag(e, t, "transformVerts", "transformverts");
            EmitFlag(e, t, "transformChildren", "transformchildren");
            EmitFlag(e, t, "ignoreAnimation", "ignoreanimation");
            EmitFlag(e, t, "ignoreHitbox", "ignorehitbox");
        }
    }

    static const struct { const char* key; const char* command; } kLists[] = {
        {"boneMerge", "$bonemerge"},
        {"doNotCollapse", "$donotcollapse"},
        {"alwaysCollapse", "$alwayscollapse"},
    };
    for (const auto& list : kLists) {
        const Json* names = Get(b, list.key);
        if (!names || names->empty()) continue;
        e.Command(list.command, b, list.key);
        EmitNames(e, *names);
    }

    if (const Json* v = Get(b, "root")) {
        e.Command("$root", b, "root");
        e.Text(v->get<std::string>());
    }
    if (const Json* v = Get(b, "cullMethod")) {
        e.Command("$bonecullmethod", b, "cullMethod");
        e.Word(v->get<std::string>());
    }
    if (True(b, "realign")) e.Command("$realignbones", b, "realign");
    if (True(b, "lockLengths")) e.Command("$lockbonelengths", b, "lockLengths");
}

void LowerAttachment(Emitter& e, const Json& a) {
    e.Command("$attachment", a);
    e.Text(Str(a, "name"));
    EmitClause(e, a, "bone", "bone");
    EmitClause(e, a, "position", "origin");
    EmitClause(e, a, "angles", "angles");
    EmitFlag(e, a, "rigid", "rigid");
    EmitFlag(e, a, "absolute", "absolute");
    EmitFlag(e, a, "worldAlign", "world_align");
    static const struct { const char* key; const char* word; } kSelectors[] = {
        {"flexGroups", "flexgroup"}, {"flexMorphs", "flexmorph"}, {"materials", "material"}};
    for (const auto& selector : kSelectors) {
        const Json* list = Get(a, selector.key);
        if (!list) continue;
        for (const Json& name : *list) {
            e.Word(selector.word);
            e.Text(name.get<std::string>());
        }
    }
}

void LowerHitboxSet(Emitter& e, const Json& set) {
    e.Command("$hitboxset", set);
    e.Text(Str(set, "name"));
    e.Word("{");
    if (const Json* boxes = Get(set, "boxes")) {
        for (const Json& box : *boxes) {
            e.Line(box);
            e.Word("$hbox");
            e.Num(box.at("group"));
            e.Text(Str(box, "bone"));
            e.Vec(box.at("min"));
            e.Vec(box.at("max"));
            EmitClause(e, box, "angles", "angles");
            EmitClause(e, box, "radius", "radius");
            EmitClause(e, box, "name", "name");
        }
    }
    e.Word("}");
}

void LowerIk(Emitter& e, const Json& m) {
    if (const Json* list = Get(m, "poseParameters")) {
        for (const Json& p : *list) {
            e.Command("$poseparameter", p);
            e.Text(Str(p, "name"));
            e.Num(p.at("min"));
            e.Num(p.at("max"));
            EmitFlag(e, p, "wrap", "wrap");
            EmitClause(e, p, "loop", "loop");
        }
    }
    if (const Json* list = Get(m, "ikChains")) {
        for (const Json& c : *list) {
            e.Command("$ikchain", c);
            e.Text(Str(c, "name"));
            e.Text(Str(c, "bone"));
            EmitClause(e, c, "knee", "knee");
            EmitClause(e, c, "height", "height");
            EmitClause(e, c, "pad", "pad");
            EmitClause(e, c, "floor", "floor");
            EmitClause(e, c, "center", "center");
        }
    }
}

// The clip options, written after the file so a sequence applies them to its first clip.
void LowerClip(Emitter& e, const Json& a) {
    EmitClause(e, a, "fps", "fps");
    EmitFlag(e, a, "loop", "loop");
    EmitFlag(e, a, "noForceLoop", "noforceloop");
    EmitClause(e, a, "startLoop", "startloop");
    if (const Json* frames = Get(a, "frames")) {
        e.Word("frame");
        e.Num(frames->at("start"));
        e.Num(frames->at("end"));
    }
    EmitClause(e, a, "numFrames", "numframes");
    EmitFlag(e, a, "reverse", "reverse");
    EmitClause(e, a, "origin", "origin");
    EmitClause(e, a, "rotate", "rotate");
    EmitClause(e, a, "angles", "angles");
    EmitClause(e, a, "scale", "scale");
    EmitClause(e, a, "motionRollback", "motionrollback");
    EmitFlag(e, a, "ignoreScale", "ignorescale");
}

// A declared slot is only a name: anything else on it would be dropped silently.
bool CheckDeclare(Emitter& e, const Json& obj) {
    for (const auto& [key, value] : obj.items())
        if (key != "name" && key != "declare" && key != "editor" && !IsInternal(key))
            return e.l.Fail(obj, key, "a declared slot takes only a name");
    return true;
}

bool LowerAnimation(Emitter& e, const Json& a) {
    if (True(a, "declare")) {
        if (!CheckDeclare(e, a)) return false;
        e.Command("$declareanimation", a);
        e.Text(Str(a, "name"));
        return true;
    }

    const bool bindPose = True(a, "bindPose");
    if (bindPose == (Get(a, "file") != nullptr))
        return e.l.Fail(a, "", "an animation takes a file, or bindPose: true");
    e.Command(bindPose ? "$bindposeanimation" : "$animation", a);
    e.Text(Str(a, "name"));
    if (!bindPose) e.Text(Str(a, "file"));
    EmitFlag(e, a, "snap", "snap");
    EmitFlag(e, a, "post", "post");
    LowerClip(e, a);
    return true;
}

bool LowerSequence(Emitter& e, const Json& s) {
    if (True(s, "declare")) {
        if (!CheckDeclare(e, s)) return false;
        e.Command("$declaresequence", s);
        e.Text(Str(s, "name"));
        return true;
    }

    const bool bindPose = True(s, "bindPose");
    const int sources = (Get(s, "file") ? 1 : 0) + (Get(s, "files") ? 1 : 0) +
                        (Get(s, "animations") ? 1 : 0) + (bindPose ? 1 : 0);
    if (sources != 1)
        return e.l.Fail(s, "", "a sequence takes one of file, files, animations or bindPose: true");

    e.Command(bindPose ? "$bindposesequence" : "$sequence", s);
    e.Text(Str(s, "name"));
    e.Word("{");
    if (const Json* file = Get(s, "file")) e.Text(file->get<std::string>());
    if (const Json* files = Get(s, "files")) EmitNames(e, *files);
    if (const Json* anims = Get(s, "animations")) EmitNames(e, *anims);

    if (const Json* activity = Get(s, "activity")) {
        e.Word("activity");
        e.Text(Str(*activity, "name"));
        if (const Json* weight = Get(*activity, "weight")) e.Num(*weight);
    }
    static const struct { const char* key; const char* word; } kFlags[] = {
        {"snap", "snap"}, {"autoplay", "autoplay"}, {"hidden", "hidden"},
        {"realtime", "realtime"}, {"post", "post"}, {"delta", "delta"},
        {"preDelta", "predelta"}, {"worldSpace", "worldspace"},
        {"worldRelative", "worldrelative"},
    };
    for (const auto& flag : kFlags) EmitFlag(e, s, flag.key, flag.word);
    EmitClause(e, s, "rootDriver", "rootdriver");
    EmitClause(e, s, "exitPhase", "exitphase");
    EmitClause(e, s, "poseCycle", "posecycle");
    EmitClause(e, s, "fadeIn", "fadein");
    EmitClause(e, s, "fadeOut", "fadeout");
    EmitClause(e, s, "blendWidth", "blendwidth");

    if (const Json* blends = Get(s, "blends")) {
        for (const Json& b : *blends) {
            e.Word("blend");
            e.Text(Str(b, "parameter"));
            e.Num(b.at("min"));
            e.Num(b.at("max"));
        }
    }
    if (const Json* layers = Get(s, "addLayers")) {
        for (const Json& layer : *layers) {
            e.Word("addlayer");
            e.Text(Str(layer, "sequence"));
            EmitFlag(e, layer, "local", "local");
        }
    }
    if (const Json* layers = Get(s, "blendLayers")) {
        for (const Json& layer : *layers) {
            e.Word("blendlayer");
            e.Text(Str(layer, "sequence"));
            for (const char* key : {"start", "peak", "tail", "end"}) e.Num(layer.at(key));
            EmitFlag(e, layer, "xfade", "xfade");
            EmitFlag(e, layer, "spline", "spline");
            EmitFlag(e, layer, "noBlend", "noblend");
            EmitFlag(e, layer, "local", "local");
            EmitClause(e, layer, "poseParameter", "poseparameter");
        }
    }
    if (const Json* locks = Get(s, "ikLocks")) {
        for (const Json& lock : *locks) {
            e.Word("iklock");
            e.Text(Str(lock, "chain"));
            e.Num(lock.at("position"));
            e.Num(lock.at("rotation"));
        }
    }
    if (const Json* tags = Get(s, "animTags")) {
        for (const Json& tag : *tags) {
            e.Word("animtag");
            e.Text(Str(tag, "name"));
            e.Num(tag.at("cycle"));
        }
    }
    EmitClause(e, s, "node", "node");
    if (const Json* t = Get(s, "transition")) {
        e.Word(True(*t, "reversible") ? "rtransition" : "transition");
        e.Text(Str(*t, "from"));
        e.Text(Str(*t, "to"));
    }
    if (const Json* mods = Get(s, "activityModifiers")) {
        for (const Json& mod : *mods) {
            e.Word("activitymodifier");
            e.Text(mod.get<std::string>());
        }
    }
    LowerClip(e, s);

    // an event reads an options string from the rest of its line, so each sits on its own
    if (const Json* events = Get(s, "events")) {
        for (const Json& ev : *events) {
            e.Line(ev);
            e.Word("event");
            e.Text(Str(ev, "name"));
            e.Num(ev.at("frame"));
            if (const Json* options = Get(ev, "options")) e.Text(options->get<std::string>());
        }
    }
    e.Word("}");
    return true;
}

bool Lower(Emitter& e, const Json& m) {
    if (const Json* dirs = Get(m, "searchDirs")) {
        for (const Json& dir : *dirs) {
            e.Command("$addsearchdir", m, "searchDirs");
            e.Text(dir.get<std::string>());
        }
    }

    const Json* settings = Get(m, "model");
    if (settings && !LowerSettings(e, *settings)) return false;

    const Json* materials = Get(m, "materials");
    if (materials) {
        if (const Json* paths = Get(*materials, "paths"); paths && !paths->empty()) {
            e.Command("$cdmaterials", *materials, "paths");
            EmitNames(e, *paths);
        }
        LowerReplacements(e, Get(*materials, "collapse"), "$collapsematerial");
    }

    if (const Json* meshes = Get(m, "meshes"))
        for (const Json& mesh : *meshes) LowerMesh(e, mesh);
    if (const Json* groups = Get(m, "modelGroups"))
        for (const Json& group : *groups)
            if (!LowerModelGroup(e, group)) return false;
    if (settings && !LowerPoints(e, *settings)) return false;

    if (materials) {
        if (const Json* skins = Get(*materials, "skins"); skins && !skins->empty()) {
            e.Command("$texturegroup", *materials, "skins");
            e.Word("{");
            for (const Json& skin : *skins) {
                e.Line(skin);
                e.Word("$set");
                e.Word("{");
                if (const Json* replace = Get(skin, "replace")) {
                    for (const Json& r : *replace) {
                        e.Line(r);
                        e.Word("material");
                        e.Text(Str(r, "material"));
                        e.Text(Str(r, "with"));
                    }
                }
                e.Word("}");
            }
            e.Word("}");
        }
        LowerReplacements(e, Get(*materials, "renames"), "$renamematerial");
        if (const Json* order = Get(*materials, "sortOrder"); order && !order->empty()) {
            e.Command("$meshsortorder", *materials, "sortOrder");
            e.Word("{");
            EmitNames(e, *order);
            e.Word("}");
        }
    }

    if (const Json* lods = Get(m, "lods"))
        for (const Json& lod : *lods)
            if (!LowerLod(e, lod)) return false;
    if (const Json* bones = Get(m, "bones")) LowerBones(e, *bones);
    if (const Json* list = Get(m, "attachments"))
        for (const Json& a : *list) LowerAttachment(e, a);
    if (const Json* list = Get(m, "hitboxSets"))
        for (const Json& set : *list) LowerHitboxSet(e, set);
    LowerIk(e, m);
    if (const Json* list = Get(m, "animations"))
        for (const Json& a : *list)
            if (!LowerAnimation(e, a)) return false;
    if (const Json* list = Get(m, "sequences"))
        for (const Json& s : *list)
            if (!LowerSequence(e, s)) return false;
    if (const Json* list = Get(m, "includeModels")) {
        for (const Json& name : *list) {
            e.Command("$includemodel", m, "includeModels");
            e.Text(name.get<std::string>());
        }
    }
    return true;
}

// The model to compile: the -only id, else the single one left to compile by default.
int SelectModel(Loader& l, const Json& doc) {
    const Json& models = doc.at("models");
    std::set<std::string> ids;
    for (size_t i = 0; i < models.size(); ++i)
        if (!ids.insert(Str(models[i], "id")).second)
            return l.Fail(0, Pointer("/models", i) + "/id", "model id \"" + Str(models[i], "id") +
                                                            "\" is used twice"), -1;

    std::string list;
    for (const std::string& id : ids) list += (list.empty() ? "" : ", ") + id;
    if (!l.options.only.empty()) {
        for (size_t i = 0; i < models.size(); ++i)
            if (Str(models[i], "id") == l.options.only) return static_cast<int>(i);
        return l.Fail(0, "/models", "no model \"" + l.options.only + "\" (models: " + list + ")"), -1;
    }

    int chosen = -1;
    for (size_t i = 0; i < models.size(); ++i) {
        if (models[i].contains("compile") && !models[i].at("compile").get<bool>()) continue;
        if (chosen >= 0)
            return l.Fail(0, "/models", "several models compile by default - pick one with "
                                        "-only <id> (models: " + list + ")"), -1;
        chosen = static_cast<int>(i);
    }
    if (chosen < 0) return l.Fail(0, "/models", "no model to compile - pick one with -only <id>"), -1;
    return chosen;
}

void SchemaFields(const Item& item, Json& out) {
    for (size_t f = 0; f < item.count; ++f) {
        const Field& field = item.fields[f];
        Json entry{{"key", field.key}, {"type", TypeName(field.type)}, {"doc", field.doc}};
        if (field.item) entry["item"] = field.item->name;
        if (field.values) {
            entry["values"] = Json::array();
            for (const char* const* v = field.values; *v; ++v) entry["values"].push_back(*v);
        }
        if (field.ref) entry["ref"] = field.ref;
        if (field.required) entry["required"] = true;
        out.push_back(std::move(entry));
    }
}

} // namespace

bool IsPulseModelPath(const char* path) {
    const std::string ext = pulse::FilePath(path).extension().string();
    return _stricmp(ext.c_str(), ".pulsemodel") == 0;
}

bool Load(const char* path, const LoadOptions& options, compile::CompileInput& out,
          std::string* err) {
    Loader l{options, err};
    const fs::path full = pulse::FilePath(path);
    if (ReadFile(l, full, false, pulse::dependencies::Kind::Script) < 0) return false;
    std::error_code ec;
    const fs::path canonical = fs::weakly_canonical(full, ec);
    l.prefabStack.push_back(ec ? full.lexically_normal().string() : canonical.string());

    const Json& doc = l.docs[0];
    const int index = SelectModel(l, doc);
    if (index < 0) return false;
    Json model = doc.at("models")[static_cast<size_t>(index)];
    model.erase("id");
    model.erase("compile");

    // shared and the model share one scope: the model may redefine a shared variable
    Scope scope(options.defvars.begin(), options.defvars.end());
    std::set<std::string> pinned;
    for (const auto& [name, value] : options.defvars) pinned.insert(name);
    const Json shared = doc.contains("shared") ? doc.at("shared") : Json();
    if (!shared.is_null() && !DefineVariables(l, shared, "variables", false, pinned, scope))
        return false;
    if (!DefineVariables(l, model, "variables", false, pinned, scope) ||
        !DefineVariables(l, model, "redefineVariables", true, pinned, scope))
        return false;

    // contributor order is list order: shared, its prefabs, the model, its prefabs
    std::vector<Json> parts;
    if (!shared.is_null() && !CollectContent(l, shared, scope, parts)) return false;
    const size_t modelPart = parts.size();
    if (!CollectContent(l, model, scope, parts)) return false;

    Json merged = Json::object();
    merged[kOrigin] = parts[modelPart].at(kOrigin);
    for (const Json& part : parts)
        if (!Merge(l, kContentItem, merged, part)) return false;

    const Json* settings = Get(merged, "model");
    if (out.outname.empty() && (!settings || !Get(*settings, "name")))
        return l.Fail(parts[modelPart], "", "the model has no model.name");

    Emitter e{l};
    if (!Lower(e, merged)) return false;

    pulse::diag::Origins& origins = pulse::diag::ScriptOrigins();
    origins.script = fs::absolute(full).lexically_normal().u8string();
    origins.lines = std::move(e.lines);
    const bool loaded = loader::LoadTokenScript(path, e.tokens, out, err, options.fileDirs);
    origins = {};
    return loaded;
}

void PrintSchema() {
    Json items = Json::object();
    for (const Item* item : kAllItems) {
        Json entry{{"fields", Json::array()}};
        if (item->base) entry["base"] = item->base->name;
        SchemaFields(*item, entry["fields"]);
        items[item->name] = std::move(entry);
    }
    const Json schema{{"format", "pulsemodel"}, {"version", kFormatVersion}, {"root", kRootItem.name},
                      {"items", std::move(items)}};
    pulse::api::Record("schema", schema.dump());
}

} // namespace pulse::pulsemodel
