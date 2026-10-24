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
    const char* declares = nullptr;      // the kind of name this field defines
    bool required = false;

    constexpr Field Of(const Item* i) const { Field f = *this; f.item = i; return f; }
    constexpr Field Values(const char* const* v) const { Field f = *this; f.values = v; return f; }
    constexpr Field Ref(const char* r) const { Field f = *this; f.ref = r; return f; }
    constexpr Field Declares(const char* d) const { Field f = *this; f.declares = d; return f; }
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
const char* const kFlexCulls[] = {"aggressive", "rules_only", "duplicates", "none", nullptr};
const char* const kTranslations[] = {"tx", "ty", "tz", nullptr};
const char* const kLids[] = {"upper", "lower", nullptr};
const char* const kPrimitives[] = {"box", "sphere", "capsule", nullptr};
const char* const kImportTypes[] = {"perjoint", "singlejoint", "singlehull", nullptr};
const char* const kJointTypes[] = {"free", "limit", "fixed", nullptr};
const char* const kIkRuleTypes[] = {"touch", "footstep", "attachment", "release", "unlatch",
                                    "autosteps", nullptr};
const char* const kAnimOps[] = {
    "reverse", "numFrames", "ikFixup", "localHierarchy", "derivative", "noAnimation",
    "linearDelta", "compress", "counterRotate", "forceBonePosRot", "weightList", "subtract",
    "fixupLoop", "walkFrame", "walkAlignTo", "walkAlign", "match", "matchBlend", "transformBone",
    "matchPosition", "copyPose", "worldSpaceBlend", "alignTo", "align", "alignBoneTo", "alignBone",
    "appendAnimation", "boneDriver", "rotateTo", nullptr};
const char* const kAnimCulls[] = {"aggressive", "duplicates", "none", nullptr};

// --- KeyValues ---

// A KeyValues block keeps its order and may repeat keys ("break" per gib), so it
// is a list of entries rather than a JSON object.
extern const Item kKeyValueItem;
const Field kKeyValue[] = {
    F("key", Type::String, "Key.").Required(),
    F("value", Type::String, "Value; omit when block is given."),
    F("block", Type::List, "Nested entries.").Of(&kKeyValueItem),
};
const Item kKeyValueItem = MakeItem("keyValue", kKeyValue);

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
    F("keyValues", Type::List, "The model's KeyValues text.").Of(&kKeyValueItem),
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
    F("name", Type::String, "Mesh name model groups refer to.").Declares("mesh").Required(),
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
    F("name", Type::String, "Group name.").Declares("modelGroup").Required(),
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
    F("name", Type::String, "Bone name.").Declares("bone").Required(),
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

// --- procedural bones ---

const Field kRange[] = {
    F("min", Type::Number, "Minimum.").Required(),
    F("max", Type::Number, "Maximum.").Required(),
};
const Item kRangeItem = MakeItem("range", kRange);

// options every jiggle section but boing takes
const Field kJiggleCommon[] = {
    F("tipMass", Type::Number, "Tip mass."),
    F("length", Type::Number, "Length."),
    F("yawFriction", Type::Number, "Yaw friction."),
    F("yawBounce", Type::Number, "Yaw bounce."),
    F("pitchFriction", Type::Number, "Pitch friction."),
    F("pitchBounce", Type::Number, "Pitch bounce."),
    F("angleConstraint", Type::Number, "Angle limit in degrees."),
    F("yawConstraint", Type::Object, "Yaw limits in degrees.").Of(&kRangeItem),
    F("pitchConstraint", Type::Object, "Pitch limits in degrees.").Of(&kRangeItem),
};
const Item kJiggleCommonItem = MakeItem("jiggleCommon", kJiggleCommon);

const Field kJiggleFlexible[] = {
    F("yawStiffness", Type::Number, "Yaw stiffness (0-1000)."),
    F("yawDamping", Type::Number, "Yaw damping (0-1000)."),
    F("pitchStiffness", Type::Number, "Pitch stiffness (0-1000)."),
    F("pitchDamping", Type::Number, "Pitch damping (0-1000)."),
    F("alongStiffness", Type::Number, "Along stiffness (0-1000)."),
    F("alongDamping", Type::Number, "Along damping (0-1000)."),
    F("allowLengthFlex", Type::Bool, "Let the length stretch."),
};
const Item kJiggleFlexibleItem = MakeItem("jiggleFlexible", kJiggleFlexible, &kJiggleCommonItem);

const Field kJiggleBaseSpring[] = {
    F("stiffness", Type::Number, "Stiffness (0-1000)."),
    F("damping", Type::Number, "Damping (0-1000)."),
    F("baseMass", Type::Number, "Base mass."),
    F("leftConstraint", Type::Object, "Left limits.").Of(&kRangeItem),
    F("leftFriction", Type::Number, "Left friction."),
    F("upConstraint", Type::Object, "Up limits.").Of(&kRangeItem),
    F("upFriction", Type::Number, "Up friction."),
    F("forwardConstraint", Type::Object, "Forward limits.").Of(&kRangeItem),
    F("forwardFriction", Type::Number, "Forward friction."),
};
const Item kJiggleBaseSpringItem = MakeItem("jiggleBaseSpring", kJiggleBaseSpring, &kJiggleCommonItem);

const Field kJiggleBoing[] = {
    F("impactSpeed", Type::Number, "Impact speed (100 when omitted)."),
    F("impactAngle", Type::Number, "Impact angle in degrees."),
    F("dampingRate", Type::Number, "Damping rate."),
    F("frequency", Type::Number, "Frequency."),
    F("amplitude", Type::Number, "Amplitude."),
};
const Item kJiggleBoingItem = MakeItem("jiggleBoing", kJiggleBoing);

const Field kJiggleBone[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("flexible", Type::Object, "Flexible behavior.").Of(&kJiggleFlexibleItem),
    F("rigid", Type::Object, "Rigid behavior.").Of(&kJiggleCommonItem),
    F("baseSpring", Type::Object, "Base spring behavior.").Of(&kJiggleBaseSpringItem),
    F("boing", Type::Object, "Boing behavior.").Of(&kJiggleBoingItem),
};
const Item kJiggleBoneItem = MakeItem("jiggleBone", kJiggleBone);

const Field kDriverTrigger[] = {
    F("tolerance", Type::Number, "Tolerance in degrees.").Required(),
    F("driverAngles", Type::Vec3, "Driver rotation (roll pitch yaw degrees)."),
    F("helperAngles", Type::Vec3, "Helper rotation (roll pitch yaw degrees)."),
    F("position", Type::Vec3, "Helper position."),
    F("frame", Type::Integer, "Sample both poses from this frame of poseAnimation instead."),
};
const Item kDriverTriggerItem = MakeItem("driverTrigger", kDriverTrigger);

const Field kDriverBone[] = {
    F("bone", Type::String, "Helper bone the engine drives.").Ref("bone").Required(),
    F("driver", Type::String, "Bone it follows.").Ref("bone").Required(),
    F("relative", Type::Bool, "Trigger poses are deltas from the bind pose."),
    F("poseAnimation", Type::File, "Animation the frame triggers sample."),
    F("basePos", Type::Vec3, "Offset added to every trigger position."),
    F("triggers", Type::List, "Triggers.").Of(&kDriverTriggerItem).Required(),
};
const Item kDriverBoneItem = MakeItem("driverBone", kDriverBone);

const Field kAimAt[] = {
    F("bone", Type::String, "Bone that aims.").Ref("bone").Required(),
    F("target", Type::String, "Attachment, else bone, it aims at.").Required(),
    F("up", Type::Vec3, "Up vector.").Required(),
    F("aim", Type::Vec3, "Aim vector.").Required(),
};
const Item kAimAtItem = MakeItem("aimAtBone", kAimAt);

const Field kHierarchy[] = {
    F("bone", Type::String, "Bone to reparent.").Ref("bone").Required(),
    F("parent", Type::String, "New parent; omitted = root bone.").Ref("bone"),
};
const Item kHierarchyItem = MakeItem("hierarchy", kHierarchy);

const Field kBones[] = {
    F("define", Type::List, "Bind-pose bones.").Of(&kDefineBoneItem),
    F("boneMerge", Type::Strings, "Bones kept for bonemerge.").Ref("bone"),
    F("doNotCollapse", Type::Strings, "Bones never collapsed.").Ref("bone"),
    F("alwaysCollapse", Type::Strings, "Bones always collapsed.").Ref("bone"),
    F("renames", Type::List, "Rename bones in the output.").Of(&kRenameItem),
    F("transforms", Type::List, "Bind-pose edits.").Of(&kTransformBoneItem),
    F("hierarchy", Type::List, "Reparented bones.").Of(&kHierarchyItem),
    F("root", Type::String, "Motion root bone.").Ref("bone"),
    F("cullMethod", Type::Enum, "Bone cull method.").Values(kBoneCulls),
    F("realign", Type::Bool, "Realign single-child chains."),
    F("lockLengths", Type::Bool, "Pin bone lengths to the bind pose."),
    F("jiggle", Type::List, "Jiggle bones.").Of(&kJiggleBoneItem),
    F("cullJiggle", Type::Bool, "Drop jiggle bones nothing is weighted to."),
    F("drivers", Type::List, "Driver (quaternion interpolation) bones.").Of(&kDriverBoneItem),
    F("aimAt", Type::List, "Aim-at bones.").Of(&kAimAtItem),
    F("vrdFiles", Type::Files, "Procedural bones from .vrd files."),
};
const Item kBonesItem = MakeItem("bones", kBones);

const Field kTriggerEdit[] = {
    F("bone", Type::String, "Procedural bone from this file.").Ref("bone").Required(),
    F("angle", Type::Number, "New angle of influence in degrees; omit with remove."),
    F("remove", Type::Bool, "Remove the triggers instead."),
    F("frame", Type::Integer, "Trigger index."),
    F("frames", Type::Object, "Trigger index range.").Of(&kRangeItem),
};
const Item kTriggerEditItem = MakeItem("triggerEdit", kTriggerEdit);

const Field kDataModelJoints[] = {
    F("file", Type::File, "DMX holding the bone markup.").Required(),
    F("jiggleBones", Type::Bool, "Import its jiggle bones."),
    F("proceduralBones", Type::Bool, "Import its procedural bones."),
    F("hitboxes", Type::Bool, "Import its hitboxes."),
    F("attachments", Type::Bool, "Import its attachments."),
    F("triggerEdits", Type::List, "Changes to imported procedural bone triggers.").Of(&kTriggerEditItem),
};
const Item kDataModelJointsItem = MakeItem("dataModelJoints", kDataModelJoints);

// --- attachments, hitboxes, pose parameters, IK ---

const Field kAttachment[] = {
    F("name", Type::String, "Attachment name.").Declares("attachment").Required(),
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
    F("name", Type::String, "Set name.").Declares("hitboxSet").Required(),
    F("boxes", Type::List, "Hitboxes.").Of(&kHitboxItem),
};
const Item kHitboxSetItem = MakeItem("hitboxSet", kHitboxSet);

const Field kPoseParameter[] = {
    F("name", Type::String, "Parameter name.").Declares("poseParameter").Required(),
    F("min", Type::Number, "Start value.").Required(),
    F("max", Type::Number, "End value.").Required(),
    F("wrap", Type::Bool, "Wraps over the full range."),
    F("loop", Type::Number, "Loops at this value."),
};
const Item kPoseParameterItem = MakeItem("poseParameter", kPoseParameter);

const Field kIkChain[] = {
    F("name", Type::String, "Chain name.").Declares("ikChain").Required(),
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

const Field kFrameRamp[] = {
    F("start", Type::Integer, "Start frame (-1 = unset).").Required(),
    F("peak", Type::Integer, "Peak frame (-1 = unset).").Required(),
    F("tail", Type::Integer, "Tail frame (-1 = unset).").Required(),
    F("end", Type::Integer, "End frame (-1 = unset).").Required(),
};
const Item kFrameRampItem = MakeItem("frameRamp", kFrameRamp);

const Field kIkRule[] = {
    F("chain", Type::String, "IK chain.").Ref("ikChain").Required(),
    F("type", Type::Enum, "Rule type.").Values(kIkRuleTypes).Required(),
    F("bone", Type::String, "touch: the bone touched; autosteps: the foot bone.").Ref("bone"),
    F("attachment", Type::String, "attachment: the attachment.").Ref("attachment"),
    F("steps", Type::Integer, "autosteps: step count."),
    F("height", Type::Number, "Height."),
    F("target", Type::Integer, "Target slot."),
    F("floor", Type::Number, "Floor."),
    F("pad", Type::Number, "Pad (sets radius to half)."),
    F("radius", Type::Number, "Radius."),
    F("contact", Type::Integer, "Contact frame."),
    F("range", Type::Object, "Ramp frames.").Of(&kFrameRampItem),
    F("useSequence", Type::Bool, "Use the sequence."),
    F("useSource", Type::Bool, "Use the source."),
    F("fakeOrigin", Type::Vec3, "Fake origin."),
    F("fakeRotate", Type::Vec3, "Fake rotation."),
};
const Item kIkRuleItem = MakeItem("ikRule", kIkRule);

// One ordered clip operation. `op` picks it; each takes only the keys its doc names.
const Field kAnimOp[] = {
    F("op", Type::Enum, "Operation.").Values(kAnimOps).Required(),
    F("animation", Type::String, "subtract, match, matchBlend, walkAlign(To), worldSpaceBlend, "
                                 "align*, appendAnimation: the reference clip.").Ref("animation"),
    F("source", Type::String, "copyPose: animation, sequence or \"bindpose\"."),
    F("bone", Type::String, "localHierarchy, counterRotate, forceBonePosRot, transformBone, "
                            "matchPosition, alignBone(To), boneDriver: the bone.").Ref("bone"),
    F("parent", Type::String, "localHierarchy: new parent (\"\" = world).").Ref("bone"),
    F("weightList", Type::String, "weightList: list name.").Ref("weightList"),
    F("ikRule", Type::Object, "ikFixup: the rule baked in.").Of(&kIkRuleItem),
    F("count", Type::Integer, "numFrames: frame count; compress: frame skip."),
    F("frame", Type::Integer, "subtract: reference frame; walk*: end frame; worldSpaceBlend: loop start."),
    F("start", Type::Integer, "fixupLoop, copyPose: start frame."),
    F("end", Type::Integer, "fixupLoop, copyPose: end frame."),
    F("sourceFrame", Type::Integer, "matchBlend, walkAlign, copyPose, align, alignBone: source frame."),
    F("destinationFrame", Type::Integer, "matchBlend, align, alignBone: destination frame; walkAlign: reference frame."),
    F("preFrames", Type::Integer, "matchBlend: frames blended in before."),
    F("postFrames", Type::Integer, "matchBlend: frames blended in after."),
    F("scale", Type::Number, "derivative: scale."),
    F("angle", Type::Number, "rotateTo: yaw in degrees."),
    F("value", Type::Number, "boneDriver: value."),
    F("fadeIn", Type::Number, "copyPose: fade-in frames."),
    F("fadeOut", Type::Number, "copyPose: fade-out frames."),
    F("controls", Type::Strings, "walk*, align, alignBone: motion controls (X Y Z XR YR ZR ...)."),
    F("axes", Type::Strings, "matchPosition: axes (x y z); omitted = all."),
    F("axis", Type::Enum, "boneDriver: axis.").Values(kAxes),
    F("origin", Type::Vec3, "transformBone: offset."),
    F("angles", Type::Vec3, "counterRotate: explicit target; transformBone: rotation."),
    F("position", Type::Vec3, "forceBonePosRot: position."),
    F("rotation", Type::Vec3, "forceBonePosRot: rotation."),
    F("range", Type::Object, "localHierarchy, boneDriver: ramp frames.").Of(&kFrameRampItem),
    F("local", Type::Bool, "forceBonePosRot: rotation is local."),
    F("keepDuration", Type::Bool, "noAnimation: keep the frame count."),
    F("spline", Type::Bool, "linearDelta: spline ease."),
    F("pre", Type::Bool, "subtract: presubtract."),
    F("loop", Type::Bool, "worldSpaceBlend: sample by cycle from frame."),
    F("copyPosition", Type::Bool, "copyPose: copy position (both when neither is set)."),
    F("copyRotation", Type::Bool, "copyPose: copy rotation."),
};
const Item kAnimOpItem = MakeItem("animationOperation", kAnimOp);

// options shared by animations and a sequence's first clip
const Field kClip[] = {
    F("fps", Type::Number, "Frame rate."),
    F("loop", Type::Bool, "Looping."),
    F("noForceLoop", Type::Bool, "Do not force the loop."),
    F("startLoop", Type::Integer, "Loop restart frame."),
    F("percentStartLoop", Type::Number, "Loop restart as a percentage."),
    F("fudgeLoop", Type::Bool, "Fudge the loop."),
    F("frames", Type::Object, "Frame range.").Of(&kFramesItem),
    F("frameStart", Type::Integer, "First frame; the end stays the clip's."),
    F("blockName", Type::String, "Clip of a multi-clip source."),
    F("origin", Type::Vec3, "Clip translation."),
    F("rotate", Type::Number, "Clip yaw in degrees."),
    F("angles", Type::Vec3, "Clip rotation in degrees."),
    F("scale", Type::Number, "Clip scale."),
    F("motion", Type::Strings, "Motion extraction controls (X Y Z XR YR ZR LX LY LZ LXR LYR LZR LM LQ)."),
    F("motionRollback", Type::Number, "Motion rollback in seconds."),
    F("ignoreScale", Type::Bool, "Ignore $transformmodel scale."),
    F("ignoreTransformBone", Type::Strings, "Undo bone transforms here: angles, position."),
    F("noAutoIk", Type::Bool, "No automatic IK release rules."),
    F("noCull", Type::Bool, "Never culled by the animation cull method."),
    F("noAnimBlock", Type::Bool, "Keep out of the demand-loaded .ani."),
    F("noAnimBlockStall", Type::Bool, "First section stays local."),
    F("noStallFrames", Type::Number, "Frames that never stall."),
    F("ikRules", Type::List, "IK rules.").Of(&kIkRuleItem),
    F("operations", Type::List, "Clip operations, applied in order.").Of(&kAnimOpItem),
};
const Item kClipItem = MakeItem("clip", kClip);

const Field kAnimation[] = {
    F("name", Type::String, "Animation name.").Declares("animation").Required(),
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
    F("name", Type::String, "Sequence name.").Declares("sequence").Required(),
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
    F("keyValues", Type::List, "The sequence's KeyValues text.").Of(&kKeyValueItem),
};
const Item kSequenceItem = MakeItem("sequence", kSequence, &kClipItem);

// --- flex and face ---

const Field kDataModelFlexes[] = {
    F("file", Type::File, "DMX holding the flex rig.").Required(),
    F("controllers", Type::Bool, "Import its flex controllers."),
    F("correctives", Type::Bool, "Import its correctives (needs controllers)."),
    F("dominators", Type::Bool, "Import its domination rules (needs correctives)."),
    F("rules", Type::Bool, "Import its flex rules."),
};
const Item kDataModelFlexesItem = MakeItem("dataModelFlexes", kDataModelFlexes);

const Field kFlexController[] = {
    F("name", Type::String, "Controller name.").Declares("flexController").Required(),
    F("group", Type::String, "Controller group.").Required(),
    F("min", Type::Number, "Range start (0 when omitted)."),
    F("max", Type::Number, "Range end (1 when omitted)."),
};
const Item kFlexControllerItem = MakeItem("flexController", kFlexController);

const Field kFlexRule[] = {
    F("name", Type::String, "Morph or local variable the rule drives.").Ref("morph").Required(),
    F("expression", Type::String, "Rule expression: controller names, %morph, numbers, operators.").Required(),
};
const Item kFlexRuleItem = MakeItem("flexRule", kFlexRule);

const Field kFlexCorrective[] = {
    F("name", Type::String, "Corrective morph.").Ref("morph").Required(),
    F("controls", Type::Strings, "Controls that must all be up: controller, or %morph.").Required(),
};
const Item kFlexCorrectiveItem = MakeItem("flexCorrective", kFlexCorrective);

const Field kFlexDomination[] = {
    F("name", Type::String, "Morph that is suppressed.").Ref("morph").Required(),
    F("dominators", Type::Strings, "Controls suppressing it: controller, or %morph.").Required(),
};
const Item kFlexDominationItem = MakeItem("flexDomination", kFlexDomination);

const Field kStereoSplit[] = {
    F("morph", Type::String, "Morph split into <name>L/<name>R.").Ref("morph").Required(),
    F("splitFactor", Type::Number, "Balance from vertex X when the mesh has none; 0 = mesh balance."),
};
const Item kStereoSplitItem = MakeItem("stereoSplit", kStereoSplit);

const Field kMorphRename[] = {
    F("morph", Type::String, "Source morph.").Ref("morph").Required(),
    F("to", Type::String, "New name.").Required(),
};
const Item kMorphRenameItem = MakeItem("morphRename", kMorphRename);

const Field kMorphValue[] = {
    F("morph", Type::String, "Morph.").Ref("morph").Required(),
    F("value", Type::Number, "Amount.").Required(),
};
const Item kMorphValueItem = MakeItem("morphValue", kMorphValue);

const Field kBoneFlexDriver[] = {
    F("bone", Type::String, "Driving bone.").Ref("bone").Required(),
    F("component", Type::Enum, "Translation axis.").Values(kTranslations).Required(),
    F("controller", Type::String, "Driven flex controller.").Ref("flexController").Required(),
    F("min", Type::Number, "Translation at controller 0.").Required(),
    F("max", Type::Number, "Translation at controller 1.").Required(),
};
const Item kBoneFlexDriverItem = MakeItem("boneFlexDriver", kBoneFlexDriver);

const Field kFlex[] = {
    F("dataModel", Type::List, "Flex rigs imported from DMX files.").Of(&kDataModelFlexesItem),
    F("controllers", Type::List, "Flex controllers.").Of(&kFlexControllerItem),
    F("localVars", Type::Strings, "Flex rule variables that drive no geometry."),
    F("rules", Type::List, "Flex rules.").Of(&kFlexRuleItem),
    F("correctives", Type::List, "Correctives.").Of(&kFlexCorrectiveItem),
    F("dominations", Type::List, "Domination rules.").Of(&kFlexDominationItem),
    F("stereoSplits", Type::List, "Morphs split into left/right halves.").Of(&kStereoSplitItem),
    F("renames", Type::List, "Morph renames.").Of(&kMorphRenameItem),
    F("fixed", Type::List, "Morph amounts baked into the rest mesh (0-1).").Of(&kMorphValueItem),
    F("wrinkleScales", Type::List, "Wrinkle scale per morph.").Of(&kMorphValueItem),
    F("boneDrivers", Type::List, "Bone translations driving controllers.").Of(&kBoneFlexDriverItem),
    F("cullMethod", Type::Enum, "How hard unused flex data is pruned.").Values(kFlexCulls),
};
const Item kFlexItem = MakeItem("flex", kFlex);

const Field kEyeball[] = {
    F("name", Type::String, "Eyeball name.").Declares("eyeball").Required(),
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("material", Type::String, "Eyeball material.").Ref("material").Required(),
    F("origin", Type::Vec3, "Center; with center, an offset from it."),
    F("center", Type::Bool, "Center on the material's vertices."),
    F("diameter", Type::Number, "Diameter."),
    F("angle", Type::Number, "Angle in degrees."),
    F("pupilScale", Type::Number, "Pupil scale."),
};
const Item kEyeballItem = MakeItem("eyeball", kEyeball);

const Field kMouth[] = {
    F("controller", Type::String, "Flex controller.").Ref("flexController").Required(),
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("forward", Type::Vec3, "Forward direction.").Required(),
};
const Item kMouthItem = MakeItem("mouth", kMouth);

const Field kLidPose[] = {
    F("delta", Type::String, "Delta state; omitted = no vertex data.").Ref("morph"),
    F("target", Type::Number, "Lid target.").Required(),
};
const Item kLidPoseItem = MakeItem("lidPose", kLidPose);

const Field kEyelid[] = {
    F("type", Type::Enum, "Lid.").Values(kLids).Required(),
    F("lowerer", Type::Object, "Lowered pose.").Of(&kLidPoseItem).Required(),
    F("neutral", Type::Object, "Neutral pose.").Of(&kLidPoseItem).Required(),
    F("raiser", Type::Object, "Raised pose.").Of(&kLidPoseItem).Required(),
    F("split", Type::Number, "Mask deltas to one side; sign picks the side."),
    F("rightEyeball", Type::String, "Right eyeball (paired form).").Ref("eyeball"),
    F("leftEyeball", Type::String, "Left eyeball (paired form).").Ref("eyeball"),
    F("eyeball", Type::String, "One eyeball (per-eye form, needs flexDesc).").Ref("eyeball"),
    F("flexDesc", Type::String, "Lid flexdesc name (per-eye form)."),
};
const Item kEyelidItem = MakeItem("eyelid", kEyelid);

const Field kFace[] = {
    F("eyeballs", Type::List, "Eyeballs.").Of(&kEyeballItem),
    F("mouths", Type::List, "Mouths, in index order.").Of(&kMouthItem),
    F("eyelids", Type::List, "Eyelids.").Of(&kEyelidItem),
};
const Item kFaceItem = MakeItem("face", kFace);

// --- physics ---

const Field kPrimitive[] = {
    F("type", Type::Enum, "Primitive shape.").Values(kPrimitives).Required(),
    F("bone", Type::String, "Bone the hull is on; values are in its space.").Ref("bone").Required(),
    F("min", Type::Vec3, "Box minimum."),
    F("max", Type::Vec3, "Box maximum."),
    F("angles", Type::Vec3, "Box orientation."),
    F("center", Type::Vec3, "Sphere center."),
    F("radius", Type::Number, "Sphere radius."),
    F("point0", Type::Vec3, "Capsule start."),
    F("point1", Type::Vec3, "Capsule end."),
    F("radius0", Type::Number, "Capsule start radius."),
    F("radius1", Type::Number, "Capsule end radius."),
    F("segments", Type::Integer, "Sphere/capsule tessellation."),
};
const Item kPrimitiveItem = MakeItem("physicsPrimitive", kPrimitive);

const Field kPhysicsShape[] = {
    F("file", Type::File, "Collision source file."),
    F("renderMesh", Type::String, "Generate from this render mesh.").Ref("mesh"),
    F("fromRender", Type::Bool, "Generate from the whole render model."),
    F("primitive", Type::Object, "A box, sphere or capsule hull.").Of(&kPrimitiveItem),
    F("offsetOrigin", Type::Vec3, "Translation."),
    F("offsetAngles", Type::Vec3, "Rotation."),
    F("importScale", Type::Number, "Scale."),
    F("parentBone", Type::String, "Bone the shape belongs to.").Ref("bone"),
    F("maxConvex", Type::Integer, "Convex piece limit."),
    F("importType", Type::Enum, "File shapes: how parts are bound to bones.").Values(kImportTypes),
    F("concave", Type::Bool, "File shapes: keep concave pieces."),
    F("remove2d", Type::Bool, "File shapes: drop flat pieces."),
    F("decimationFactor", Type::Number, "Generated shapes: decimation."),
    F("cullWeight", Type::Number, "Generated shapes: bone weight cull (0-1)."),
    F("concavity", Type::Number, "Generated shapes: concavity."),
    F("maxHulls", Type::Integer, "Generated shapes: hull limit."),
    F("maxDepth", Type::Integer, "Generated shapes: split depth (0-10)."),
    F("extraBones", Type::Strings, "Generated shapes: more bones to include.").Ref("bone"),
    F("excludeMeshes", Type::Strings, "Whole-model generation: render meshes left out.").Ref("mesh"),
};
const Item kPhysicsShapeItem = MakeItem("physicsShape", kPhysicsShape);

const Field kJointAxis[] = {
    F("axis", Type::Enum, "Axis.").Values(kAxes).Required(),
    F("type", Type::Enum, "Motion.").Values(kJointTypes).Required(),
    F("min", Type::Number, "Limit minimum in degrees."),
    F("max", Type::Number, "Limit maximum in degrees."),
    F("friction", Type::Number, "Friction (1 when omitted)."),
};
const Item kJointAxisItem = MakeItem("jointAxis", kJointAxis);

const Field kPhysicsJoint[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("axes", Type::List, "Axes; an axis left out is locked.").Of(&kJointAxisItem).Required(),
};
const Item kPhysicsJointItem = MakeItem("physicsJoint", kPhysicsJoint);

const Field kPhysicsMarkup[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("massBias", Type::Number, "Mass bias."),
    F("inertia", Type::Number, "Inertia."),
    F("damping", Type::Number, "Damping."),
    F("rotDamping", Type::Number, "Rotation damping."),
    F("skip", Type::Bool, "No body for this bone."),
    F("mergeInto", Type::String, "Merge this body into another bone's.").Ref("bone"),
};
const Item kPhysicsMarkupItem = MakeItem("physicsMarkup", kPhysicsMarkup);

const Field kBonePair[] = {
    F("a", Type::String, "First bone.").Ref("bone").Required(),
    F("b", Type::String, "Second bone.").Ref("bone").Required(),
};
const Item kBonePairItem = MakeItem("bonePair", kBonePair);

const Field kAnimatedFriction[] = {
    F("min", Type::Integer, "Minimum friction.").Required(),
    F("max", Type::Integer, "Maximum friction.").Required(),
    F("timeIn", Type::Number, "Ramp-in time.").Required(),
    F("timeOut", Type::Number, "Ramp-out time.").Required(),
    F("timeHold", Type::Number, "Hold time.").Required(),
};
const Item kAnimatedFrictionItem = MakeItem("animatedFriction", kAnimatedFriction);

const Field kJointContents[] = {
    F("bone", Type::String, "Bone; its children inherit.").Ref("bone").Required(),
    F("contents", Type::Strings, "Contents words, as model.contents.").Required(),
};
const Item kJointContentsItem = MakeItem("jointContents", kJointContents);

const Field kJointSurfaceProp[] = {
    F("bone", Type::String, "Bone; its children inherit.").Ref("bone").Required(),
    F("surfaceProp", Type::String, "Surface property.").Required(),
};
const Item kJointSurfacePropItem = MakeItem("jointSurfaceProp", kJointSurfaceProp);

const Field kPhysics[] = {
    F("dataModel", Type::File, "DMX whose physics shapes are imported."),
    F("shapes", Type::List, "Collision shapes.").Of(&kPhysicsShapeItem),
    F("joints", Type::List, "Ragdoll joints, one entry per bone.").Of(&kPhysicsJointItem),
    F("markups", Type::List, "Per-body overrides, one entry per bone.").Of(&kPhysicsMarkupItem),
    F("collisions", Type::List, "Body pairs that collide.").Of(&kBonePairItem),
    F("mass", Type::Number, "Mass in kg."),
    F("autoMass", Type::Bool, "Mass from volume."),
    F("massCenter", Type::Vec3, "Center of mass."),
    F("rootBone", Type::String, "Root body bone.").Ref("bone"),
    F("damping", Type::Number, "Damping."),
    F("rotDamping", Type::Number, "Rotation damping."),
    F("inertia", Type::Number, "Inertia."),
    F("drag", Type::Number, "Drag."),
    F("weldPosition", Type::Number, "Position weld epsilon."),
    F("weldNormal", Type::Number, "Normal weld epsilon."),
    F("noSelfCollisions", Type::Bool, "Bodies never collide with each other."),
    F("assumeWorldSpace", Type::Bool, "Source is already in world space."),
    F("phyName", Type::String, "Name of the .phy, when not the model's."),
    F("animatedFriction", Type::Object, "Animated friction.").Of(&kAnimatedFrictionItem),
    F("jointContents", Type::List, "Trace contents per bone.").Of(&kJointContentsItem),
    F("jointSurfaceProps", Type::List, "Surface property per bone.").Of(&kJointSurfacePropItem),
    F("collisionText", Type::List, "The .phy's KeyValues text (gibs, break pieces, ...).").Of(&kKeyValueItem),
};
const Item kPhysicsItem = MakeItem("physics", kPhysics);

// --- model-wide animation settings ---

const Field kWeight[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("weight", Type::Number, "Weight.").Required(),
    F("posWeight", Type::Number, "Position weight (the weight when omitted)."),
};
const Item kWeightItem = MakeItem("weight", kWeight);

const Field kWeightList[] = {
    F("name", Type::String, "List name.").Declares("weightList").Required(),
    F("bones", Type::List, "Bone weights.").Of(&kWeightItem).Required(),
};
const Item kWeightListItem = MakeItem("weightList", kWeightList);

const Field kSectionFrames[] = {
    F("frames", Type::Integer, "Section length in frames.").Required(),
    F("minLimit", Type::Integer, "Minimum frame limit.").Required(),
};
const Item kSectionFramesItem = MakeItem("sectionFrames", kSectionFrames);

const Field kAnimBlocks[] = {
    F("size", Type::Integer, "Block size; under 1024 reads as kilobytes.").Required(),
    F("noStall", Type::Bool, "No stall."),
    F("highRes", Type::Bool, "High resolution cached frames."),
    F("lowRes", Type::Bool, "Low resolution cached frames."),
    F("zeroFrames", Type::Integer, "Cached frame count (1-4)."),
    F("cacheHighRes", Type::Bool, "Cache high resolution."),
    F("posDelta", Type::Number, "Position delta."),
};
const Item kAnimBlocksItem = MakeItem("animBlocks", kAnimBlocks);

const Field kBoneSaveFrame[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("position", Type::Bool, "Save position."),
    F("rotation", Type::Bool, "Save rotation."),
    F("rotation64", Type::Bool, "Save 64-bit rotation."),
};
const Item kBoneSaveFrameItem = MakeItem("boneSaveFrame", kBoneSaveFrame);

const Field kSkipTransition[] = {
    F("nodes", Type::Strings, "Nodes routed directly between each other.").Required(),
};
const Item kSkipTransitionItem = MakeItem("skipTransition", kSkipTransition);

const Field kAnimationSettings[] = {
    F("defaultFps", Type::Number, "Frame rate of new clips."),
    F("motionRollback", Type::Number, "Motion rollback of new clips."),
    F("defaultFadeIn", Type::Number, "Fade-in of new sequences."),
    F("defaultFadeOut", Type::Number, "Fade-out of new sequences."),
    F("lowercaseSequences", Type::Bool, "Lowercase sequence names."),
    F("allowedActivities", Type::Strings, "Only these activities may be used."),
    F("sectionFrames", Type::Object, "Compression sectioning.").Of(&kSectionFramesItem),
    F("animBlocks", Type::Object, "Demand-loaded animation (.ani).").Of(&kAnimBlocksItem),
    F("cullMethod", Type::Enum, "Which animations reach the .mdl.").Values(kAnimCulls),
    F("defaultWeights", Type::List, "The default weight list.").Of(&kWeightItem),
    F("autoplayLocks", Type::List, "IK locks on autoplay sequences.").Of(&kIkLockItem),
    F("boneSaveFrames", Type::List, "Bones cached for demand loading.").Of(&kBoneSaveFrameItem),
    F("skipTransitions", Type::List, "Direct transition routes.").Of(&kSkipTransitionItem),
    F("calcTransitions", Type::Bool, "Fill multi-stage transition routes."),
};
const Item kAnimationSettingsItem = MakeItem("animationSettings", kAnimationSettings);

const Field kPresetChoice[] = {
    F("group", Type::String, "Model group.").Ref("modelGroup").Required(),
    F("choice", Type::Integer, "Choice index.").Required(),
};
const Item kPresetChoiceItem = MakeItem("presetChoice", kPresetChoice);

const Field kModelGroupPreset[] = {
    F("name", Type::String, "Preset name.").Declares("modelGroupPreset").Required(),
    F("choices", Type::List, "Group choices.").Of(&kPresetChoiceItem).Required(),
};
const Item kModelGroupPresetItem = MakeItem("modelGroupPreset", kModelGroupPreset);

const Field kHitGroup[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("group", Type::Integer, "Hit group of its automatic hitbox.").Required(),
};
const Item kHitGroupItem = MakeItem("hitGroup", kHitGroup);

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
    F("modelGroupPresets", Type::List, "Named model group combinations.").Of(&kModelGroupPresetItem),
    F("lods", Type::List, "LODs, in switch order.").Of(&kLodItem),
    F("flex", Type::Object, "Flex controllers, rules and morph edits.").Of(&kFlexItem),
    F("face", Type::Object, "Eyeballs, mouths and eyelids.").Of(&kFaceItem),
    F("bones", Type::Object, "Skeleton edits.").Of(&kBonesItem),
    F("dataModelJoints", Type::List, "Bone markup imported from DMX files.").Of(&kDataModelJointsItem),
    F("attachments", Type::List, "Attachments.").Of(&kAttachmentItem),
    F("hitboxSets", Type::List, "Hitbox sets.").Of(&kHitboxSetItem),
    F("hitGroups", Type::List, "Hit groups of automatic hitboxes.").Of(&kHitGroupItem),
    F("physics", Type::Object, "Collision model and ragdoll.").Of(&kPhysicsItem),
    F("poseParameters", Type::List, "Pose parameters.").Of(&kPoseParameterItem),
    F("ikChains", Type::List, "IK chains.").Of(&kIkChainItem),
    F("animationSettings", Type::Object, "Model-wide animation settings.").Of(&kAnimationSettingsItem),
    F("weightLists", Type::List, "Named bone weight lists.").Of(&kWeightListItem),
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
    &kTransitionItem, &kFlexItem, &kDataModelFlexesItem, &kFlexControllerItem, &kFlexRuleItem,
    &kFlexCorrectiveItem, &kFlexDominationItem, &kStereoSplitItem, &kMorphRenameItem,
    &kMorphValueItem, &kBoneFlexDriverItem, &kFaceItem, &kEyeballItem, &kMouthItem,
    &kLidPoseItem, &kEyelidItem, &kKeyValueItem, &kPhysicsItem, &kPrimitiveItem,
    &kPhysicsShapeItem, &kJointAxisItem, &kPhysicsJointItem, &kPhysicsMarkupItem, &kBonePairItem,
    &kAnimatedFrictionItem, &kJointContentsItem, &kJointSurfacePropItem, &kRangeItem,
    &kJiggleCommonItem, &kJiggleFlexibleItem, &kJiggleBaseSpringItem, &kJiggleBoingItem,
    &kJiggleBoneItem, &kDriverTriggerItem, &kDriverBoneItem, &kAimAtItem, &kHierarchyItem,
    &kTriggerEditItem, &kDataModelJointsItem, &kFrameRampItem, &kIkRuleItem, &kAnimOpItem,
    &kWeightItem, &kWeightListItem, &kSectionFramesItem, &kAnimBlocksItem, &kBoneSaveFrameItem,
    &kSkipTransitionItem, &kAnimationSettingsItem, &kPresetChoiceItem, &kModelGroupPresetItem,
    &kHitGroupItem,
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

bool EmitKeyValues(Emitter& e, const Json& entries);

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
    if (const Json* text = Get(m, "keyValues")) {
        e.Command("$keyvalues", m, "keyValues");
        if (!EmitKeyValues(e, *text)) return false;
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

// `{ key value key { ... } }`, entries in order.
bool EmitKeyValues(Emitter& e, const Json& entries) {
    e.Word("{");
    for (const Json& entry : entries) {
        const Json* value = Get(entry, "value");
        const Json* block = Get(entry, "block");
        if ((value != nullptr) == (block != nullptr))
            return e.l.Fail(entry, "", "a KeyValues entry takes a value or a block");
        e.Text(Str(entry, "key"));
        if (value) e.Text(value->get<std::string>());
        else if (!EmitKeyValues(e, *block)) return false;
    }
    e.Word("}");
    return true;
}

bool LowerPhysics(Emitter& e, const Json& p) {
    if (const Json* file = Get(p, "dataModel")) {
        e.Command("$datamodelphysics", p, "dataModel");
        e.Text(file->get<std::string>());
    }

    e.Command("$physicsmodel", p);
    e.Word("{");
    if (const Json* shapes = Get(p, "shapes")) {
        for (const Json& s : *shapes) {
            const Json* file = Get(s, "file");
            const Json* mesh = Get(s, "renderMesh");
            const Json* primitive = Get(s, "primitive");
            const bool render = True(s, "fromRender");
            if ((file ? 1 : 0) + (mesh ? 1 : 0) + (primitive ? 1 : 0) + (render ? 1 : 0) != 1)
                return e.l.Fail(s, "", "a shape takes one of file, renderMesh, fromRender or primitive");

            e.Line(s);
            e.Word("$physicsshape");
            if (primitive) {
                const Json& q = *primitive;
                const std::string type = Str(q, "type");
                e.Word("primitive");
                e.Word(type);
                e.Text(Str(q, "bone"));
                const char* const* keys = nullptr;
                static const char* const kBox[] = {"min", "max", "angles", nullptr};
                static const char* const kSphere[] = {"center", "radius", "segments", nullptr};
                static const char* const kCapsule[] = {"point0", "point1", "radius0", "radius1", "segments", nullptr};
                keys = type == "box" ? kBox : type == "sphere" ? kSphere : kCapsule;
                // the values are positional; only the last ones are optional
                for (const char* const* key = keys; *key; ++key) {
                    const Json* v = Get(q, *key);
                    const bool optional = std::string(*key) == "angles" || std::string(*key) == "segments";
                    if (!v && optional) break;
                    if (!v) return e.l.Fail(q, "", "a " + type + " needs " + *key);
                    if (v->is_array()) e.Vec(*v);
                    else e.Num(*v);
                }
                continue;
            }
            e.Word(file ? "fromfile" : mesh ? "fromrendermesh" : "fromrender");
            if (file || mesh) e.Text((file ? file : mesh)->get<std::string>());

            static const struct { const char* key; const char* word; } kOptions[] = {
                {"offsetOrigin", "offsetorigin"}, {"offsetAngles", "offsetangles"},
                {"importScale", "importscale"}, {"parentBone", "parentbone"},
                {"maxConvex", "maxconvex"}, {"importType", "importtype"},
                {"decimationFactor", "decimationfactor"}, {"cullWeight", "cullweight"},
                {"concavity", "concavity"}, {"maxHulls", "maxhulls"}, {"maxDepth", "maxdepth"},
            };
            e.Word("{");
            for (const auto& option : kOptions) EmitClause(e, s, option.key, option.word);
            EmitFlag(e, s, "concave", "concave");
            EmitFlag(e, s, "remove2d", "remove2d");
            for (const auto& [key, word] : {std::pair{"extraBones", "extrabone"}, std::pair{"excludeMeshes", "excludemesh"}})
                if (const Json* names = Get(s, key))
                    for (const Json& name : *names) {
                        e.Word(word);
                        e.Text(name.get<std::string>());
                    }
            e.Word("}");
        }
    }

    if (const Json* joints = Get(p, "joints")) {
        for (const Json& j : *joints) {
            e.Line(j);
            e.Word("$physicsjoint");
            e.Text(Str(j, "bone"));
            e.Word("{");
            for (const Json& a : j.at("axes")) {
                const std::string type = Str(a, "type");
                e.Word(Str(a, "axis"));
                e.Word(type);
                if (type == "limit") {
                    if (!Get(a, "min") || !Get(a, "max")) return e.l.Fail(a, "", "a limit axis needs min and max");
                    e.Num(a.at("min"));
                    e.Num(a.at("max"));
                }
                if (const Json* friction = Get(a, "friction")) e.Num(*friction);
            }
            e.Word("}");
        }
    }
    if (const Json* markups = Get(p, "markups")) {
        for (const Json& m : *markups) {
            e.Line(m);
            e.Word("$physicsmarkup");
            e.Text(Str(m, "bone"));
            e.Word("{");
            EmitClause(e, m, "massBias", "massbias");
            EmitClause(e, m, "inertia", "inertia");
            EmitClause(e, m, "damping", "damping");
            EmitClause(e, m, "rotDamping", "rotdamping");
            EmitFlag(e, m, "skip", "skip");
            EmitClause(e, m, "mergeInto", "mergeinto");
            e.Word("}");
        }
    }
    if (const Json* pairs = Get(p, "collisions")) {
        for (const Json& pair : *pairs) {
            e.Line(pair);
            e.Word("$physicscollide");
            e.Text(Str(pair, "a"));
            e.Text(Str(pair, "b"));
        }
    }

    static const struct { const char* key; const char* command; } kValues[] = {
        {"mass", "$mass"}, {"damping", "$damping"}, {"rotDamping", "$rotdamping"},
        {"inertia", "$inertia"}, {"drag", "$drag"}, {"weldPosition", "$weldposition"},
        {"weldNormal", "$weldnormal"},
    };
    for (const auto& value : kValues) {
        const Json* v = Get(p, value.key);
        if (!v) continue;
        e.Line(p, value.key);
        e.Word(value.command);
        e.Num(*v);
    }
    static const struct { const char* key; const char* command; } kFlags[] = {
        {"autoMass", "$automass"}, {"noSelfCollisions", "$noselfcollisions"},
        {"assumeWorldSpace", "$assumeworldspace"},
    };
    for (const auto& flag : kFlags) {
        if (!True(p, flag.key)) continue;
        e.Line(p, flag.key);
        e.Word(flag.command);
    }
    if (const Json* v = Get(p, "massCenter")) {
        e.Line(p, "massCenter");
        e.Word("$masscenter");
        e.Vec(*v);
    }
    for (const auto& [key, command] : {std::pair{"rootBone", "$rootbone"}, std::pair{"phyName", "$phyname"}}) {
        const Json* v = Get(p, key);
        if (!v) continue;
        e.Line(p, key);
        e.Word(command);
        e.Text(v->get<std::string>());
    }
    if (const Json* f = Get(p, "animatedFriction")) {
        e.Line(*f);
        e.Word("$animatedfriction");
        for (const char* key : {"min", "max", "timeIn", "timeOut", "timeHold"}) e.Num(f->at(key));
    }
    e.Word("}");

    if (const Json* list = Get(p, "jointContents")) {
        for (const Json& j : *list) {
            e.Command("$jointcontents", j);
            e.Text(Str(j, "bone"));
            EmitNames(e, j.at("contents"));
        }
    }
    if (const Json* list = Get(p, "jointSurfaceProps")) {
        for (const Json& j : *list) {
            e.Command("$jointsurfaceprop", j);
            e.Text(Str(j, "bone"));
            e.Text(Str(j, "surfaceProp"));
        }
    }
    if (const Json* text = Get(p, "collisionText")) {
        e.Command("$collisiontext", p, "collisionText");
        if (!EmitKeyValues(e, *text)) return false;
    }
    return true;
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

void LowerFlex(Emitter& e, const Json& f) {
    if (const Json* list = Get(f, "dataModel")) {
        for (const Json& d : *list) {
            e.Command("$datamodelflexes", d);
            e.Text(Str(d, "file"));
            EmitFlag(e, d, "controllers", "flexcontroller");
            EmitFlag(e, d, "correctives", "flexcorrective");
            EmitFlag(e, d, "dominators", "flexdominator");
            EmitFlag(e, d, "rules", "flexrule");
        }
    }
    // one command per controller: a range applies to the names after it, so it never leaks
    if (const Json* list = Get(f, "controllers")) {
        for (const Json& c : *list) {
            e.Command("$flexcontroller", c);
            e.Text(Str(c, "group"));
            if (Get(c, "min") || Get(c, "max")) {
                e.Word("range");
                e.Num(Get(c, "min") ? c.at("min") : Json(0));
                e.Num(Get(c, "max") ? c.at("max") : Json(1));
            }
            e.Text(Str(c, "name"));
        }
    }
    if (const Json* names = Get(f, "localVars"); names && !names->empty()) {
        e.Command("$flexlocalvar", f, "localVars");
        EmitNames(e, *names);
    }
    if (const Json* list = Get(f, "rules")) {
        for (const Json& r : *list) {
            e.Command("$flexrule", r);
            e.Text(Str(r, "name"));
            e.Word("=");
            e.Text(Str(r, "expression"));
        }
    }
    static const struct { const char* list; const char* key; const char* command; } kCombos[] = {
        {"correctives", "controls", "$flexcorrective"},
        {"dominations", "dominators", "$flexdominate"},
    };
    for (const auto& combo : kCombos) {
        const Json* list = Get(f, combo.list);
        if (!list) continue;
        for (const Json& r : *list) {
            e.Command(combo.command, r);
            e.Text(Str(r, "name"));
            EmitNames(e, r.at(combo.key));
        }
    }
    if (const Json* list = Get(f, "stereoSplits")) {
        for (const Json& s : *list) {
            e.Command("$morphsplitstereo", s);
            EmitClause(e, s, "splitFactor", "splitfactor");
            e.Text(Str(s, "morph"));
        }
    }
    if (const Json* list = Get(f, "renames")) {
        for (const Json& r : *list) {
            e.Command("$renamemorph", r);
            e.Text(Str(r, "morph"));
            e.Text(Str(r, "to"));
        }
    }
    static const struct { const char* list; const char* command; } kValues[] = {
        {"fixed", "$setflex"}, {"wrinkleScales", "$wrinklescale"}};
    for (const auto& values : kValues) {
        const Json* list = Get(f, values.list);
        if (!list) continue;
        for (const Json& v : *list) {
            e.Command(values.command, v);
            e.Text(Str(v, "morph"));
            e.Num(v.at("value"));
        }
    }
    if (const Json* list = Get(f, "boneDrivers")) {
        for (const Json& d : *list) {
            e.Command("$boneflexdriver", d);
            e.Text(Str(d, "bone"));
            e.Word(Str(d, "component"));
            e.Text(Str(d, "controller"));
            e.Num(d.at("min"));
            e.Num(d.at("max"));
        }
    }
    if (const Json* v = Get(f, "cullMethod")) {
        e.Command("$flexcullmethod", f, "cullMethod");
        e.Word(v->get<std::string>());
    }
}

// Eyeballs first: an eyelid names the eyeballs it closes over.
bool LowerFace(Emitter& e, const Json& face) {
    if (const Json* list = Get(face, "eyeballs")) {
        for (const Json& b : *list) {
            if (!Get(b, "origin") && !True(b, "center"))
                return e.l.Fail(b, "", "an eyeball takes origin, or center: true");
            e.Command("$eyeball", b);
            e.Text(Str(b, "name"));
            EmitClause(e, b, "bone", "bone");
            EmitClause(e, b, "origin", "origin");
            EmitFlag(e, b, "center", "center");
            EmitClause(e, b, "material", "material");
            EmitClause(e, b, "diameter", "diameter");
            EmitClause(e, b, "angle", "angle");
            EmitClause(e, b, "pupilScale", "pupilscale");
        }
    }
    if (const Json* list = Get(face, "mouths")) {
        for (const Json& m : *list) {
            e.Command("$mouth", m);
            e.Text(Str(m, "controller"));
            e.Text(Str(m, "bone"));
            e.Vec(m.at("forward"));
        }
    }
    if (const Json* list = Get(face, "eyelids")) {
        for (const Json& lid : *list) {
            e.Command("$eyelid", lid);
            e.Word(Str(lid, "type"));
            for (const char* pose : {"lowerer", "neutral", "raiser"}) {
                const Json& p = lid.at(pose);
                e.Word(pose);
                const Json* delta = Get(p, "delta");
                e.Text(delta ? delta->get<std::string>() : "-");
                e.Num(p.at("target"));
            }
            EmitClause(e, lid, "split", "split");
            EmitClause(e, lid, "flexDesc", "flexdesc");
            EmitClause(e, lid, "eyeball", "eyeball");
            EmitClause(e, lid, "rightEyeball", "righteyeball");
            EmitClause(e, lid, "leftEyeball", "lefteyeball");
        }
    }
    return true;
}

void EmitRange(Emitter& e, const Json& obj, const char* key, const char* word) {
    const Json* range = Get(obj, key);
    if (!range) return;
    e.Word(word);
    e.Num(range->at("min"));
    e.Num(range->at("max"));
}

void LowerJiggle(Emitter& e, const Json& jb) {
    static const struct { const char* key; const char* word; } kCommon[] = {
        {"tipMass", "tip_mass"}, {"length", "length"}, {"yawFriction", "yaw_friction"},
        {"yawBounce", "yaw_bounce"}, {"pitchFriction", "pitch_friction"},
        {"pitchBounce", "pitch_bounce"}, {"angleConstraint", "angle_constraint"},
    };
    static const struct { const char* key; const char* word; } kFlexible[] = {
        {"yawStiffness", "yaw_stiffness"}, {"yawDamping", "yaw_damping"},
        {"pitchStiffness", "pitch_stiffness"}, {"pitchDamping", "pitch_damping"},
        {"alongStiffness", "along_stiffness"}, {"alongDamping", "along_damping"},
    };
    static const struct { const char* key; const char* word; } kBase[] = {
        {"stiffness", "stiffness"}, {"damping", "damping"}, {"baseMass", "base_mass"},
        {"leftFriction", "left_friction"}, {"upFriction", "up_friction"},
        {"forwardFriction", "forward_friction"},
    };
    static const struct { const char* key; const char* word; } kBoing[] = {
        {"impactSpeed", "impact_speed"}, {"impactAngle", "impact_angle"},
        {"dampingRate", "damping_rate"}, {"frequency", "frequency"}, {"amplitude", "amplitude"},
    };
    auto common = [&](const Json& s) {
        for (const auto& o : kCommon) EmitClause(e, s, o.key, o.word);
        EmitRange(e, s, "yawConstraint", "yaw_constraint");
        EmitRange(e, s, "pitchConstraint", "pitch_constraint");
    };

    e.Command("$jigglebone", jb);
    e.Text(Str(jb, "bone"));
    e.Word("{");
    if (const Json* s = Get(jb, "flexible")) {
        e.Word("is_flexible");
        e.Word("{");
        for (const auto& o : kFlexible) EmitClause(e, *s, o.key, o.word);
        EmitFlag(e, *s, "allowLengthFlex", "allow_length_flex");
        common(*s);
        e.Word("}");
    }
    if (const Json* s = Get(jb, "rigid")) {
        e.Word("is_rigid");
        e.Word("{");
        common(*s);
        e.Word("}");
    }
    if (const Json* s = Get(jb, "baseSpring")) {
        e.Word("has_base_spring");
        e.Word("{");
        for (const auto& o : kBase) EmitClause(e, *s, o.key, o.word);
        EmitRange(e, *s, "leftConstraint", "left_constraint");
        EmitRange(e, *s, "upConstraint", "up_constraint");
        EmitRange(e, *s, "forwardConstraint", "forward_constraint");
        common(*s);
        e.Word("}");
    }
    if (const Json* s = Get(jb, "boing")) {
        e.Word("is_boing");
        e.Word("{");
        for (const auto& o : kBoing) EmitClause(e, *s, o.key, o.word);
        e.Word("}");
    }
    e.Word("}");
}

bool LowerDriver(Emitter& e, const Json& d) {
    e.Command("$driverbone", d);
    e.Text(Str(d, "bone"));
    e.Text(Str(d, "driver"));
    if (True(d, "relative")) e.Word("relative");
    EmitClause(e, d, "poseAnimation", "poseanim");
    e.Word("{");
    EmitClause(e, d, "basePos", "basepos");
    for (const Json& t : d.at("triggers")) {
        if (const Json* frame = Get(t, "frame")) {
            if (Get(t, "driverAngles") || Get(t, "helperAngles") || Get(t, "position"))
                return e.l.Fail(t, "frame", "a frame trigger samples its pose; drop the angles and position");
            e.Word("posetrigger");
            e.Num(t.at("tolerance"));
            e.Num(*frame);
            continue;
        }
        if (!Get(t, "driverAngles") || !Get(t, "helperAngles") || !Get(t, "position"))
            return e.l.Fail(t, "", "a trigger needs driverAngles, helperAngles and position, or a frame");
        e.Word("trigger");
        e.Num(t.at("tolerance"));
        e.Vec(t.at("driverAngles"));
        e.Vec(t.at("helperAngles"));
        e.Vec(t.at("position"));
    }
    e.Word("}");
    return true;
}

bool LowerBones(Emitter& e, const Json& b) {
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

    if (const Json* list = Get(b, "hierarchy")) {
        for (const Json& h : *list) {
            e.Command("$hierarchy", h);
            e.Text(Str(h, "bone"));
            const Json* parent = Get(h, "parent");
            e.Text(parent ? parent->get<std::string>() : "");
        }
    }
    if (const Json* list = Get(b, "jiggle"))
        for (const Json& jb : *list) LowerJiggle(e, jb);
    if (True(b, "cullJiggle")) e.Command("$culljigglebones", b, "cullJiggle");
    if (const Json* list = Get(b, "drivers"))
        for (const Json& d : *list)
            if (!LowerDriver(e, d)) return false;
    if (const Json* list = Get(b, "aimAt")) {
        for (const Json& a : *list) {
            e.Command("$driveraimat", a);
            e.Text(Str(a, "bone"));
            e.Text(Str(a, "target"));
            e.Vec(a.at("up"));
            e.Vec(a.at("aim"));
        }
    }
    if (const Json* files = Get(b, "vrdFiles")) {
        for (const Json& file : *files) {
            e.Command("$proceduralbones", b, "vrdFiles");
            e.Text(file.get<std::string>());
        }
    }
    return true;
}

// Script-defined procedural bones come first: a $driverbone wins over a DMX one of the same name.
bool LowerDataModelJoints(Emitter& e, const Json& d) {
    e.Command("$datamodeljoints", d);
    e.Text(Str(d, "file"));
    EmitFlag(e, d, "jiggleBones", "jigglebones");
    EmitFlag(e, d, "proceduralBones", "proceduralbones");
    EmitFlag(e, d, "hitboxes", "hitboxes");
    EmitFlag(e, d, "attachments", "attachments");

    const Json* edits = Get(d, "triggerEdits");
    if (!edits || edits->empty()) return true;
    e.Word("{");
    e.Word("$proceduralbones");
    e.Word("{");
    for (const Json& edit : *edits) {
        const bool remove = True(edit, "remove");
        if (remove == (Get(edit, "angle") != nullptr))
            return e.l.Fail(edit, "", "a trigger edit takes angle, or remove: true");
        const Json* frame = Get(edit, "frame");
        const Json* frames = Get(edit, "frames");
        if ((frame != nullptr) == (frames != nullptr))
            return e.l.Fail(edit, "", "a trigger edit takes frame or frames");

        e.Word(remove ? "removetrigger" : "settriggeraoi");
        e.Text(Str(edit, "bone"));
        if (!remove) e.Num(edit.at("angle"));
        if (frame) {
            e.Word("frame");
            e.Num(*frame);
        } else {
            e.Word("framerange");
            e.Num(frames->at("min"));
            e.Num(frames->at("max"));
        }
    }
    e.Word("}");
    e.Word("}");
    return true;
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

void EmitRamp(Emitter& e, const Json& ramp) {
    for (const char* key : {"start", "peak", "tail", "end"}) e.Num(ramp.at(key));
}

// True when every key is present; otherwise reports the first missing one.
bool Need(Emitter& e, const Json& obj, const std::string& what, std::initializer_list<const char*> keys) {
    for (const char* key : keys)
        if (!Get(obj, key)) return e.l.Fail(obj, "", what + " needs " + key);
    return true;
}

bool EmitIkRule(Emitter& e, const Json& r, const char* word) {
    const std::string type = Str(r, "type");
    e.Word(word);
    e.Text(Str(r, "chain"));
    e.Word(type);
    if (type == "touch") {
        if (!Need(e, r, "a touch rule", {"bone"})) return false;
        e.Text(Str(r, "bone"));
    } else if (type == "attachment") {
        if (!Need(e, r, "an attachment rule", {"attachment"})) return false;
        e.Text(Str(r, "attachment"));
    } else if (type == "autosteps") {
        if (!Need(e, r, "an autosteps rule", {"steps", "bone"})) return false;
        e.Num(r.at("steps"));
        e.Text(Str(r, "bone"));
    }

    EmitClause(e, r, "height", "height");
    EmitClause(e, r, "target", "target");
    EmitClause(e, r, "floor", "floor");
    EmitClause(e, r, "pad", "pad");
    EmitClause(e, r, "radius", "radius");
    EmitClause(e, r, "contact", "contact");
    if (const Json* range = Get(r, "range")) {
        e.Word("range");
        EmitRamp(e, *range);
    }
    EmitFlag(e, r, "useSequence", "usesequence");
    EmitFlag(e, r, "useSource", "usesource");
    EmitClause(e, r, "fakeOrigin", "fakeorigin");
    EmitClause(e, r, "fakeRotate", "fakerotate");
    return true;
}

void EmitControls(Emitter& e, const Json& op, const char* key) {
    if (const Json* list = Get(op, key))
        for (const Json& word : *list) e.Word(word.get<std::string>());
}

bool EmitOperation(Emitter& e, const Json& op) {
    const std::string name = Str(op, "op");
    const std::string what = "a " + name + " operation";
    auto need = [&](std::initializer_list<const char*> keys) { return Need(e, op, what, keys); };

    if (name == "reverse") {
        e.Word("reverse");
    } else if (name == "numFrames" || name == "compress") {
        if (!need({"count"})) return false;
        e.Word(name == "compress" ? "compress" : "numframes");
        e.Num(op.at("count"));
    } else if (name == "ikFixup") {
        if (!need({"ikRule"})) return false;
        return EmitIkRule(e, op.at("ikRule"), "ikfixup");
    } else if (name == "localHierarchy") {
        if (!need({"bone"})) return false;
        e.Word("localhierarchy");
        e.Text(Str(op, "bone"));
        const Json* parent = Get(op, "parent");
        e.Text(parent ? parent->get<std::string>() : "");
        if (const Json* range = Get(op, "range")) {
            e.Word("range");
            EmitRamp(e, *range);
        }
    } else if (name == "derivative") {
        if (!need({"scale"})) return false;
        e.Word("derivative");
        e.Num(op.at("scale"));
    } else if (name == "noAnimation") {
        e.Word(True(op, "keepDuration") ? "noanim_keepduration" : "noanimation");
    } else if (name == "linearDelta") {
        e.Word(True(op, "spline") ? "splinedelta" : "lineardelta");
    } else if (name == "counterRotate") {
        if (!need({"bone"})) return false;
        const Json* angles = Get(op, "angles");
        e.Word(angles ? "counterrotateto" : "counterrotate");
        if (angles) e.Vec(*angles);
        e.Text(Str(op, "bone"));
    } else if (name == "forceBonePosRot") {
        if (!need({"bone"})) return false;
        e.Word("forceboneposrot");
        e.Text(Str(op, "bone"));
        EmitClause(e, op, "position", "pos");
        if (const Json* rotation = Get(op, "rotation")) {
            e.Word("rot");
            e.Vec(*rotation);
            EmitFlag(e, op, "local", "local");
        }
    } else if (name == "weightList") {
        if (!need({"weightList"})) return false;
        e.Word("weightlist");
        e.Text(Str(op, "weightList"));
    } else if (name == "subtract") {
        if (!need({"animation", "frame"})) return false;
        e.Word(True(op, "pre") ? "presubtract" : "subtract");
        e.Text(Str(op, "animation"));
        e.Num(op.at("frame"));
    } else if (name == "fixupLoop") {
        if (!need({"start", "end"})) return false;
        e.Word("fixuploop");
        e.Num(op.at("start"));
        e.Num(op.at("end"));
    } else if (name == "walkFrame" || name == "walkAlignTo" || name == "walkAlign") {
        const bool reference = name != "walkFrame";
        if (!need({"frame", "controls"}) || (reference && !need({"animation"})) ||
            (name == "walkAlign" && !need({"destinationFrame", "sourceFrame"})))
            return false;
        e.Word(name == "walkFrame" ? "walkframe" : name == "walkAlignTo" ? "walkalignto" : "walkalign");
        e.Num(op.at("frame"));
        if (reference) e.Text(Str(op, "animation"));
        EmitControls(e, op, "controls");
        if (name == "walkAlign") {
            e.Num(op.at("destinationFrame"));
            e.Num(op.at("sourceFrame"));
        }
    } else if (name == "match") {
        if (!need({"animation"})) return false;
        e.Word("match");
        e.Text(Str(op, "animation"));
    } else if (name == "matchBlend") {
        if (!need({"animation", "sourceFrame", "destinationFrame", "preFrames", "postFrames"})) return false;
        e.Word("matchblend");
        e.Text(Str(op, "animation"));
        for (const char* key : {"sourceFrame", "destinationFrame", "preFrames", "postFrames"}) e.Num(op.at(key));
    } else if (name == "transformBone") {
        if (!need({"bone"})) return false;
        e.Word("transformbone");
        e.Text(Str(op, "bone"));
        EmitClause(e, op, "origin", "origin");
        EmitClause(e, op, "angles", "angles");
    } else if (name == "matchPosition") {
        if (!need({"bone"})) return false;
        e.Word("matchposition");
        e.Text(Str(op, "bone"));
        EmitControls(e, op, "axes");
    } else if (name == "copyPose") {
        if (!need({"source", "sourceFrame", "start", "end"})) return false;
        e.Word("copypose");
        e.Text(Str(op, "source"));
        for (const char* key : {"sourceFrame", "start", "end"}) e.Num(op.at(key));
        EmitClause(e, op, "fadeIn", "fadein");
        EmitClause(e, op, "fadeOut", "fadeout");
        EmitFlag(e, op, "copyPosition", "position");
        EmitFlag(e, op, "copyRotation", "rotation");
    } else if (name == "worldSpaceBlend") {
        const bool loop = True(op, "loop");
        if (!need({"animation"}) || (loop && !need({"frame"}))) return false;
        e.Word(loop ? "worldspaceblendloop" : "worldspaceblend");
        e.Text(Str(op, "animation"));
        if (loop) e.Num(op.at("frame"));
    } else if (name == "alignTo" || name == "align" || name == "alignBoneTo" || name == "alignBone") {
        const bool bone = name == "alignBoneTo" || name == "alignBone";
        const bool frames = name == "align" || name == "alignBone";
        if (!need({"animation"}) || (bone && !need({"bone"})) ||
            (frames && !need({"controls", "sourceFrame", "destinationFrame"})))
            return false;
        e.Word(name == "alignTo" ? "alignto" : name == "align" ? "align" :
               name == "alignBoneTo" ? "alignboneto" : "alignbone");
        if (bone) e.Text(Str(op, "bone"));
        e.Text(Str(op, "animation"));
        if (frames) {
            EmitControls(e, op, "controls");
            e.Num(op.at("sourceFrame"));
            e.Num(op.at("destinationFrame"));
        }
    } else if (name == "appendAnimation") {
        if (!need({"animation"})) return false;
        e.Word("appendanim");
        e.Text(Str(op, "animation"));
    } else if (name == "boneDriver") {
        if (!need({"bone"})) return false;
        e.Word("bonedriver");
        e.Text(Str(op, "bone"));
        if (const Json* axis = Get(op, "axis")) {
            e.Word("axis");
            e.Word(axis->get<std::string>());
        }
        EmitClause(e, op, "value", "value");
        if (const Json* range = Get(op, "range")) {
            e.Word("range");
            EmitRamp(e, *range);
        }
    } else if (name == "rotateTo") {
        if (!need({"angle"})) return false;
        e.Word("rotateto");
        e.Num(op.at("angle"));
    }
    return true;
}

// The clip options, written after the file so a sequence applies them to its first
// clip. IK rules and operations get a line each: several read clauses up to line end.
bool LowerClip(Emitter& e, const Json& a) {
    EmitClause(e, a, "fps", "fps");
    EmitFlag(e, a, "loop", "loop");
    EmitFlag(e, a, "noForceLoop", "noforceloop");
    EmitClause(e, a, "startLoop", "startloop");
    EmitClause(e, a, "percentStartLoop", "percentstartloop");
    EmitFlag(e, a, "fudgeLoop", "fudgeloop");
    if (const Json* frames = Get(a, "frames")) {
        e.Word("frame");
        e.Num(frames->at("start"));
        e.Num(frames->at("end"));
    }
    EmitClause(e, a, "frameStart", "framestart");
    EmitClause(e, a, "blockName", "blockname");
    EmitClause(e, a, "origin", "origin");
    EmitClause(e, a, "rotate", "rotate");
    EmitClause(e, a, "angles", "angles");
    EmitClause(e, a, "scale", "scale");
    EmitControls(e, a, "motion");
    EmitClause(e, a, "motionRollback", "motionrollback");
    EmitFlag(e, a, "ignoreScale", "ignorescale");
    if (const Json* list = Get(a, "ignoreTransformBone"))
        for (const Json& which : *list) {
            e.Word("ignoretransformbone");
            e.Word(which.get<std::string>());
        }
    EmitFlag(e, a, "noAutoIk", "noautoik");
    EmitFlag(e, a, "noCull", "nocull");
    EmitFlag(e, a, "noAnimBlock", "noanimblock");
    EmitFlag(e, a, "noAnimBlockStall", "noanimblockstall");
    EmitClause(e, a, "noStallFrames", "nostallframes");

    if (const Json* rules = Get(a, "ikRules")) {
        for (const Json& r : *rules) {
            e.Line(r);
            if (!EmitIkRule(e, r, "ikrule")) return false;
        }
    }
    if (const Json* ops = Get(a, "operations")) {
        for (const Json& op : *ops) {
            e.Line(op);
            if (!EmitOperation(e, op)) return false;
        }
    }
    return true;
}

// `{ bone weight [posweight p] ... }`
void EmitWeights(Emitter& e, const Json& weights) {
    e.Word("{");
    for (const Json& w : weights) {
        e.Text(Str(w, "bone"));
        e.Num(w.at("weight"));
        EmitClause(e, w, "posWeight", "posweight");
    }
    e.Word("}");
}

// Defaults that only reach the clips and sequences written after them.
void LowerAnimationSettings(Emitter& e, const Json& a) {
    static const struct { const char* key; const char* command; } kValues[] = {
        {"defaultFps", "$defaultfps"}, {"motionRollback", "$motionrollback"},
        {"defaultFadeIn", "$defaultfadein"}, {"defaultFadeOut", "$defaultfadeout"},
    };
    for (const auto& value : kValues) {
        const Json* v = Get(a, value.key);
        if (!v) continue;
        e.Command(value.command, a, value.key);
        e.Num(*v);
    }
    if (True(a, "lowercaseSequences")) e.Command("$lcaseallsequences", a, "lowercaseSequences");
    if (const Json* names = Get(a, "allowedActivities")) {
        for (const Json& name : *names) {
            e.Command("$allowactivityname", a, "allowedActivities");
            e.Text(name.get<std::string>());
        }
    }
    if (const Json* s = Get(a, "sectionFrames")) {
        e.Command("$sectionframes", *s);
        e.Num(s->at("frames"));
        e.Num(s->at("minLimit"));
    }
    if (const Json* b = Get(a, "animBlocks")) {
        e.Command("$animblocksize", *b);
        e.Num(b->at("size"));
        EmitFlag(e, *b, "noStall", "nostall");
        EmitFlag(e, *b, "highRes", "highres");
        EmitFlag(e, *b, "lowRes", "lowres");
        EmitClause(e, *b, "zeroFrames", "numframes");
        EmitFlag(e, *b, "cacheHighRes", "cachehighres");
        EmitClause(e, *b, "posDelta", "posdelta");
    }
    if (const Json* v = Get(a, "cullMethod")) {
        e.Command("$animationcullmethod", a, "cullMethod");
        e.Word(v->get<std::string>());
    }
    if (const Json* weights = Get(a, "defaultWeights")) {
        e.Command("$defaultweightlist", a, "defaultWeights");
        EmitWeights(e, *weights);
    }
    if (const Json* locks = Get(a, "autoplayLocks")) {
        for (const Json& lock : *locks) {
            e.Command("$ikautoplaylock", lock);
            e.Text(Str(lock, "chain"));
            e.Num(lock.at("position"));
            e.Num(lock.at("rotation"));
        }
    }
    if (const Json* saves = Get(a, "boneSaveFrames")) {
        for (const Json& s : *saves) {
            e.Command("$bonesaveframe", s);
            e.Text(Str(s, "bone"));
            EmitFlag(e, s, "position", "position");
            EmitFlag(e, s, "rotation", "rotation");
            EmitFlag(e, s, "rotation64", "rotation64");
        }
    }
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
    return LowerClip(e, a);
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
    if (const Json* text = Get(s, "keyValues")) {
        e.Word("keyvalues");
        if (!EmitKeyValues(e, *text)) return false;
    }
    if (!LowerClip(e, s)) return false;

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
    if (const Json* presets = Get(m, "modelGroupPresets")) {
        for (const Json& preset : *presets) {
            e.Command("$modelgrouppreset", preset);
            e.Text(Str(preset, "name"));
            e.Word("{");
            for (const Json& choice : preset.at("choices")) {
                e.Text(Str(choice, "group"));
                e.Num(choice.at("choice"));
            }
            e.Word("}");
        }
    }
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
    if (const Json* face = Get(m, "face"); face && !LowerFace(e, *face)) return false;
    if (const Json* flex = Get(m, "flex")) LowerFlex(e, *flex);
    if (const Json* bones = Get(m, "bones"); bones && !LowerBones(e, *bones)) return false;
    if (const Json* list = Get(m, "dataModelJoints"))
        for (const Json& d : *list)
            if (!LowerDataModelJoints(e, d)) return false;
    if (const Json* list = Get(m, "attachments"))
        for (const Json& a : *list) LowerAttachment(e, a);
    if (const Json* list = Get(m, "hitboxSets"))
        for (const Json& set : *list) LowerHitboxSet(e, set);
    if (const Json* list = Get(m, "hitGroups")) {
        for (const Json& g : *list) {
            e.Command("$hgroup", g);
            e.Num(g.at("group"));
            e.Text(Str(g, "bone"));
        }
    }
    LowerIk(e, m);

    const Json* animation = Get(m, "animationSettings");
    if (animation) LowerAnimationSettings(e, *animation);
    if (const Json* lists = Get(m, "weightLists")) {
        for (const Json& list : *lists) {
            e.Command("$weightlist", list);
            e.Text(Str(list, "name"));
            EmitWeights(e, list.at("bones"));
        }
    }
    if (const Json* list = Get(m, "animations"))
        for (const Json& a : *list)
            if (!LowerAnimation(e, a)) return false;
    if (const Json* list = Get(m, "sequences"))
        for (const Json& s : *list)
            if (!LowerSequence(e, s)) return false;

    // transition nodes are numbered as they are first named, so after every sequence
    if (animation) {
        if (const Json* skips = Get(*animation, "skipTransitions")) {
            for (const Json& skip : *skips) {
                e.Command("$skiptransition", skip);
                EmitNames(e, skip.at("nodes"));
            }
        }
        if (True(*animation, "calcTransitions"))
            e.Command("$calctransitions", *animation, "calcTransitions");
    }
    if (const Json* list = Get(m, "includeModels")) {
        for (const Json& name : *list) {
            e.Command("$includemodel", m, "includeModels");
            e.Text(name.get<std::string>());
        }
    }
    if (const Json* physics = Get(m, "physics"); physics && !LowerPhysics(e, *physics)) return false;
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

// The model the last Load merged, kept for the API records that follow it.
struct LoadedModel {
    std::string id;
    Json merged; // with the internal origin keys
    std::vector<std::string> files;
};
LoadedModel g_loaded;

Json SourceOf(const Json& origin, const std::string& at) {
    const std::string path = origin[1].get<std::string>();
    return Json{{"at", at}, {"file", g_loaded.files[origin[0].get<size_t>()]},
                {"path", path.empty() ? "/" : path}};
}

// `value` without the internal keys; `sources` gets an entry per object and per
// key another file merged in. A value without an entry lies inside its parent's.
Json Clean(const Json& value, const std::string& at, Json& sources) {
    if (value.is_array()) {
        Json out = Json::array();
        for (size_t i = 0; i < value.size(); ++i) out.push_back(Clean(value[i], Pointer(at, i), sources));
        return out;
    }
    if (!value.is_object()) return value;

    if (value.contains(kOrigin)) sources.push_back(SourceOf(KeyOrigin(value, ""), at));
    if (const auto keys = value.find(kKeyOrigins); keys != value.end())
        for (const auto& [key, origin] : keys->items()) sources.push_back(SourceOf(origin, Pointer(at, key)));

    Json out = Json::object();
    for (const auto& [key, entry] : value.items())
        if (!IsInternal(key)) out[key] = Clean(entry, Pointer(at, key), sources);
    return out;
}

// {file, path} of a declaring object; nothing for an item a mesh file or the compiler made.
void Locate(Json& record, const Json* entry) {
    if (!entry) return;
    const Json source = SourceOf(KeyOrigin(*entry, ""), "");
    record["file"] = source["file"];
    record["path"] = source["path"];
}

const Json* FindNamed(const Json* list, const char* key, const std::string& name) {
    if (!list) return nullptr;
    for (const Json& entry : *list)
        if (_stricmp(Str(entry, key).c_str(), name.c_str()) == 0) return &entry;
    return nullptr;
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
        if (field.declares) entry["declares"] = field.declares;
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
    const std::string modelId = Str(model, "id");
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

    g_loaded = {modelId, merged, {}};
    for (const SourceFile& file : l.files) g_loaded.files.push_back(file.path);

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

void PrintResolved() {
    if (g_loaded.id.empty()) return;

    Json sources = Json::array();
    Json content = Clean(g_loaded.merged, "", sources);
    const Json record{{"model", g_loaded.id}, {"content", std::move(content)}, {"sources", std::move(sources)}};
    pulse::api::Record("resolved", record.dump());
}

void PrintCompiled(const compile::CompiledModel& model) {
    if (g_loaded.id.empty()) return;
    const Json& m = g_loaded.merged;
    const Json* bones = Get(m, "bones");
    const Json* settings = Get(m, "model");

    // a renamed bone is declared by its definition under the old name, else by the rename
    Json compiledBones = Json::array();
    for (const compile::Bone& bone : model.bones) {
        Json record{{"name", bone.name}};
        const Json* rename = bones ? FindNamed(Get(*bones, "renames"), "to", bone.name) : nullptr;
        const std::string original = rename ? Str(*rename, "bone") : bone.name;
        const Json* define = bones ? FindNamed(Get(*bones, "define"), "name", original) : nullptr;
        Locate(record, define ? define : rename);
        compiledBones.push_back(std::move(record));
    }

    Json attachments = Json::array();
    for (const compile::Attachment& attachment : model.attachments) {
        Json record{{"name", attachment.name}};
        if (attachment.name == "__illumPosition" && settings)
            Locate(record, Get(*settings, "illumPosition"));
        else
            Locate(record, FindNamed(Get(m, "attachments"), "name", attachment.name));
        attachments.push_back(std::move(record));
    }

    // boxes map by index only while the set holds exactly what was authored
    Json sets = Json::array();
    for (const compile::HitboxSet& set : model.hitboxsets) {
        Json record{{"name", set.name}, {"boxes", Json::array()}};
        const Json* authored = FindNamed(Get(m, "hitboxSets"), "name", set.name);
        Locate(record, authored);
        const Json* boxes = authored ? Get(*authored, "boxes") : nullptr;
        const bool exact = boxes && boxes->size() == set.hitboxes.size();
        for (size_t i = 0; i < set.hitboxes.size(); ++i) {
            Json box = Json::object();
            if (exact) Locate(box, &(*boxes)[i]);
            record["boxes"].push_back(std::move(box));
        }
        sets.push_back(std::move(record));
    }

    const Json record{{"model", g_loaded.id}, {"bones", std::move(compiledBones)},
                      {"attachments", std::move(attachments)}, {"hitboxSets", std::move(sets)}};
    pulse::api::Record("compiled", record.dump());
}

} // namespace pulse::pulsemodel
