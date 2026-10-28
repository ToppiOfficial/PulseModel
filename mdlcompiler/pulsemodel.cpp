// pulsemodel.cpp - .pulsemodel (JSON) front end. See pulsemodel.h.
//
// Load order: parse + validate each file against the schema, drop disabled
// nodes, substitute variables, expand prefabs, merge `shared` and prefab content
// into the chosen model, reshape it, then lower it to script tokens in a fixed order.

#include "strcompat.h"
#include "pathcompat.h"

#include "pulsemodel.h"
#include "api.h"
#include "dependencies.h"
#include "diagnostic.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
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

constexpr int kFormatVersion = 2;
// Added after validation, so no file can write them: an object's [file index,
// JSON pointer], and per key the same for keys merged in from another file.
constexpr const char* kOrigin = "@origin";
constexpr const char* kKeyOrigins = "@keys";

bool IsInternal(const std::string& key) { return !key.empty() && key[0] == '@'; }

// ---------------------------------------------------------------------------
// Schema - every key a .pulsemodel may hold. A model is a fixed set of sections;
// a section or node lists typed child nodes in "children". "editor" is allowed
// in any object and is never read by the compiler.
// ---------------------------------------------------------------------------

enum class Type {
    String, Number, Integer, Bool, Vec3, Enum, File,
    Strings, Files, Variables, Object, List, Any
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
    const char* group = nullptr;         // inspector heading of related fields
    bool required = false;

    constexpr Field Of(const Item* i) const { Field f = *this; f.item = i; return f; }
    constexpr Field Values(const char* const* v) const { Field f = *this; f.values = v; return f; }
    constexpr Field Ref(const char* r) const { Field f = *this; f.ref = r; return f; }
    constexpr Field Declares(const char* d) const { Field f = *this; f.declares = d; return f; }
    constexpr Field Group(const char* g) const { Field f = *this; f.group = g; return f; }
    constexpr Field Required() const { Field f = *this; f.required = true; return f; }
};

struct Item {
    const char* name;
    const Field* fields;
    size_t count;
    const Item* base = nullptr;              // fields shared with another item
    const char* title = "";
    const char* category = "";
    const Item* const* children = nullptr;   // node types it holds, null-terminated
    size_t minChildren = 0;
    bool single = false;                     // at most one per parent

    constexpr Item Titled(const char* t, const char* c) const {
        Item i = *this;
        i.title = t;
        i.category = c;
        return i;
    }
    constexpr Item Holds(const Item* const* kinds, size_t min = 0) const {
        Item i = *this;
        i.children = kinds;
        i.minChildren = min;
        return i;
    }
    constexpr Item Single() const { Item i = *this; i.single = true; return i; }
};

constexpr Field F(const char* key, Type type, const char* doc) { return {key, type, doc}; }

template <size_t N>
constexpr Item MakeItem(const char* name, const Field (&fields)[N], const Item* base = nullptr) {
    return {name, fields, N, base};
}

constexpr Item MakeItem(const char* name, const Item* base = nullptr) { return {name, nullptr, 0, base}; }

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

// A KeyValues block keeps its order and may repeat keys ("break" per gib), so
// each entry is a node and a nested block is its children.
extern const Item kKeyValueItem;
const Item* const kKeyValueKids[] = {&kKeyValueItem, nullptr};
const Field kKeyValue[] = {
    F("key", Type::String, "Key.").Required(),
    F("value", Type::String, "Value; omit when the entry holds a nested block."),
};
const Item kKeyValueItem = MakeItem("keyValue", kKeyValue).Titled("Key Value", "Text").Holds(kKeyValueKids);

// --- one-value nodes: each stands for one entry of a name list ---

const Field kMeshRef[] = {F("mesh", Type::String, "Mesh.").Ref("mesh").Required()};
const Field kMaterialRef[] = {F("material", Type::String, "Material.").Ref("material").Required()};
const Field kBoneRef[] = {F("bone", Type::String, "Bone.").Ref("bone").Required()};
const Field kWordRef[] = {F("word", Type::String, "Word matched in material names.").Required()};
const Field kNameRef[] = {F("name", Type::String, "Name.").Required()};
const Field kFileRef[] = {F("file", Type::File, "File.").Required()};
const Field kFolderRef[] = {F("folder", Type::File, "Folder.").Required()};
const Field kControlRef[] = {F("control", Type::String, "Flex controller, or %morph.").Required()};

const Item kBoundsMeshItem = MakeItem("boundsMesh", kMeshRef).Titled("Bounds Mesh", "Model");
const Item kSearchFolderItem = MakeItem("searchFolder", kFolderRef).Titled("Search Folder", "Model");
const Item kIncludeModelItem = MakeItem("includeModel", kFileRef).Titled("Include Model", "Model");
const Item kMaterialFolderItem = MakeItem("materialFolder", kFolderRef).Titled("Material Folder", "Materials");
const Item kDrawOrderItem = MakeItem("drawOrder", kMaterialRef).Titled("Draw Order", "Materials");
const Item kDropMaterialItem = MakeItem("dropMaterial", kMaterialRef).Titled("Drop Material", "Mesh");
const Item kDropMaterialWordItem = MakeItem("dropMaterialWord", kWordRef).Titled("Drop Material Word", "Mesh");
const Item kChoiceMeshItem = MakeItem("choiceMesh", kMeshRef).Titled("Mesh", "Model Groups");
const Item kRemoveMeshItem = MakeItem("lodRemoveMesh", kMeshRef).Titled("Remove Mesh", "LOD");
const Item kSmallMeshItem = MakeItem("lodSmallMesh", kMeshRef).Titled("Small Mesh Filter", "LOD");
const Item kCollapseTreeItem = MakeItem("lodCollapseTree", kBoneRef).Titled("Collapse Bone Tree", "LOD");
const Item kVrdFileItem = MakeItem("vrdFile", kFileRef).Titled("VRD File", "Bones");
const Item kPlaceFlexGroupItem = MakeItem("placeOnFlexGroup", kNameRef).Titled("On Flex Group", "Attachments");
const Item kPlaceMorphItem = MakeItem("placeOnMorph", kNameRef).Titled("On Morph", "Attachments");
const Item kPlaceMaterialItem = MakeItem("placeOnMaterial", kMaterialRef).Titled("On Material", "Attachments");
const Item kBlendFileItem = MakeItem("blendFile", kFileRef).Titled("Blend File", "Sequences");
const Field kAnimationRef[] = {F("animation", Type::String, "Animation.").Ref("animation").Required()};
const Item kBlendAnimationItem = MakeItem("blendAnimation", kAnimationRef).Titled("Blend Animation", "Sequences");
const Item kActivityModifierItem = MakeItem("activityModifier", kNameRef).Titled("Activity Modifier", "Sequences");
const Item kLocalVarItem = MakeItem("flexLocalVar", kNameRef).Titled("Local Variable", "Flex");
const Item kCorrectiveControlItem = MakeItem("correctiveControl", kControlRef).Titled("Control", "Flex");
const Item kDominatorItem = MakeItem("dominator", kControlRef).Titled("Dominator", "Flex");
const Item kExtraBoneItem = MakeItem("extraBone", kBoneRef).Titled("Extra Bone", "Physics");
const Item kExcludeMeshItem = MakeItem("excludeMesh", kMeshRef).Titled("Excluded Mesh", "Physics");
const Item kTransitionNodeItem = MakeItem("transitionNode", kNameRef).Titled("Node", "Animation");
const Item kAllowedActivityItem = MakeItem("allowedActivity", kNameRef).Titled("Allowed Activity", "Animation");

// trace contents words, shared by the model and per-bone contents; a mask covers other bits
const Field kContentsFlags[] = {
    F("solid", Type::Bool, "Solid.").Group("Contents"),
    F("grate", Type::Bool, "Grate: blocks movement, not bullets.").Group("Contents"),
    F("ladder", Type::Bool, "Ladder.").Group("Contents"),
    F("monster", Type::Bool, "Monster.").Group("Contents"),
    F("debris", Type::Bool, "Debris.").Group("Contents"),
    F("notSolid", Type::Bool, "Not solid.").Group("Contents"),
    F("contentsMask", Type::Integer, "Extra contents bits as a number.").Group("Contents"),
};
const Item kContentsFlagsItem = MakeItem("contentsFlags", kContentsFlags);

// --- settings ---

const Field kSettings[] = {
    F("name", Type::String, "Model path under models/, e.g. player/foo."),
    F("archetype", Type::Enum, "Model archetype.").Values(kArchetypes),
    F("origin", Type::Vec3, "Model translation.").Group("Transform"),
    F("angles", Type::Vec3, "Model rotation in degrees (pitch yaw roll).").Group("Transform"),
    F("scale", Type::Number, "Uniform model scale.").Group("Transform"),
    F("upAxis", Type::Enum, "Up axis of the source art; replaces angles.").Values(kUpAxes).Group("Transform"),
    F("renderPass", Type::Enum, "Render pass.").Values(kRenderPasses),
    F("surfaceProp", Type::String, "Surface property."),
    F("maxBones", Type::Integer, "Bone ceiling.").Group("Budget"),
    F("maxMaterials", Type::Integer, "Material ceiling.").Group("Budget"),
    F("bboxMin", Type::Vec3, "Hull box minimum corner.").Group("Bounds"),
    F("bboxMax", Type::Vec3, "Hull box maximum corner.").Group("Bounds"),
    F("cboxMin", Type::Vec3, "View clipping box minimum corner.").Group("Bounds"),
    F("cboxMax", Type::Vec3, "View clipping box maximum corner.").Group("Bounds"),
    F("skipBoneInBBox", Type::Bool, "Auto hitboxes grow from vertices only.").Group("Bounds"),
    F("illumPosition", Type::Vec3, "Lighting sample point.").Group("Lighting"),
    F("illumBone", Type::String, "Bone the lighting point follows; omitted = static.").Ref("bone").Group("Lighting"),
    F("constantDirectionalLight", Type::Number, "Constant directional light scale (0-1).").Group("Lighting"),
    F("ambientBoost", Type::Bool, "Ambient boost flag.").Group("Lighting"),
    F("doNotCastShadows", Type::Bool, "Cast no shadows.").Group("Lighting"),
    F("castTextureShadows", Type::Bool, "Cast texture shadows.").Group("Lighting"),
    F("eyePosition", Type::Vec3, "Ideal eye point.").Group("Eyes"),
    F("eyeAutoHeight", Type::Bool, "Z comes from the eyeballs; the eye position Z is an offset.").Group("Eyes"),
    F("maxEyeDeflection", Type::Number, "Eye turn limit in degrees.").Group("Eyes"),
    F("forcePhonemeCrossfade", Type::Bool, "Force phoneme crossfade.").Group("Eyes"),
    F("allowRootLods", Type::Integer, "Allowed root LOD count.").Group("LOD"),
    F("minLod", Type::Integer, "Promote this LOD to root.").Group("LOD"),
    F("noForcedFade", Type::Bool, "No forced fade.").Group("LOD"),
    F("vtxFormat", Type::Integer, ".vtx layout: 0 legacy (TF2/L4D2/GMod/HL2), 1 full (SFM/CS:GO).").Group("Output"),
    F("noDx80", Type::Bool, "Skip the .dx80.vtx.").Group("Output"),
    F("autoCenter", Type::Bool, "Center the model.").Group("Output"),
    F("forceWriteVertexData", Type::Bool, "Write .vvd/.vtx even without geometry.").Group("Output"),
};
const Item* const kSettingsKids[] = {&kSearchFolderItem, &kIncludeModelItem, &kBoundsMeshItem, &kKeyValueItem,
                                     nullptr};
const Item kSettingsItem =
    MakeItem("settings", kSettings, &kContentsFlagsItem).Titled("Settings", "Model").Holds(kSettingsKids);

// --- variables and prefabs ---

const Field kVariable[] = {
    F("name", Type::String, "Variable name, written $name$ in any string.").Required(),
    F("value", Type::String, "Value.").Required(),
    F("redefine", Type::Bool, "Change a variable defined earlier instead of defining a new one."),
};
const Item kVariableItem = MakeItem("variable", kVariable).Titled("Variable", "Variables");
const Item* const kVariablesKids[] = {&kVariableItem, nullptr};
const Item kVariablesItem = MakeItem("variables").Titled("Variables", "Variables").Holds(kVariablesKids);

const Field kPrefab[] = {
    F("file", Type::File, "A .pulsemodel holding one model, or only shared content.").Required(),
    F("variables", Type::Variables, "Values passed in; they win over the prefab's own."),
};
const Item kPrefabItem = MakeItem("prefab", kPrefab).Titled("Prefab", "Variables");
const Item* const kPrefabsKids[] = {&kPrefabItem, nullptr};
const Item kPrefabsItem = MakeItem("prefabs").Titled("Prefabs", "Variables").Holds(kPrefabsKids);

// --- materials ---

const Field kReplace[] = {
    F("material", Type::String, "Material to replace.").Ref("material").Required(),
    F("with", Type::String, "Replacement material.").Required(),
};
const Item kSkinReplaceItem = MakeItem("skinReplace", kReplace).Titled("Material Swap", "Materials");
const Item* const kSkinKids[] = {&kSkinReplaceItem, nullptr};
const Item kSkinItem = MakeItem("skin").Titled("Skin", "Materials").Holds(kSkinKids);
const Item kMaterialRenameItem = MakeItem("materialRename", kReplace).Titled("Material Rename", "Materials");
const Item kMaterialCollapseItem = MakeItem("materialCollapse", kReplace).Titled("Material Collapse", "Materials");

const Item* const kMaterialsKids[] = {&kMaterialFolderItem, &kSkinItem, &kMaterialRenameItem,
                                      &kMaterialCollapseItem, &kDrawOrderItem, nullptr};
const Item kMaterialsItem = MakeItem("materials").Titled("Materials", "Materials").Holds(kMaterialsKids);

// --- meshes and model groups ---

const Field kScaleBone[] = {
    F("bone", Type::String, "Bone to scale.").Ref("bone").Required(),
    F("factor", Type::Number, "Scale factor.").Required(),
    F("x", Type::Bool, "Scale along X; no axis set = all.").Group("Axes"),
    F("y", Type::Bool, "Scale along Y; no axis set = all.").Group("Axes"),
    F("z", Type::Bool, "Scale along Z; no axis set = all.").Group("Axes"),
};
const Item kScaleBoneItem = MakeItem("scaleBone", kScaleBone).Titled("Scale Bone", "Mesh");

const Field kMesh[] = {
    F("name", Type::String, "Mesh name model groups refer to.").Declares("mesh").Required(),
    F("file", Type::File, "Source file (.dmx, .smd, .fbx).").Required(),
    F("noMorph", Type::Bool, "Load without morphs."),
    F("flipNormals", Type::Bool, "Flip normals."),
    F("decimate", Type::Number, "Keep this share of faces (0-1]."),
    F("skinnedBoneCull", Type::Enum, "Drop unweighted bones.").Values(kSkinnedBoneCulls),
};
const Item* const kMeshKids[] = {&kScaleBoneItem, &kDropMaterialItem, &kDropMaterialWordItem, nullptr};
const Item kMeshItem = MakeItem("mesh", kMesh).Titled("Mesh", "Mesh").Holds(kMeshKids);
const Item* const kMeshesKids[] = {&kMeshItem, nullptr};
const Item kMeshesItem = MakeItem("meshes").Titled("Meshes", "Mesh").Holds(kMeshesKids);

const Field kChoice[] = {
    F("name", Type::String, "Choice name; omitted = named after its meshes."),
    F("blank", Type::Bool, "An empty choice."),
};
const Item* const kChoiceKids[] = {&kChoiceMeshItem, nullptr};
const Item kChoiceItem = MakeItem("choice", kChoice).Titled("Choice", "Model Groups").Holds(kChoiceKids);
const Item* const kModelGroupKids[] = {&kChoiceItem, nullptr};
const Field kModelGroup[] = {
    F("name", Type::String, "Group name.").Declares("modelGroup").Required(),
};
const Item kModelGroupItem =
    MakeItem("modelGroup", kModelGroup).Titled("Model Group", "Model Groups").Holds(kModelGroupKids, 1);

const Field kPresetChoice[] = {
    F("group", Type::String, "Model group.").Ref("modelGroup").Required(),
    F("choice", Type::Integer, "Choice index.").Required(),
};
const Item kPresetChoiceItem = MakeItem("presetChoice", kPresetChoice).Titled("Preset Choice", "Model Groups");
const Item* const kModelGroupPresetKids[] = {&kPresetChoiceItem, nullptr};
const Field kModelGroupPreset[] = {
    F("name", Type::String, "Preset name.").Declares("modelGroupPreset").Required(),
};
const Item kModelGroupPresetItem = MakeItem("modelGroupPreset", kModelGroupPreset)
                                       .Titled("Model Group Preset", "Model Groups")
                                       .Holds(kModelGroupPresetKids, 1);
const Item* const kModelGroupsKids[] = {&kModelGroupItem, &kModelGroupPresetItem, nullptr};
const Item kModelGroupsItem = MakeItem("modelGroups").Titled("Model Groups", "Model Groups").Holds(kModelGroupsKids);

// --- LODs ---

const Field kLodModel[] = {
    F("model", Type::String, "Mesh this LOD replaces.").Ref("mesh").Required(),
    F("mesh", Type::String, "Replacement mesh.").Ref("mesh"),
    F("file", Type::File, "Replacement source file."),
};
const Item kLodModelItem = MakeItem("lodReplaceMesh", kLodModel).Titled("Replace Mesh", "LOD");

const Field kLodDecimate[] = {
    F("model", Type::String, "Mesh to decimate.").Ref("mesh").Required(),
    F("factor", Type::Number, "Share of faces kept (0-1].").Required(),
};
const Item kLodDecimateItem = MakeItem("lodDecimateMesh", kLodDecimate).Titled("Decimate Mesh", "LOD");

const Field kLodBone[] = {
    F("bone", Type::String, "Bone to replace.").Ref("bone").Required(),
    F("with", Type::String, "Replacement bone.").Ref("bone").Required(),
};
const Item kLodBoneItem = MakeItem("lodReplaceBone", kLodBone).Titled("Replace Bone", "LOD");
const Item kLodMaterialItem = MakeItem("lodReplaceMaterial", kReplace).Titled("Replace Material", "LOD");

const Field kLodWord[] = {
    F("word", Type::String, "Word to match in material names.").Required(),
    F("with", Type::String, "Replacement material.").Required(),
};
const Item kLodWordItem = MakeItem("lodReplaceMaterialWord", kLodWord).Titled("Replace Material Word", "LOD");

const Field kLod[] = {
    F("switch", Type::Number, "Switch value; omit for the shadow LOD."),
    F("shadow", Type::Bool, "The shadow LOD (last, at most one)."),
    F("decimateAll", Type::Number, "Decimate every mesh to this share."),
    F("collapseJiggleBones", Type::Bool, "Collapse jiggle bones."),
    F("collapseProceduralBones", Type::Bool, "Collapse procedural bones."),
    F("facial", Type::Bool, "Facial animation on this LOD."),
    F("skeletalAwareDecimation", Type::Bool, "Decimate with the skeleton in mind."),
    F("useShadowLodMaterials", Type::Bool, "Shadow LOD keeps its own materials."),
    F("removeSmallMeshes", Type::Number, "Drop meshes under this unit size; Small Mesh Filter nodes limit it."),
};
const Item* const kLodKids[] = {&kLodModelItem, &kLodDecimateItem, &kLodBoneItem, &kLodMaterialItem,
                                &kLodWordItem, &kRemoveMeshItem, &kCollapseTreeItem, &kDropMaterialItem,
                                &kDropMaterialWordItem, &kSmallMeshItem, nullptr};
const Item kLodItem = MakeItem("lod", kLod).Titled("LOD", "LOD").Holds(kLodKids);
const Item* const kLodsKids[] = {&kLodItem, nullptr};
const Item kLodsItem = MakeItem("lods").Titled("LODs", "LOD").Holds(kLodsKids);

// --- bones ---

const Field kDefineBone[] = {
    F("name", Type::String, "Bone name.").Declares("bone").Required(),
    F("parent", Type::String, "Parent bone; omitted = root bone.").Ref("bone"),
    F("unlocked", Type::Bool, "Appended after the source bones instead of replacing one."),
    F("position", Type::Vec3, "Bind position.").Required(),
    F("angles", Type::Vec3, "Bind rotation in degrees.").Required(),
    F("realignPosition", Type::Vec3, "Realign position; marks the bone pre-aligned.").Group("Realign"),
    F("realignAngles", Type::Vec3, "Realign rotation in degrees.").Group("Realign"),
};
const Item kDefineBoneItem = MakeItem("defineBone", kDefineBone).Titled("Define Bone", "Bones");

const Field kRename[] = {
    F("bone", Type::String, "Bone to rename.").Ref("bone").Required(),
    F("to", Type::String, "New name.").Required(),
};
const Item kRenameItem = MakeItem("renameBone", kRename).Titled("Rename Bone", "Bones");

const Field kTransformBone[] = {
    F("bone", Type::String, "Bone to edit.").Ref("bone").Required(),
    F("angles", Type::Vec3, "Rotate the bind orientation."),
    F("position", Type::Vec3, "Translate the bind position."),
    F("worldAngles", Type::Bool, "Angles are in world axes."),
    F("worldPosition", Type::Bool, "Position is in world axes."),
    F("transformVerts", Type::Bool, "Carry the rigged vertices."),
    F("transformChildren", Type::Bool, "Move the whole subtree."),
    F("ignoreAnimation", Type::Bool, "Never apply to animation."),
    F("ignoreHitbox", Type::Bool, "Leave this bone's hitbox in place."),
    F("residualBone", Type::String, "Bone the weights ramp onto.").Ref("bone").Group("Re-skin"),
    F("weightsFactor", Type::Number, "Ramp band width (0-1).").Group("Re-skin"),
    F("weightsSmoothing", Type::Number, "S-curve easing (0-1).").Group("Re-skin"),
    F("offset", Type::Vec3, "Shift the ramp end.").Group("Re-skin"),
};
const Item kTransformBoneItem = MakeItem("transformBone", kTransformBone).Titled("Transform Bone", "Bones");

const Field kRealignBone[] = {
    F("bone", Type::String, "Bone to realign; does not prevent collapse.").Ref("bone").Required(),
    F("origin", Type::Vec3, "Origin offset along the starting bone axes."),
    F("angles", Type::Vec3, "Rotation offset in degrees (pitch, yaw, roll)."),
    F("worldAxes", Type::Bool, "Apply relative offsets in model/pose axes."),
    F("absolute", Type::Bool, "Set supplied components to absolute model/pose values."),
    F("affectChildren", Type::Bool,
      "Carry descendant bone frames; requires origin or angles and preserves the rest mesh."),
    F("ignoreAnimation", Type::Bool,
      "Exclude from source animation; bind-pose clips still use the edited pose."),
};
const Item kRealignBoneItem = MakeItem("realignBone", kRealignBone).Titled("Realign Bone", "Bones");

const Field kHierarchy[] = {
    F("bone", Type::String, "Bone to reparent.").Ref("bone").Required(),
    F("parent", Type::String, "New parent; omitted = root bone.").Ref("bone"),
};
const Item kHierarchyItem = MakeItem("reparentBone", kHierarchy).Titled("Reparent Bone", "Bones");

// options every jiggle behavior but boing takes
const Field kJiggleCommon[] = {
    F("tipMass", Type::Number, "Tip mass."),
    F("length", Type::Number, "Length."),
    F("yawFriction", Type::Number, "Yaw friction."),
    F("yawBounce", Type::Number, "Yaw bounce."),
    F("pitchFriction", Type::Number, "Pitch friction."),
    F("pitchBounce", Type::Number, "Pitch bounce."),
    F("angleConstraint", Type::Number, "Angle limit in degrees.").Group("Limits"),
    F("yawMin", Type::Number, "Yaw limit minimum in degrees.").Group("Limits"),
    F("yawMax", Type::Number, "Yaw limit maximum in degrees.").Group("Limits"),
    F("pitchMin", Type::Number, "Pitch limit minimum in degrees.").Group("Limits"),
    F("pitchMax", Type::Number, "Pitch limit maximum in degrees.").Group("Limits"),
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
const Item kJiggleFlexibleItem =
    MakeItem("jiggleFlexible", kJiggleFlexible, &kJiggleCommonItem).Titled("Flexible", "Bones").Single();
const Item kJiggleRigidItem = MakeItem("jiggleRigid", &kJiggleCommonItem).Titled("Rigid", "Bones").Single();

const Field kJiggleBaseSpring[] = {
    F("stiffness", Type::Number, "Stiffness (0-1000)."),
    F("damping", Type::Number, "Damping (0-1000)."),
    F("baseMass", Type::Number, "Base mass."),
    F("leftMin", Type::Number, "Left limit minimum.").Group("Left"),
    F("leftMax", Type::Number, "Left limit maximum.").Group("Left"),
    F("leftFriction", Type::Number, "Left friction.").Group("Left"),
    F("upMin", Type::Number, "Up limit minimum.").Group("Up"),
    F("upMax", Type::Number, "Up limit maximum.").Group("Up"),
    F("upFriction", Type::Number, "Up friction.").Group("Up"),
    F("forwardMin", Type::Number, "Forward limit minimum.").Group("Forward"),
    F("forwardMax", Type::Number, "Forward limit maximum.").Group("Forward"),
    F("forwardFriction", Type::Number, "Forward friction.").Group("Forward"),
};
const Item kJiggleBaseSpringItem =
    MakeItem("jiggleBaseSpring", kJiggleBaseSpring, &kJiggleCommonItem).Titled("Base Spring", "Bones").Single();

const Field kJiggleBoing[] = {
    F("impactSpeed", Type::Number, "Impact speed (100 when omitted)."),
    F("impactAngle", Type::Number, "Impact angle in degrees."),
    F("dampingRate", Type::Number, "Damping rate."),
    F("frequency", Type::Number, "Frequency."),
    F("amplitude", Type::Number, "Amplitude."),
};
const Item kJiggleBoingItem = MakeItem("jiggleBoing", kJiggleBoing).Titled("Boing", "Bones").Single();

const Field kJiggleBone[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
};
const Item* const kJiggleBoneKids[] = {&kJiggleFlexibleItem, &kJiggleRigidItem, &kJiggleBaseSpringItem,
                                       &kJiggleBoingItem, nullptr};
const Item kJiggleBoneItem = MakeItem("jiggleBone", kJiggleBone).Titled("Jiggle Bone", "Bones").Holds(kJiggleBoneKids);

const Field kDriverTrigger[] = {
    F("tolerance", Type::Number, "Tolerance in degrees.").Required(),
    F("driverAngles", Type::Vec3, "Driver rotation (roll pitch yaw degrees)."),
    F("helperAngles", Type::Vec3, "Helper rotation (roll pitch yaw degrees)."),
    F("position", Type::Vec3, "Helper position."),
    F("frame", Type::Integer, "Sample both poses from this frame of poseAnimation instead."),
};
const Item kDriverTriggerItem = MakeItem("driverTrigger", kDriverTrigger).Titled("Trigger", "Bones");

const Field kDriverBone[] = {
    F("bone", Type::String, "Helper bone the engine drives.").Ref("bone").Required(),
    F("driver", Type::String, "Bone it follows.").Ref("bone").Required(),
    F("relative", Type::Bool, "Trigger poses are deltas from the bind pose."),
    F("poseAnimation", Type::File, "Animation the frame triggers sample."),
    F("basePos", Type::Vec3, "Offset added to every trigger position."),
};
const Item* const kDriverBoneKids[] = {&kDriverTriggerItem, nullptr};
const Item kDriverBoneItem =
    MakeItem("driverBone", kDriverBone).Titled("Driver Bone", "Bones").Holds(kDriverBoneKids, 1);

const Field kAimAt[] = {
    F("bone", Type::String, "Bone that aims.").Ref("bone").Required(),
    F("target", Type::String, "Attachment, else bone, it aims at.").Required(),
    F("up", Type::Vec3, "Up vector.").Required(),
    F("aim", Type::Vec3, "Aim vector.").Required(),
};
const Item kAimAtItem = MakeItem("aimAtBone", kAimAt).Titled("Aim-at Bone", "Bones");

const Field kDataModelJoints[] = {
    F("file", Type::File, "DMX holding the bone markup.").Required(),
    F("jiggleBones", Type::Bool, "Import its jiggle bones."),
    F("proceduralBones", Type::Bool, "Import its procedural bones."),
    F("hitboxes", Type::Bool, "Import its hitboxes."),
    F("attachments", Type::Bool, "Import its attachments."),
};
const Item kDataModelJointsItem =
    MakeItem("dataModelJoints", kDataModelJoints).Titled("DMX Bone Markup", "Bones");

const Field kBones[] = {
    F("root", Type::String, "Motion root bone.").Ref("bone"),
    F("cullMethod", Type::Enum, "Bone cull method.").Values(kBoneCulls),
    F("realign", Type::Bool, "Realign single-child chains."),
    F("lockLengths", Type::Bool, "Pin bone lengths to the bind pose."),
    F("cullJiggle", Type::Bool, "Drop jiggle bones nothing is weighted to."),
};

const Field kBoneMarkup[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("boneMerge", Type::Bool, "Kept for bonemerge."),
    F("doNotCollapse", Type::Bool, "Never collapsed."),
    F("noRealign", Type::Bool,
      "Disable realignment; the last per-bone realign or noRealign instruction wins."),
    F("alwaysCollapse", Type::Bool, "Always collapsed."),
};
const Item kBoneMarkupItem = MakeItem("boneMarkup", kBoneMarkup).Titled("Bone Markup", "Bones");

const Item* const kBonesKids[] = {&kDefineBoneItem, &kBoneMarkupItem, &kRenameItem, &kTransformBoneItem,
                                  &kRealignBoneItem, &kHierarchyItem, &kJiggleBoneItem, &kDriverBoneItem, &kAimAtItem,
                                  &kDataModelJointsItem, &kVrdFileItem, nullptr};
const Item kBonesItem = MakeItem("bones", kBones).Titled("Bones", "Bones").Holds(kBonesKids);

// --- attachments, hitboxes, pose parameters, IK ---

const Field kAttachment[] = {
    F("name", Type::String, "Attachment name.").Declares("attachment").Required(),
    F("bone", Type::String, "Parent bone; omitted = model space.").Ref("bone"),
    F("position", Type::Vec3, "Position."),
    F("angles", Type::Vec3, "Rotation in degrees."),
    F("rigid", Type::Bool, "Rigid."),
    F("absolute", Type::Bool, "Position is in model space."),
    F("worldAlign", Type::Bool, "World aligned."),
};
const Item* const kAttachmentKids[] = {&kPlaceFlexGroupItem, &kPlaceMorphItem, &kPlaceMaterialItem, nullptr};
const Item kAttachmentItem =
    MakeItem("attachment", kAttachment).Titled("Attachment", "Attachments").Holds(kAttachmentKids);
const Item* const kAttachmentsKids[] = {&kAttachmentItem, nullptr};
const Item kAttachmentsItem = MakeItem("attachments").Titled("Attachments", "Attachments").Holds(kAttachmentsKids);

const Field kHitbox[] = {
    F("group", Type::Integer, "Hit group.").Required(),
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("min", Type::Vec3, "Minimum corner.").Required(),
    F("max", Type::Vec3, "Maximum corner.").Required(),
    F("angles", Type::Vec3, "Rotation in degrees."),
    F("radius", Type::Number, "Capsule radius."),
    F("name", Type::String, "Hitbox name."),
};
const Item kHitboxItem = MakeItem("hitbox", kHitbox).Titled("Hitbox", "Hitboxes");

const Field kHitboxSet[] = {
    F("name", Type::String, "Set name.").Declares("hitboxSet").Required(),
};
const Item* const kHitboxSetKids[] = {&kHitboxItem, nullptr};
const Item kHitboxSetItem = MakeItem("hitboxSet", kHitboxSet).Titled("Hitbox Set", "Hitboxes").Holds(kHitboxSetKids);

const Field kHitGroup[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("group", Type::Integer, "Hit group of its automatic hitbox.").Required(),
};
const Item kHitGroupItem = MakeItem("hitGroup", kHitGroup).Titled("Hit Group", "Hitboxes");
const Item* const kHitboxesKids[] = {&kHitboxSetItem, &kHitGroupItem, nullptr};
const Item kHitboxesItem = MakeItem("hitboxes").Titled("Hitboxes", "Hitboxes").Holds(kHitboxesKids);

const Field kPoseParameter[] = {
    F("name", Type::String, "Parameter name.").Declares("poseParameter").Required(),
    F("min", Type::Number, "Start value.").Required(),
    F("max", Type::Number, "End value.").Required(),
    F("wrap", Type::Bool, "Wraps over the full range."),
    F("loop", Type::Number, "Loops at this value."),
};
const Item kPoseParameterItem = MakeItem("poseParameter", kPoseParameter).Titled("Pose Parameter", "Animation");
const Item* const kPoseParametersKids[] = {&kPoseParameterItem, nullptr};
const Item kPoseParametersItem =
    MakeItem("poseParameters").Titled("Pose Parameters", "Animation").Holds(kPoseParametersKids);

const Field kIkChain[] = {
    F("name", Type::String, "Chain name.").Declares("ikChain").Required(),
    F("bone", Type::String, "End bone.").Ref("bone").Required(),
    F("knee", Type::Vec3, "Knee direction."),
    F("height", Type::Number, "Height."),
    F("pad", Type::Number, "Pad."),
    F("floor", Type::Number, "Floor."),
    F("center", Type::Vec3, "Center."),
};
const Item kIkChainItem = MakeItem("ikChain", kIkChain).Titled("IK Chain", "IK");
const Item* const kIkKids[] = {&kIkChainItem, nullptr};
const Item kIkItem = MakeItem("ik").Titled("IK", "IK").Holds(kIkKids);

// --- animations and sequences ---

const Field kIkRule[] = {
    F("chain", Type::String, "IK chain.").Ref("ikChain").Required(),
    F("rule", Type::Enum, "Rule type.").Values(kIkRuleTypes).Required(),
    F("bone", Type::String, "touch: the bone touched; autosteps: the foot bone.").Ref("bone"),
    F("attachment", Type::String, "attachment: the attachment.").Ref("attachment"),
    F("steps", Type::Integer, "autosteps: step count."),
    F("height", Type::Number, "Height."),
    F("target", Type::Integer, "Target slot."),
    F("floor", Type::Number, "Floor."),
    F("pad", Type::Number, "Pad (sets radius to half)."),
    F("radius", Type::Number, "Radius."),
    F("contact", Type::Integer, "Contact frame."),
    F("useSequence", Type::Bool, "Use the sequence."),
    F("useSource", Type::Bool, "Use the source."),
    F("fakeOrigin", Type::Vec3, "Fake origin."),
    F("fakeRotate", Type::Vec3, "Fake rotation."),
    F("rangeStart", Type::Integer, "Ramp start frame (-1 = unset).").Group("Ramp"),
    F("rangePeak", Type::Integer, "Ramp peak frame (-1 = unset).").Group("Ramp"),
    F("rangeTail", Type::Integer, "Ramp tail frame (-1 = unset).").Group("Ramp"),
    F("rangeEnd", Type::Integer, "Ramp end frame (-1 = unset).").Group("Ramp"),
};
const Item kIkRuleItem = MakeItem("ikRule", kIkRule).Titled("IK Rule", "Animation");
const Item kIkFixupRuleItem = MakeItem("ikFixupRule", &kIkRuleItem).Titled("IK Fixup Rule", "Animation").Single();

// motion extraction controls; a clip's motion and an operation's controls both use them
const Field kMotionControls[] = {
    F("x", Type::Bool, "X.").Group("Motion"),       F("y", Type::Bool, "Y.").Group("Motion"),
    F("z", Type::Bool, "Z.").Group("Motion"),       F("xr", Type::Bool, "XR.").Group("Motion"),
    F("yr", Type::Bool, "YR.").Group("Motion"),     F("zr", Type::Bool, "ZR.").Group("Motion"),
    F("lx", Type::Bool, "LX.").Group("Motion"),     F("ly", Type::Bool, "LY.").Group("Motion"),
    F("lz", Type::Bool, "LZ.").Group("Motion"),     F("lxr", Type::Bool, "LXR.").Group("Motion"),
    F("lyr", Type::Bool, "LYR.").Group("Motion"),   F("lzr", Type::Bool, "LZR.").Group("Motion"),
    F("lm", Type::Bool, "LM.").Group("Motion"),     F("lq", Type::Bool, "LQ.").Group("Motion"),
};
const Item kMotionControlsItem = MakeItem("motionControls", kMotionControls);

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
    F("axisX", Type::Bool, "matchPosition: match X; no axis set = all.").Group("Axes"),
    F("axisY", Type::Bool, "matchPosition: match Y; no axis set = all.").Group("Axes"),
    F("axisZ", Type::Bool, "matchPosition: match Z; no axis set = all.").Group("Axes"),
    F("axis", Type::Enum, "boneDriver: axis.").Values(kAxes),
    F("origin", Type::Vec3, "transformBone: offset."),
    F("angles", Type::Vec3, "counterRotate: explicit target; transformBone: rotation."),
    F("position", Type::Vec3, "forceBonePosRot: position."),
    F("rotation", Type::Vec3, "forceBonePosRot: rotation."),
    F("local", Type::Bool, "forceBonePosRot: rotation is local."),
    F("keepDuration", Type::Bool, "noAnimation: keep the frame count."),
    F("spline", Type::Bool, "linearDelta: spline ease."),
    F("pre", Type::Bool, "subtract: presubtract."),
    F("loop", Type::Bool, "worldSpaceBlend: sample by cycle from frame."),
    F("copyPosition", Type::Bool, "copyPose: copy position (both when neither is set)."),
    F("copyRotation", Type::Bool, "copyPose: copy rotation."),
    F("rangeStart", Type::Integer, "localHierarchy, boneDriver: ramp start frame.").Group("Ramp"),
    F("rangePeak", Type::Integer, "localHierarchy, boneDriver: ramp peak frame.").Group("Ramp"),
    F("rangeTail", Type::Integer, "localHierarchy, boneDriver: ramp tail frame.").Group("Ramp"),
    F("rangeEnd", Type::Integer, "localHierarchy, boneDriver: ramp end frame.").Group("Ramp"),
};
const Item* const kAnimOpKids[] = {&kIkFixupRuleItem, nullptr};
const Item kAnimOpItem = MakeItem("animationOperation", kAnimOp, &kMotionControlsItem)
                             .Titled("Operation", "Animation")
                             .Holds(kAnimOpKids);

// options shared by animations and a sequence's first clip
const Field kClip[] = {
    F("fps", Type::Number, "Frame rate."),
    F("loop", Type::Bool, "Looping.").Group("Loop"),
    F("noForceLoop", Type::Bool, "Do not force the loop.").Group("Loop"),
    F("startLoop", Type::Integer, "Loop restart frame.").Group("Loop"),
    F("percentStartLoop", Type::Number, "Loop restart as a percentage.").Group("Loop"),
    F("fudgeLoop", Type::Bool, "Fudge the loop.").Group("Loop"),
    F("framesFrom", Type::Integer, "First frame of the range.").Group("Frames"),
    F("framesTo", Type::Integer, "Last frame of the range.").Group("Frames"),
    F("frameStart", Type::Integer, "First frame; the end stays the clip's.").Group("Frames"),
    F("blockName", Type::String, "Clip of a multi-clip source."),
    F("origin", Type::Vec3, "Clip translation.").Group("Transform"),
    F("rotate", Type::Number, "Clip yaw in degrees.").Group("Transform"),
    F("angles", Type::Vec3, "Clip rotation in degrees.").Group("Transform"),
    F("scale", Type::Number, "Clip scale.").Group("Transform"),
    F("ignoreScale", Type::Bool, "Ignore the model transform scale.").Group("Transform"),
    F("ignoreBoneAngles", Type::Bool, "Undo the bone angle transforms here.").Group("Transform"),
    F("ignoreBonePosition", Type::Bool, "Undo the bone position transforms here.").Group("Transform"),
    F("motionRollback", Type::Number, "Motion rollback in seconds.").Group("Motion"),
    F("noAutoIk", Type::Bool, "No automatic IK release rules."),
    F("noCull", Type::Bool, "Never culled by the animation cull method."),
    F("noAnimBlock", Type::Bool, "Keep out of the demand-loaded .ani.").Group("Demand loading"),
    F("noAnimBlockStall", Type::Bool, "First section stays local.").Group("Demand loading"),
    F("noStallFrames", Type::Number, "Frames that never stall.").Group("Demand loading"),
};
const Item kClipItem = MakeItem("clip", kClip, &kMotionControlsItem);

const Field kAnimation[] = {
    F("name", Type::String, "Animation name.").Declares("animation").Required(),
    F("file", Type::File, "Source file."),
    F("bindPose", Type::Bool, "The compile's own bind pose instead of a file."),
    F("declare", Type::Bool, "An empty slot an included model fills."),
    F("snap", Type::Bool, "Snap."),
    F("post", Type::Bool, "Post."),
};
const Item* const kAnimationKids[] = {&kIkRuleItem, &kAnimOpItem, nullptr};
const Item kAnimationItem =
    MakeItem("animation", kAnimation, &kClipItem).Titled("Animation", "Animation").Holds(kAnimationKids);
const Item* const kAnimationsKids[] = {&kAnimationItem, nullptr};
const Item kAnimationsItem = MakeItem("animations").Titled("Animations", "Animation").Holds(kAnimationsKids);

const Field kBlend[] = {
    F("parameter", Type::String, "Pose parameter.").Ref("poseParameter").Required(),
    F("min", Type::Number, "Start value.").Required(),
    F("max", Type::Number, "End value.").Required(),
};
const Item kBlendItem = MakeItem("blend", kBlend).Titled("Blend Axis", "Sequences");

const Field kAddLayer[] = {
    F("sequence", Type::String, "Layered sequence.").Ref("sequence").Required(),
    F("local", Type::Bool, "Local layer."),
};
const Item kAddLayerItem = MakeItem("addLayer", kAddLayer).Titled("Add Layer", "Sequences");

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
const Item kBlendLayerItem = MakeItem("blendLayer", kBlendLayer).Titled("Blend Layer", "Sequences");

const Field kIkLock[] = {
    F("chain", Type::String, "IK chain.").Ref("ikChain").Required(),
    F("position", Type::Number, "Position weight.").Required(),
    F("rotation", Type::Number, "Rotation weight.").Required(),
};
const Item kIkLockItem = MakeItem("ikLock", kIkLock).Titled("IK Lock", "Sequences");

const Field kEvent[] = {
    F("name", Type::String, "Event name or numeric id.").Required(),
    F("frame", Type::Integer, "Frame.").Required(),
    F("options", Type::String, "Option string the game reads."),
};
const Item kEventItem = MakeItem("event", kEvent).Titled("Event", "Sequences");

const Field kAnimTag[] = {
    F("name", Type::String, "Tag name.").Required(),
    F("cycle", Type::Number, "Cycle fraction.").Required(),
};
const Item kAnimTagItem = MakeItem("animTag", kAnimTag).Titled("Animation Tag", "Sequences");

const Field kSequence[] = {
    F("name", Type::String, "Sequence name.").Declares("sequence").Required(),
    F("file", Type::File, "Source file of the only clip; Blend File or Blend Animation nodes make a grid."),
    F("bindPose", Type::Bool, "The compile's own bind pose instead of a file."),
    F("declare", Type::Bool, "An empty slot an included model fills."),
    F("blendWidth", Type::Integer, "Blend grid width."),
    F("activity", Type::String, "Activity, e.g. ACT_IDLE.").Group("Activity"),
    F("activityWeight", Type::Integer, "Activity weight.").Group("Activity"),
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
    F("node", Type::String, "Transition node.").Group("Transition"),
    F("transitionFrom", Type::String, "Transition entry node.").Group("Transition"),
    F("transitionTo", Type::String, "Transition exit node.").Group("Transition"),
    F("transitionReversible", Type::Bool, "The transition is reversible.").Group("Transition"),
};
const Item* const kSequenceKids[] = {&kBlendFileItem, &kBlendAnimationItem, &kBlendItem, &kAddLayerItem,
                                     &kBlendLayerItem, &kIkLockItem, &kEventItem, &kAnimTagItem, &kIkRuleItem,
                                     &kAnimOpItem, &kActivityModifierItem, &kKeyValueItem, nullptr};
const Item kSequenceItem =
    MakeItem("sequence", kSequence, &kClipItem).Titled("Sequence", "Sequences").Holds(kSequenceKids);
const Item* const kSequencesKids[] = {&kSequenceItem, nullptr};
const Item kSequencesItem = MakeItem("sequences").Titled("Sequences", "Sequences").Holds(kSequencesKids);

// --- model-wide animation settings ---

const Field kWeight[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("weight", Type::Number, "Weight.").Required(),
    F("posWeight", Type::Number, "Position weight (the weight when omitted)."),
};
const Item kWeightItem = MakeItem("weight", kWeight).Titled("Bone Weight", "Animation");
const Item kDefaultWeightItem = MakeItem("defaultWeight", kWeight).Titled("Default Weight", "Animation");

const Field kWeightList[] = {
    F("name", Type::String, "List name.").Declares("weightList").Required(),
};
const Item* const kWeightListKids[] = {&kWeightItem, nullptr};
const Item kWeightListItem =
    MakeItem("weightList", kWeightList).Titled("Weight List", "Animation").Holds(kWeightListKids, 1);
const Item* const kWeightListsKids[] = {&kWeightListItem, nullptr};
const Item kWeightListsItem = MakeItem("weightLists").Titled("Weight Lists", "Animation").Holds(kWeightListsKids);

const Field kAnimBlocks[] = {
    F("size", Type::Integer, "Block size; under 1024 reads as kilobytes.").Required(),
    F("noStall", Type::Bool, "No stall."),
    F("highRes", Type::Bool, "High resolution cached frames."),
    F("lowRes", Type::Bool, "Low resolution cached frames."),
    F("zeroFrames", Type::Integer, "Cached frame count (1-4)."),
    F("cacheHighRes", Type::Bool, "Cache high resolution."),
    F("posDelta", Type::Number, "Position delta."),
};
const Item kAnimBlocksItem = MakeItem("animBlocks", kAnimBlocks).Titled("Animation Blocks", "Animation").Single();

const Field kBoneSaveFrame[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("position", Type::Bool, "Save position."),
    F("rotation", Type::Bool, "Save rotation."),
    F("rotation64", Type::Bool, "Save 64-bit rotation."),
};
const Item kBoneSaveFrameItem = MakeItem("boneSaveFrame", kBoneSaveFrame).Titled("Bone Save Frame", "Animation");

const Item* const kSkipTransitionKids[] = {&kTransitionNodeItem, nullptr};
const Item kSkipTransitionItem =
    MakeItem("skipTransition").Titled("Skip Transition", "Animation").Holds(kSkipTransitionKids, 1);
const Item kAutoplayLockItem = MakeItem("autoplayLock", kIkLock).Titled("Autoplay IK Lock", "Animation");

const Field kAnimationSettings[] = {
    F("defaultFps", Type::Number, "Frame rate of new clips."),
    F("motionRollback", Type::Number, "Motion rollback of new clips."),
    F("defaultFadeIn", Type::Number, "Fade-in of new sequences."),
    F("defaultFadeOut", Type::Number, "Fade-out of new sequences."),
    F("lowercaseSequences", Type::Bool, "Lowercase sequence names."),
    F("cullMethod", Type::Enum, "Which animations reach the .mdl.").Values(kAnimCulls),
    F("calcTransitions", Type::Bool, "Fill multi-stage transition routes."),
    F("sectionFrames", Type::Integer, "Compression section length in frames.").Group("Sections"),
    F("sectionMinLimit", Type::Integer, "Minimum frame limit.").Group("Sections"),
};
const Item* const kAnimationSettingsKids[] = {&kAnimBlocksItem, &kAllowedActivityItem, &kDefaultWeightItem,
                                              &kAutoplayLockItem, &kBoneSaveFrameItem, &kSkipTransitionItem,
                                              nullptr};
const Item kAnimationSettingsItem = MakeItem("animationSettings", kAnimationSettings)
                                        .Titled("Animation Settings", "Animation")
                                        .Holds(kAnimationSettingsKids);

// --- flex and face ---

const Field kDataModelFlexes[] = {
    F("file", Type::File, "DMX holding the flex rig.").Required(),
    F("controllers", Type::Bool, "Import its flex controllers."),
    F("correctives", Type::Bool, "Import its correctives (needs controllers)."),
    F("dominators", Type::Bool, "Import its domination rules (needs correctives)."),
    F("rules", Type::Bool, "Import its flex rules."),
};
const Item kDataModelFlexesItem = MakeItem("dataModelFlexes", kDataModelFlexes).Titled("DMX Flex Rig", "Flex");

const Field kFlexController[] = {
    F("name", Type::String, "Controller name.").Declares("flexController").Required(),
    F("group", Type::String, "Controller group.").Required(),
    F("min", Type::Number, "Range start (0 when omitted)."),
    F("max", Type::Number, "Range end (1 when omitted)."),
};
const Item kFlexControllerItem = MakeItem("flexController", kFlexController).Titled("Flex Controller", "Flex");

const Field kFlexRule[] = {
    F("name", Type::String, "Morph or local variable the rule drives.").Ref("morph").Required(),
    F("expression", Type::String, "Rule expression: controller names, %morph, numbers, operators.").Required(),
};
const Item kFlexRuleItem = MakeItem("flexRule", kFlexRule).Titled("Flex Rule", "Flex");

const Field kFlexCorrective[] = {
    F("name", Type::String, "Corrective morph; its Control nodes must all be up.").Ref("morph").Required(),
};
const Item* const kFlexCorrectiveKids[] = {&kCorrectiveControlItem, nullptr};
const Item kFlexCorrectiveItem =
    MakeItem("flexCorrective", kFlexCorrective).Titled("Corrective", "Flex").Holds(kFlexCorrectiveKids, 1);

const Field kFlexDomination[] = {
    F("name", Type::String, "Morph its Dominator nodes suppress.").Ref("morph").Required(),
};
const Item* const kFlexDominationKids[] = {&kDominatorItem, nullptr};
const Item kFlexDominationItem =
    MakeItem("flexDomination", kFlexDomination).Titled("Domination Rule", "Flex").Holds(kFlexDominationKids, 1);

const Field kStereoSplit[] = {
    F("morph", Type::String, "Morph split into <name>L/<name>R.").Ref("morph").Required(),
    F("splitFactor", Type::Number, "Balance from vertex X when the mesh has none; 0 = mesh balance."),
};
const Item kStereoSplitItem = MakeItem("stereoSplit", kStereoSplit).Titled("Stereo Split", "Flex");

const Field kMorphRename[] = {
    F("morph", Type::String, "Source morph.").Ref("morph").Required(),
    F("to", Type::String, "New name.").Required(),
};
const Item kMorphRenameItem = MakeItem("morphRename", kMorphRename).Titled("Morph Rename", "Flex");

const Field kMorphValue[] = {
    F("morph", Type::String, "Morph.").Ref("morph").Required(),
    F("value", Type::Number, "Amount.").Required(),
};
const Item kFixedMorphItem = MakeItem("fixedMorph", kMorphValue).Titled("Fixed Morph", "Flex");
const Item kWrinkleScaleItem = MakeItem("wrinkleScale", kMorphValue).Titled("Wrinkle Scale", "Flex");

const Field kBoneFlexDriver[] = {
    F("bone", Type::String, "Driving bone.").Ref("bone").Required(),
    F("component", Type::Enum, "Translation axis.").Values(kTranslations).Required(),
    F("controller", Type::String, "Driven flex controller.").Ref("flexController").Required(),
    F("min", Type::Number, "Translation at controller 0.").Required(),
    F("max", Type::Number, "Translation at controller 1.").Required(),
};
const Item kBoneFlexDriverItem = MakeItem("boneFlexDriver", kBoneFlexDriver).Titled("Bone Flex Driver", "Flex");

const Field kFlex[] = {
    F("cullMethod", Type::Enum, "How hard unused flex data is pruned.").Values(kFlexCulls),
};
const Item* const kFlexKids[] = {&kDataModelFlexesItem, &kFlexControllerItem, &kLocalVarItem, &kFlexRuleItem,
                                 &kFlexCorrectiveItem, &kFlexDominationItem, &kStereoSplitItem,
                                 &kMorphRenameItem, &kFixedMorphItem, &kWrinkleScaleItem,
                                 &kBoneFlexDriverItem, nullptr};
const Item kFlexItem = MakeItem("flex", kFlex).Titled("Flex", "Flex").Holds(kFlexKids);

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
const Item kEyeballItem = MakeItem("eyeball", kEyeball).Titled("Eyeball", "Face");

const Field kMouth[] = {
    F("controller", Type::String, "Flex controller.").Ref("flexController").Required(),
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("forward", Type::Vec3, "Forward direction.").Required(),
};
const Item kMouthItem = MakeItem("mouth", kMouth).Titled("Mouth", "Face");

const Field kEyelid[] = {
    F("lid", Type::Enum, "Lid.").Values(kLids).Required(),
    F("split", Type::Number, "Mask deltas to one side; sign picks the side."),
    F("rightEyeball", Type::String, "Right eyeball (paired form).").Ref("eyeball"),
    F("leftEyeball", Type::String, "Left eyeball (paired form).").Ref("eyeball"),
    F("eyeball", Type::String, "One eyeball (per-eye form, needs flexDesc).").Ref("eyeball"),
    F("flexDesc", Type::String, "Lid flexdesc name (per-eye form)."),
    F("lowererTarget", Type::Number, "Lowered lid target.").Group("Lowered").Required(),
    F("lowererDelta", Type::String, "Lowered delta state; omitted = no vertex data.").Ref("morph").Group("Lowered"),
    F("neutralTarget", Type::Number, "Neutral lid target.").Group("Neutral").Required(),
    F("neutralDelta", Type::String, "Neutral delta state; omitted = no vertex data.").Ref("morph").Group("Neutral"),
    F("raiserTarget", Type::Number, "Raised lid target.").Group("Raised").Required(),
    F("raiserDelta", Type::String, "Raised delta state; omitted = no vertex data.").Ref("morph").Group("Raised"),
};
const Item kEyelidItem = MakeItem("eyelid", kEyelid).Titled("Eyelid", "Face");

const Item* const kFaceKids[] = {&kEyeballItem, &kMouthItem, &kEyelidItem, nullptr};
const Item kFaceItem = MakeItem("face").Titled("Face", "Face").Holds(kFaceKids);

// --- physics ---

const Field kPhysicsShape[] = {
    F("file", Type::File, "Collision source file."),
    F("renderMesh", Type::String, "Generate from this render mesh.").Ref("mesh"),
    F("fromRender", Type::Bool, "Generate from the whole render model."),
    F("parentBone", Type::String, "Bone the shape belongs to.").Ref("bone"),
    F("offsetOrigin", Type::Vec3, "Translation."),
    F("offsetAngles", Type::Vec3, "Rotation."),
    F("importScale", Type::Number, "Scale."),
    F("maxConvex", Type::Integer, "Convex piece limit."),
    F("importType", Type::Enum, "File shapes: how parts are bound to bones.").Values(kImportTypes).Group("File"),
    F("concave", Type::Bool, "File shapes: keep concave pieces.").Group("File"),
    F("remove2d", Type::Bool, "File shapes: drop flat pieces.").Group("File"),
    F("decimationFactor", Type::Number, "Generated shapes: decimation.").Group("Generated"),
    F("cullWeight", Type::Number, "Generated shapes: bone weight cull (0-1).").Group("Generated"),
    F("concavity", Type::Number, "Generated shapes: concavity.").Group("Generated"),
    F("maxHulls", Type::Integer, "Generated shapes: hull limit.").Group("Generated"),
    F("maxDepth", Type::Integer, "Generated shapes: split depth (0-10).").Group("Generated"),
    F("primitive", Type::Enum, "A box, sphere or capsule hull instead of a file or mesh.").Values(kPrimitives)
        .Group("Primitive"),
    F("bone", Type::String, "Bone the primitive is on; values are in its space.").Ref("bone").Group("Primitive"),
    F("min", Type::Vec3, "Box minimum.").Group("Primitive"),
    F("max", Type::Vec3, "Box maximum.").Group("Primitive"),
    F("angles", Type::Vec3, "Box orientation.").Group("Primitive"),
    F("center", Type::Vec3, "Sphere center.").Group("Primitive"),
    F("radius", Type::Number, "Sphere radius.").Group("Primitive"),
    F("point0", Type::Vec3, "Capsule start.").Group("Primitive"),
    F("point1", Type::Vec3, "Capsule end.").Group("Primitive"),
    F("radius0", Type::Number, "Capsule start radius.").Group("Primitive"),
    F("radius1", Type::Number, "Capsule end radius.").Group("Primitive"),
    F("segments", Type::Integer, "Sphere/capsule tessellation.").Group("Primitive"),
};
const Item* const kPhysicsShapeKids[] = {&kExtraBoneItem, &kExcludeMeshItem, nullptr};
const Item kPhysicsShapeItem =
    MakeItem("physicsShape", kPhysicsShape).Titled("Collision Shape", "Physics").Holds(kPhysicsShapeKids);

const Field kJointAxis[] = {
    F("axis", Type::Enum, "Axis.").Values(kAxes).Required(),
    F("motion", Type::Enum, "Motion.").Values(kJointTypes).Required(),
    F("min", Type::Number, "Limit minimum in degrees."),
    F("max", Type::Number, "Limit maximum in degrees."),
    F("friction", Type::Number, "Friction (1 when omitted)."),
};
const Item kJointAxisItem = MakeItem("jointAxis", kJointAxis).Titled("Joint Axis", "Physics");

const Field kPhysicsJoint[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
};
const Item* const kPhysicsJointKids[] = {&kJointAxisItem, nullptr};
const Item kPhysicsJointItem =
    MakeItem("physicsJoint", kPhysicsJoint).Titled("Ragdoll Joint", "Physics").Holds(kPhysicsJointKids, 1);

const Field kPhysicsMarkup[] = {
    F("bone", Type::String, "Bone.").Ref("bone").Required(),
    F("massBias", Type::Number, "Mass bias."),
    F("inertia", Type::Number, "Inertia."),
    F("damping", Type::Number, "Damping."),
    F("rotDamping", Type::Number, "Rotation damping."),
    F("skip", Type::Bool, "No body for this bone."),
    F("mergeInto", Type::String, "Merge this body into another bone's.").Ref("bone"),
};
const Item kPhysicsMarkupItem = MakeItem("physicsMarkup", kPhysicsMarkup).Titled("Body Override", "Physics");

const Field kBonePair[] = {
    F("a", Type::String, "First bone.").Ref("bone").Required(),
    F("b", Type::String, "Second bone.").Ref("bone").Required(),
};
const Item kBonePairItem = MakeItem("collisionPair", kBonePair).Titled("Collision Pair", "Physics");

const Field kJointContents[] = {
    F("bone", Type::String, "Bone; its children inherit.").Ref("bone").Required(),
};
const Item kJointContentsItem =
    MakeItem("jointContents", kJointContents, &kContentsFlagsItem).Titled("Joint Contents", "Physics");

const Field kJointSurfaceProp[] = {
    F("bone", Type::String, "Bone; its children inherit.").Ref("bone").Required(),
    F("surfaceProp", Type::String, "Surface property.").Required(),
};
const Item kJointSurfacePropItem =
    MakeItem("jointSurfaceProp", kJointSurfaceProp).Titled("Joint Surface", "Physics");

const Field kPhysics[] = {
    F("dataModel", Type::File, "DMX whose physics shapes are imported."),
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
    F("frictionMin", Type::Integer, "Minimum friction.").Group("Animated friction"),
    F("frictionMax", Type::Integer, "Maximum friction.").Group("Animated friction"),
    F("frictionTimeIn", Type::Number, "Ramp-in time.").Group("Animated friction"),
    F("frictionTimeOut", Type::Number, "Ramp-out time.").Group("Animated friction"),
    F("frictionTimeHold", Type::Number, "Hold time.").Group("Animated friction"),
};
const Item* const kPhysicsKids[] = {&kPhysicsShapeItem, &kPhysicsJointItem, &kPhysicsMarkupItem, &kBonePairItem,
                                    &kJointContentsItem, &kJointSurfacePropItem, &kKeyValueItem, nullptr};
const Item kPhysicsItem = MakeItem("physics", kPhysics).Titled("Physics", "Physics").Holds(kPhysicsKids);

// --- files ---

const Field kContent[] = {
    F("settings", Type::Object, "Model settings.").Of(&kSettingsItem),
    F("variables", Type::Object, "Variables, written $name$ in any string.").Of(&kVariablesItem),
    F("prefabs", Type::Object, "Other .pulsemodel files merged in.").Of(&kPrefabsItem),
    F("meshes", Type::Object, "Render meshes.").Of(&kMeshesItem),
    F("modelGroups", Type::Object, "Model groups (bodygroups) and presets.").Of(&kModelGroupsItem),
    F("materials", Type::Object, "Materials and skins.").Of(&kMaterialsItem),
    F("lods", Type::Object, "LODs, in switch order.").Of(&kLodsItem),
    F("bones", Type::Object, "Skeleton edits.").Of(&kBonesItem),
    F("attachments", Type::Object, "Attachments.").Of(&kAttachmentsItem),
    F("hitboxes", Type::Object, "Hitbox sets and hit groups.").Of(&kHitboxesItem),
    F("physics", Type::Object, "Collision model and ragdoll.").Of(&kPhysicsItem),
    F("face", Type::Object, "Eyeballs, mouths and eyelids.").Of(&kFaceItem),
    F("flex", Type::Object, "Flex controllers, rules and morph edits.").Of(&kFlexItem),
    F("poseParameters", Type::Object, "Pose parameters.").Of(&kPoseParametersItem),
    F("ik", Type::Object, "IK chains.").Of(&kIkItem),
    F("animationSettings", Type::Object, "Model-wide animation settings.").Of(&kAnimationSettingsItem),
    F("weightLists", Type::Object, "Named bone weight lists.").Of(&kWeightListsItem),
    F("animations", Type::Object, "Named animations.").Of(&kAnimationsItem),
    F("sequences", Type::Object, "Sequences, in index order.").Of(&kSequencesItem),
};
const Item kContentItem = MakeItem("content", kContent);

const Field kModel[] = {
    F("id", Type::String, "Model id, unique in the file.").Required(),
    F("compile", Type::Bool, "Compiled by default (true when omitted)."),
};
const Item kModelItem = MakeItem("modelEntry", kModel, &kContentItem).Titled("Model", "Model");

const Field kRoot[] = {
    F("format", Type::String, "Always \"pulsemodel\".").Required(),
    F("version", Type::Integer, "Format version.").Required(),
    F("shared", Type::Object, "Content merged into every model.").Of(&kContentItem),
    F("models", Type::List, "Models.").Of(&kModelItem).Required(),
};
const Item kRootItem = MakeItem("file", kRoot);

const Item* const kAllItems[] = {
    &kRootItem, &kModelItem, &kContentItem, &kKeyValueItem, &kSettingsItem, &kVariableItem,
    &kVariablesItem, &kPrefabItem, &kPrefabsItem, &kSkinReplaceItem, &kSkinItem, &kMaterialRenameItem,
    &kMaterialCollapseItem, &kMaterialsItem, &kScaleBoneItem, &kMeshItem, &kMeshesItem, &kChoiceItem,
    &kModelGroupItem, &kPresetChoiceItem, &kModelGroupPresetItem, &kModelGroupsItem, &kLodModelItem,
    &kLodDecimateItem, &kLodBoneItem, &kLodMaterialItem, &kLodWordItem, &kLodItem, &kLodsItem,
    &kDefineBoneItem, &kRenameItem, &kTransformBoneItem, &kRealignBoneItem, &kHierarchyItem, &kJiggleCommonItem,
    &kJiggleFlexibleItem, &kJiggleRigidItem, &kJiggleBaseSpringItem, &kJiggleBoingItem, &kJiggleBoneItem,
    &kDriverTriggerItem, &kDriverBoneItem, &kAimAtItem, &kDataModelJointsItem, &kBonesItem,
    &kAttachmentItem, &kAttachmentsItem, &kHitboxItem, &kHitboxSetItem, &kHitGroupItem, &kHitboxesItem,
    &kPoseParameterItem, &kPoseParametersItem, &kIkChainItem, &kIkItem, &kIkRuleItem, &kIkFixupRuleItem,
    &kAnimOpItem, &kClipItem, &kAnimationItem, &kAnimationsItem, &kBlendItem, &kAddLayerItem,
    &kBlendLayerItem, &kIkLockItem, &kEventItem, &kAnimTagItem, &kSequenceItem, &kSequencesItem,
    &kWeightItem, &kDefaultWeightItem, &kWeightListItem, &kWeightListsItem, &kAnimBlocksItem,
    &kBoneSaveFrameItem, &kSkipTransitionItem, &kAutoplayLockItem, &kAnimationSettingsItem,
    &kDataModelFlexesItem, &kFlexControllerItem, &kFlexRuleItem, &kFlexCorrectiveItem, &kFlexDominationItem,
    &kStereoSplitItem, &kMorphRenameItem, &kFixedMorphItem, &kWrinkleScaleItem, &kBoneFlexDriverItem,
    &kFlexItem, &kEyeballItem, &kMouthItem, &kEyelidItem, &kFaceItem, &kPhysicsShapeItem, &kJointAxisItem,
    &kPhysicsJointItem, &kPhysicsMarkupItem, &kBonePairItem, &kJointContentsItem, &kJointSurfacePropItem,
    &kPhysicsItem, &kContentsFlagsItem, &kMotionControlsItem, &kBoneMarkupItem, &kBoundsMeshItem,
    &kSearchFolderItem, &kIncludeModelItem, &kMaterialFolderItem, &kDrawOrderItem, &kDropMaterialItem,
    &kDropMaterialWordItem, &kChoiceMeshItem, &kRemoveMeshItem, &kSmallMeshItem, &kCollapseTreeItem,
    &kVrdFileItem, &kPlaceFlexGroupItem, &kPlaceMorphItem, &kPlaceMaterialItem, &kBlendFileItem,
    &kBlendAnimationItem, &kActivityModifierItem, &kLocalVarItem, &kCorrectiveControlItem, &kDominatorItem,
    &kExtraBoneItem, &kExcludeMeshItem, &kTransitionNodeItem, &kAllowedActivityItem,
};

const Field* FindField(const Item& item, const std::string& key) {
    for (const Item* i = &item; i; i = i->base)
        for (size_t f = 0; f < i->count; ++f)
            if (key == i->fields[f].key) return &i->fields[f];
    return nullptr;
}

// The node type `name` when `parent` may hold it.
const Item* AllowedChild(const Item& parent, const std::string& name) {
    if (!parent.children) return nullptr;
    for (const Item* const* kind = parent.children; *kind; ++kind)
        if (name == (*kind)->name) return *kind;
    return nullptr;
}

const Item& ItemOf(const Item& parent, const Json& node) {
    return *AllowedChild(parent, node.at("type").get<std::string>());
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
bool CheckObject(Loader& l, size_t file, const std::string& path, const Item& item, Json& obj,
                 bool node = false);

// Each child names its type; the parent's schema decides which types fit.
bool CheckChildren(Loader& l, size_t file, const std::string& path, const Item& parent, Json& list) {
    if (!list.is_array()) return l.Fail(file, path, "expected an array of nodes");

    std::set<std::string> singles;
    for (size_t i = 0; i < list.size(); ++i) {
        Json& child = list[i];
        const std::string at = Pointer(path, i);
        if (!child.is_object()) return l.Fail(file, at, "expected a node object");
        const auto type = child.find("type");
        if (type == child.end() || !type->is_string()) return l.Fail(file, at, "missing \"type\"");

        const std::string& name = type->get_ref<const std::string&>();
        const Item* item = AllowedChild(parent, name);
        if (!item)
            return l.Fail(file, Pointer(at, "type"), "a \"" + name + "\" node does not fit in " + parent.name);
        if (item->single && !singles.insert(item->name).second)
            return l.Fail(file, at, std::string("only one ") + item->title + " fits in " + parent.name);
        if (!CheckObject(l, file, at, *item, child, true)) return false;
    }
    return true;
}

bool CheckObject(Loader& l, size_t file, const std::string& path, const Item& item, Json& obj, bool node) {
    if (!obj.is_object())
        return l.Fail(file, path, std::string("expected an object (") + item.name + ")");

    for (auto& [key, value] : obj.items()) {
        if (key == "editor" || (node && key == "type")) continue;
        if (node && key == "enabled") {
            if (!value.is_boolean()) return l.Fail(file, Pointer(path, key), "expected true or false");
            continue;
        }
        if (key == "children" && item.children) {
            if (!CheckChildren(l, file, Pointer(path, key), item, value)) return false;
            continue;
        }
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

// Drops nodes switched off with "enabled": false, then checks the child counts
// the rest of the load relies on.
bool DropDisabled(Loader& l, const Item& item, Json& obj) {
    for (auto& [key, value] : obj.items()) {
        if (IsInternal(key) || key == "editor") continue;
        if (key == "children") {
            Json kept = Json::array();
            for (Json& child : value) {
                const auto enabled = child.find("enabled");
                if (enabled != child.end() && !enabled->get<bool>()) continue;
                if (!DropDisabled(l, ItemOf(item, child), child)) return false;
                kept.push_back(std::move(child));
            }
            value = std::move(kept);
            continue;
        }

        const Field* field = FindField(item, key);
        if (!field) continue;
        if (field->type == Type::Object && !DropDisabled(l, *field->item, value)) return false;
        if (field->type == Type::List)
            for (Json& entry : value)
                if (!DropDisabled(l, *field->item, entry)) return false;
    }

    const auto children = obj.find("children");
    const size_t count = children == obj.end() ? 0 : children->size();
    if (count < item.minChildren)
        return l.Fail(obj, "", std::string("a ") + item.title + " needs at least one enabled " +
                               item.children[0]->title);
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
    if (version->get<int>() < kFormatVersion)
        return l.Fail(file, "/version", "format " + std::to_string(version->get<int>()) +
                                        " is no longer read - this PulseModel reads format " +
                                        std::to_string(kFormatVersion)), -1;

    if (!CheckObject(l, file, "", kRootItem, doc) || !DropDisabled(l, kRootItem, doc)) return -1;
    l.docs.resize(l.files.size());
    l.docs[file] = std::move(doc);
    return static_cast<int>(file);
}

// ---------------------------------------------------------------------------
// Variables - the .pulseqc rules: $name$ inside any string, case-sensitive,
// a variable node defines a name, one with `redefine` only changes it.
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
// Variable nodes run in order; a redefine changes an earlier value, any other defines one.
bool DefineVariables(Loader& l, const Json& owner, const std::set<std::string>& fixed, Scope& scope) {
    const auto section = owner.find("variables");
    if (section == owner.end() || !section->contains("children")) return true;

    for (const Json& node : section->at("children")) {
        const std::string name = node.at("name").get<std::string>();
        const Json& value = node.at("value");
        const bool redefine = node.value("redefine", false);
        const Json at = KeyOrigin(node, "value");
        const size_t file = at[0].get<size_t>();
        const std::string path = at[1].get<std::string>();
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
                                          "set redefine to change it");
            current = text;
            found = true;
        }
        if (redefine && !found)
            return l.Fail(file, path, "variable \"" + name + "\" is not defined - "
                                      "clear redefine to define it");
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
        if (key == "children") {
            for (Json& child : value)
                if (!Substitute(l, ItemOf(item, child), scope, child)) return false;
            continue;
        }
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
        case Type::Any:
            break;
        default:
            if (!SubstituteValue(l, file, path, *field, scope, value)) return false;
            if (field->type == Type::File && source.prefab)
                value = PrefabFile(source.dir, value.get<std::string>(), key == "folder");
            break;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Prefabs and merging
// ---------------------------------------------------------------------------

bool CollectContent(Loader& l, Json part, const Scope& scope, std::vector<Json>& out);

// Loads the file one prefab node names and appends its content.
bool IncludePrefab(Loader& l, const Json& entry, const Scope& callerScope, std::vector<Json>& out) {
    const Json origin = KeyOrigin(entry, "");
    const size_t file = origin[0].get<size_t>();
    const std::string at = origin[1].get<std::string>();

    std::string name = entry.at("file").get<std::string>();
    if (!Expand(l, file, Pointer(at, "file"), callerScope, name)) return false;
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
    if (entry.contains("variables")) {
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
    if (!shared.is_null() &&
        (!DefineVariables(l, shared, fixed, scope) || !CollectContent(l, shared, scope, out)))
        return false;
    if (!model.is_null()) {
        model.erase("id");
        model.erase("compile");
        if (!DefineVariables(l, model, fixed, scope) || !CollectContent(l, model, scope, out))
            return false;
    }
    l.prefabStack.pop_back();
    return true;
}

// Substitutes one content object and appends it, followed by the prefabs it includes.
bool CollectContent(Loader& l, Json part, const Scope& scope, std::vector<Json>& out) {
    Json prefabs = Json::array();
    if (const auto section = part.find("prefabs"); section != part.end() && section->contains("children"))
        prefabs = section->at("children");
    part.erase("variables");
    part.erase("prefabs");
    if (!Substitute(l, kContentItem, scope, part)) return false;
    out.push_back(std::move(part));

    for (const Json& prefab : prefabs)
        if (!IncludePrefab(l, prefab, scope, out)) return false;
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
        if (key == "children") {
            for (const Json& child : value) dst[key].push_back(child);
            continue;
        }

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

    if (const Json* list = Get(b, "realignDirectives")) {
        for (const Json& edit : *list) {
            const bool disable = True(edit, "noRealign");
            e.Command(disable ? "$norealign" : "$realignbone", edit);
            e.Text(Str(edit, "bone"));
            if (disable) continue;

            EmitClause(e, edit, "origin", "origin");
            EmitClause(e, edit, "angles", "angles");
            EmitFlag(e, edit, "worldAxes", "worldaxes");
            EmitFlag(e, edit, "absolute", "absolute");
            EmitFlag(e, edit, "ignoreAnimation", "ignoreanimation");
            EmitFlag(e, edit, "affectChildren", "affectchildren");
        }
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
    e.Command("$hboxset", set);
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

// ---------------------------------------------------------------------------
// Reshaping - the merged node tree becomes the record layout the lowering reads:
// child nodes are gathered into lists by type and grouped fields into the
// objects they stand for. Every object keeps the origin of the node it came from.
// ---------------------------------------------------------------------------

enum class Into { List, Object, Model };

// Where a child node of `parent` goes: a list or object on the parent, or a model-level list.
struct Move {
    const char* parent;
    const char* child;
    const char* key;
    Into into = Into::List;
};

const Move kMoves[] = {
    {"keyValue", "keyValue", "block"},
    {"settings", "keyValue", "keyValues"},
    {"skin", "skinReplace", "replace"},
    {"materials", "skin", "skins"},
    {"materials", "materialRename", "renames"},
    {"materials", "materialCollapse", "collapse"},
    {"mesh", "scaleBone", "scaleBones"},
    {"meshes", "mesh", "meshes", Into::Model},
    {"modelGroup", "choice", "choices"},
    {"modelGroupPreset", "presetChoice", "choices"},
    {"modelGroups", "modelGroup", "modelGroups", Into::Model},
    {"modelGroups", "modelGroupPreset", "modelGroupPresets", Into::Model},
    {"lod", "lodReplaceMesh", "replaceModels"},
    {"lod", "lodDecimateMesh", "decimateModels"},
    {"lod", "lodReplaceBone", "replaceBones"},
    {"lod", "lodReplaceMaterial", "replaceMaterials"},
    {"lod", "lodReplaceMaterialWord", "replaceMaterialWords"},
    {"lods", "lod", "lods", Into::Model},
    {"jiggleBone", "jiggleFlexible", "flexible", Into::Object},
    {"jiggleBone", "jiggleRigid", "rigid", Into::Object},
    {"jiggleBone", "jiggleBaseSpring", "baseSpring", Into::Object},
    {"jiggleBone", "jiggleBoing", "boing", Into::Object},
    {"driverBone", "driverTrigger", "triggers"},
    {"bones", "defineBone", "define"},
    {"bones", "renameBone", "renames"},
    {"bones", "transformBone", "transforms"},
    {"bones", "realignBone", "realignDirectives"},
    {"bones", "reparentBone", "hierarchy"},
    {"bones", "jiggleBone", "jiggle"},
    {"bones", "driverBone", "drivers"},
    {"bones", "aimAtBone", "aimAt"},
    {"bones", "dataModelJoints", "dataModelJoints", Into::Model},
    {"attachments", "attachment", "attachments", Into::Model},
    {"hitboxSet", "hitbox", "boxes"},
    {"hitboxes", "hitboxSet", "hitboxSets", Into::Model},
    {"hitboxes", "hitGroup", "hitGroups", Into::Model},
    {"poseParameters", "poseParameter", "poseParameters", Into::Model},
    {"ik", "ikChain", "ikChains", Into::Model},
    {"animationOperation", "ikFixupRule", "ikRule", Into::Object},
    {"clip", "ikRule", "ikRules"},
    {"clip", "animationOperation", "operations"},
    {"sequence", "blend", "blends"},
    {"sequence", "addLayer", "addLayers"},
    {"sequence", "blendLayer", "blendLayers"},
    {"sequence", "ikLock", "ikLocks"},
    {"sequence", "event", "events"},
    {"sequence", "animTag", "animTags"},
    {"sequence", "keyValue", "keyValues"},
    {"animations", "animation", "animations", Into::Model},
    {"sequences", "sequence", "sequences", Into::Model},
    {"weightList", "weight", "bones"},
    {"weightLists", "weightList", "weightLists", Into::Model},
    {"animationSettings", "animBlocks", "animBlocks", Into::Object},
    {"animationSettings", "defaultWeight", "defaultWeights"},
    {"animationSettings", "autoplayLock", "autoplayLocks"},
    {"animationSettings", "boneSaveFrame", "boneSaveFrames"},
    {"animationSettings", "skipTransition", "skipTransitions"},
    {"flex", "dataModelFlexes", "dataModel"},
    {"flex", "flexController", "controllers"},
    {"flex", "flexRule", "rules"},
    {"flex", "flexCorrective", "correctives"},
    {"flex", "flexDomination", "dominations"},
    {"flex", "stereoSplit", "stereoSplits"},
    {"flex", "morphRename", "renames"},
    {"flex", "fixedMorph", "fixed"},
    {"flex", "wrinkleScale", "wrinkleScales"},
    {"flex", "boneFlexDriver", "boneDrivers"},
    {"face", "eyeball", "eyeballs"},
    {"face", "mouth", "mouths"},
    {"face", "eyelid", "eyelids"},
    {"physicsJoint", "jointAxis", "axes"},
    {"physics", "physicsShape", "shapes"},
    {"physics", "physicsJoint", "joints"},
    {"physics", "physicsMarkup", "markups"},
    {"physics", "collisionPair", "collisions"},
    {"physics", "jointContents", "jointContents"},
    {"physics", "jointSurfaceProp", "jointSurfaceProps"},
    {"physics", "keyValue", "collisionText"},
};

// A field of `item` that is one key of a grouped object. Without `sub`, the field
// is a switch: true makes the object, false keeps it out.
struct Flat {
    const char* item;
    const char* key;
    const char* object;
    const char* sub;
    bool required = false;
};

const Flat kFlats[] = {
    {"settings", "origin", "transform", "origin"},
    {"settings", "angles", "transform", "angles"},
    {"settings", "scale", "transform", "scale"},
    {"settings", "upAxis", "transform", "upAxis"},
    {"settings", "maxBones", "budget", "bones"},
    {"settings", "maxMaterials", "budget", "materials"},
    {"settings", "bboxMin", "bbox", "min"},
    {"settings", "bboxMax", "bbox", "max"},
    {"settings", "bboxMeshes", "bbox", "meshes"},
    {"settings", "cboxMin", "cbox", "min", true},
    {"settings", "cboxMax", "cbox", "max", true},
    {"settings", "illumPosition", "illumPosition", "position", true},
    {"settings", "illumBone", "illumPosition", "bone"},
    {"settings", "eyePosition", "eyePosition", "position", true},
    {"settings", "eyeAutoHeight", "eyePosition", "autoHeight"},
    {"lod", "removeSmallMeshes", "removeSmallMeshes", "size", true},
    {"lod", "removeSmallMeshesIn", "removeSmallMeshes", "meshes"},
    {"defineBone", "realignPosition", "realign", "position", true},
    {"defineBone", "realignAngles", "realign", "angles", true},
    {"transformBone", "residualBone", "transformWeights", "residualBone", true},
    {"transformBone", "weightsFactor", "transformWeights", "factor"},
    {"transformBone", "weightsSmoothing", "transformWeights", "smoothing"},
    {"jiggleCommon", "yawMin", "yawConstraint", "min", true},
    {"jiggleCommon", "yawMax", "yawConstraint", "max", true},
    {"jiggleCommon", "pitchMin", "pitchConstraint", "min", true},
    {"jiggleCommon", "pitchMax", "pitchConstraint", "max", true},
    {"jiggleBaseSpring", "leftMin", "leftConstraint", "min", true},
    {"jiggleBaseSpring", "leftMax", "leftConstraint", "max", true},
    {"jiggleBaseSpring", "upMin", "upConstraint", "min", true},
    {"jiggleBaseSpring", "upMax", "upConstraint", "max", true},
    {"jiggleBaseSpring", "forwardMin", "forwardConstraint", "min", true},
    {"jiggleBaseSpring", "forwardMax", "forwardConstraint", "max", true},
    {"ikRule", "rangeStart", "range", "start", true},
    {"ikRule", "rangePeak", "range", "peak", true},
    {"ikRule", "rangeTail", "range", "tail", true},
    {"ikRule", "rangeEnd", "range", "end", true},
    {"animationOperation", "rangeStart", "range", "start", true},
    {"animationOperation", "rangePeak", "range", "peak", true},
    {"animationOperation", "rangeTail", "range", "tail", true},
    {"animationOperation", "rangeEnd", "range", "end", true},
    {"clip", "framesFrom", "frames", "start", true},
    {"clip", "framesTo", "frames", "end", true},
    {"sequence", "activity", "activity", "name", true},
    {"sequence", "activityWeight", "activity", "weight"},
    {"sequence", "transitionFrom", "transition", "from", true},
    {"sequence", "transitionTo", "transition", "to", true},
    {"sequence", "transitionReversible", "transition", "reversible"},
    {"eyelid", "lowererTarget", "lowerer", "target", true},
    {"eyelid", "lowererDelta", "lowerer", "delta"},
    {"eyelid", "neutralTarget", "neutral", "target", true},
    {"eyelid", "neutralDelta", "neutral", "delta"},
    {"eyelid", "raiserTarget", "raiser", "target", true},
    {"eyelid", "raiserDelta", "raiser", "delta"},
    {"physicsShape", "primitive", "primitive", "type", true},
    {"physicsShape", "bone", "primitive", "bone", true},
    {"physicsShape", "min", "primitive", "min"},
    {"physicsShape", "max", "primitive", "max"},
    {"physicsShape", "angles", "primitive", "angles"},
    {"physicsShape", "center", "primitive", "center"},
    {"physicsShape", "radius", "primitive", "radius"},
    {"physicsShape", "point0", "primitive", "point0"},
    {"physicsShape", "point1", "primitive", "point1"},
    {"physicsShape", "radius0", "primitive", "radius0"},
    {"physicsShape", "radius1", "primitive", "radius1"},
    {"physicsShape", "segments", "primitive", "segments"},
    {"physics", "frictionMin", "animatedFriction", "min", true},
    {"physics", "frictionMax", "animatedFriction", "max", true},
    {"physics", "frictionTimeIn", "animatedFriction", "timeIn", true},
    {"physics", "frictionTimeOut", "animatedFriction", "timeOut", true},
    {"physics", "frictionTimeHold", "animatedFriction", "timeHold", true},
    {"animationSettings", "sectionFrames", "sectionFrames", "frames", true},
    {"animationSettings", "sectionMinLimit", "sectionFrames", "minLimit", true},
};

// A switch that adds `word` to the record's `list`; without a word the field's own value goes in.
struct Word {
    const char* item;
    const char* key;
    const char* list;
    const char* word;
};

const Word kWords[] = {
    {"contentsFlags", "solid", "contents", "solid"},
    {"contentsFlags", "grate", "contents", "grate"},
    {"contentsFlags", "ladder", "contents", "ladder"},
    {"contentsFlags", "monster", "contents", "monster"},
    {"contentsFlags", "debris", "contents", "debris"},
    {"contentsFlags", "notSolid", "contents", "notsolid"},
    {"contentsFlags", "contentsMask", "contents", nullptr},
    {"scaleBone", "x", "axes", "x"},
    {"scaleBone", "y", "axes", "y"},
    {"scaleBone", "z", "axes", "z"},
    {"animationOperation", "axisX", "axes", "x"},
    {"animationOperation", "axisY", "axes", "y"},
    {"animationOperation", "axisZ", "axes", "z"},
    {"clip", "ignoreBoneAngles", "ignoreTransformBone", "angles"},
    {"clip", "ignoreBonePosition", "ignoreTransformBone", "position"},
};

// The words of the motion controls, the same for a clip's motion and an operation's controls.
const struct { const char* key; const char* word; } kMotionWords[] = {
    {"x", "X"},   {"y", "Y"},   {"z", "Z"},     {"xr", "XR"},   {"yr", "YR"},   {"zr", "ZR"}, {"lx", "LX"},
    {"ly", "LY"}, {"lz", "LZ"}, {"lxr", "LXR"}, {"lyr", "LYR"}, {"lzr", "LZR"}, {"lm", "LM"}, {"lq", "LQ"},
};

// A child node that is one entry of its parent's `list`: its `field` value, when `flag` is
// unset or true.
struct Gather {
    const char* parent;
    const char* child;
    const char* list;
    const char* field;
    const char* flag = nullptr;
};

const Gather kGathers[] = {
    {"settings", "searchFolder", "searchDirs", "folder"},
    {"settings", "includeModel", "includeModels", "file"},
    {"settings", "boundsMesh", "bboxMeshes", "mesh"},
    {"materials", "materialFolder", "paths", "folder"},
    {"materials", "drawOrder", "sortOrder", "material"},
    {"mesh", "dropMaterial", "removeMeshes", "material"},
    {"mesh", "dropMaterialWord", "removeMeshWords", "word"},
    {"choice", "choiceMesh", "meshes", "mesh"},
    {"lod", "lodRemoveMesh", "removeModels", "mesh"},
    {"lod", "lodCollapseTree", "collapseBoneTrees", "bone"},
    {"lod", "dropMaterial", "removeMeshes", "material"},
    {"lod", "dropMaterialWord", "removeMeshWords", "word"},
    {"lod", "lodSmallMesh", "removeSmallMeshesIn", "mesh"},
    {"bones", "boneMarkup", "boneMerge", "bone", "boneMerge"},
    {"bones", "boneMarkup", "doNotCollapse", "bone", "doNotCollapse"},
    {"bones", "boneMarkup", "alwaysCollapse", "bone", "alwaysCollapse"},
    {"bones", "vrdFile", "vrdFiles", "file"},
    {"attachment", "placeOnFlexGroup", "flexGroups", "name"},
    {"attachment", "placeOnMorph", "flexMorphs", "name"},
    {"attachment", "placeOnMaterial", "materials", "material"},
    {"sequence", "blendFile", "files", "file"},
    {"sequence", "blendAnimation", "animations", "animation"},
    {"sequence", "activityModifier", "activityModifiers", "name"},
    {"flex", "flexLocalVar", "localVars", "name"},
    {"flexCorrective", "correctiveControl", "controls", "control"},
    {"flexDomination", "dominator", "dominators", "control"},
    {"physicsShape", "extraBone", "extraBones", "bone"},
    {"physicsShape", "excludeMesh", "excludeMeshes", "mesh"},
    {"skipTransition", "transitionNode", "nodes", "name"},
    {"animationSettings", "allowedActivity", "allowedActivities", "name"},
};

// Fields named apart from the record key they fill, as "type" names the node itself.
const struct { const char* item; const char* key; const char* record; } kRenamed[] = {
    {"ikRule", "rule", "type"},
    {"eyelid", "lid", "type"},
    {"jointAxis", "motion", "type"},
};

// The model-level record each section's own fields form; null = the section only holds nodes.
const struct { const char* section; const char* key; } kSectionRecords[] = {
    {"settings", "model"},         {"materials", "materials"}, {"bones", "bones"},
    {"physics", "physics"},        {"face", "face"},           {"flex", "flex"},
    {"animationSettings", "animationSettings"},
};

bool IsA(const Item& item, const char* name) {
    for (const Item* i = &item; i; i = i->base)
        if (std::strcmp(i->name, name) == 0) return true;
    return false;
}

const Move& MoveOf(const Item& parent, const Item& child) {
    for (const Move& move : kMoves)
        if (IsA(parent, move.parent) && std::strcmp(child.name, move.child) == 0) return move;
    std::abort(); // every child type the schema allows has a move
}

bool HasContent(const Json& obj) {
    for (const auto& [key, value] : obj.items())
        if (!IsInternal(key)) return true;
    return false;
}

bool Reshape(Loader& l, const Json& node, const Item& item, Json& model, Json& out) {
    out = Json::object();
    for (const auto& [key, value] : node.items())
        if (key != "type" && key != "enabled" && key != "children" && key != "editor") out[key] = value;
    for (const auto& renamed : kRenamed) {
        if (!IsA(item, renamed.item) || !out.contains(renamed.key)) continue;
        out[renamed.record] = out.at(renamed.key);
        out[kKeyOrigins][renamed.record] = KeyOrigin(node, renamed.key);
        out.erase(renamed.key);
    }

    // switches become the word lists they stand for
    const auto addWord = [&](const char* key, const char* list, const char* word) {
        const auto found = out.find(key);
        if (found == out.end()) return;
        const Json value = *found;
        out.erase(key);
        if (word && !value.get<bool>()) return;
        if (!out.contains(list)) out[kKeyOrigins][list] = KeyOrigin(node, key);
        out[list].push_back(word ? Json(word) : Json(ValueText(value)));
    };
    for (const Word& word : kWords)
        if (IsA(item, word.item)) addWord(word.key, word.list, word.word);
    if (IsA(item, "motionControls"))
        for (const auto& control : kMotionWords)
            addWord(control.key, IsA(item, "clip") ? "motion" : "controls", control.word);
    if (IsA(item, "jointContents") && !out.contains("contents"))
        return l.Fail(node, "", "a Joint Contents node needs at least one contents flag");

    // one-value children become their parent's lists; the rest keep their own records
    if (const Json* children = Get(node, "children")) {
        for (const Json& child : *children) {
            const Item& kind = ItemOf(item, child);
            if (IsA(item, "bones") && std::strcmp(kind.name, "boneMarkup") == 0 &&
                child.value("noRealign", false))
                out["realignDirectives"].push_back(child);

            bool gathered = false;
            for (const Gather& gather : kGathers) {
                if (!IsA(item, gather.parent) || std::strcmp(kind.name, gather.child) != 0) continue;
                gathered = true;
                if (gather.flag && !child.value(gather.flag, false)) continue;
                if (!out.contains(gather.list)) out[kKeyOrigins][gather.list] = KeyOrigin(child, gather.field);
                out[gather.list].push_back(child.at(gather.field));
            }
            if (gathered) continue;

            const Move& move = MoveOf(item, kind);
            Json shaped;
            if (!Reshape(l, child, kind, model, shaped)) return false;
            if (move.into == Into::Object) out[move.key] = std::move(shaped);
            else if (move.into == Into::Model) model[move.key].push_back(std::move(shaped));
            else out[move.key].push_back(std::move(shaped));
        }
    }

    // every grouped field is taken out first, as a field may share its object's name
    std::vector<std::pair<const Flat*, Json>> grouped;
    for (const Flat& flat : kFlats) {
        if (!IsA(item, flat.item) || !out.contains(flat.key)) continue;
        grouped.emplace_back(&flat, out.at(flat.key));
        out.erase(flat.key);
        if (out.contains(kKeyOrigins)) out[kKeyOrigins].erase(flat.key);
    }

    std::set<std::string> off;
    for (const auto& [flat, value] : grouped) {
        if (!flat->sub && !value.get<bool>()) {
            off.insert(flat->object);
            continue;
        }
        Json& object = out[flat->object];
        if (!object.contains(kOrigin)) object[kOrigin] = node.at(kOrigin);
        if (!flat->sub) continue;
        object[flat->sub] = value;
        object[kKeyOrigins][flat->sub] = KeyOrigin(node, flat->key);
    }
    for (const auto& [flat, value] : grouped)
        if (flat->sub && off.count(flat->object))
            return l.Fail(node, flat->key, "\"" + std::string(flat->key) + "\" is set while its switch is off");
    for (const Flat& flat : kFlats)
        if (flat.required && IsA(item, flat.item) && out.contains(flat.object) &&
            !out.at(flat.object).contains(flat.sub))
            return l.Fail(node, "", "missing \"" + std::string(flat.key) + "\"");
    return true;
}

// The merged model in the layout the lowering reads.
bool ReshapeModel(Loader& l, const Json& merged, Json& model) {
    model = Json::object();
    model[kOrigin] = merged.at(kOrigin);
    for (const auto& [key, section] : merged.items()) {
        if (IsInternal(key) || key == "editor") continue;
        const Item& item = *FindField(kContentItem, key)->item;
        Json shaped;
        if (!Reshape(l, section, item, model, shaped)) return false;

        if (key == "settings") {
            for (const char* moved : {"searchDirs", "includeModels"}) {
                if (!shaped.contains(moved)) continue;
                model[kKeyOrigins][moved] = KeyOrigin(section, moved);
                model[moved] = shaped.at(moved);
                shaped.erase(moved);
            }
        }
        for (const auto& record : kSectionRecords)
            if (key == record.section && HasContent(shaped)) model[record.key] = std::move(shaped);
    }
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
    Json merged;  // the node tree, with the internal origin keys
    Json lowered; // the same in the layout the lowering reads
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
        if (field.group) entry["group"] = field.group;
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
    if (!shared.is_null() && !DefineVariables(l, shared, pinned, scope)) return false;
    if (!DefineVariables(l, model, pinned, scope)) return false;

    // contributor order is list order: shared, its prefabs, the model, its prefabs
    std::vector<Json> parts;
    if (!shared.is_null() && !CollectContent(l, shared, scope, parts)) return false;
    const size_t modelPart = parts.size();
    if (!CollectContent(l, model, scope, parts)) return false;

    Json merged = Json::object();
    merged[kOrigin] = parts[modelPart].at(kOrigin);
    for (const Json& part : parts)
        if (!Merge(l, kContentItem, merged, part)) return false;

    const Json* settings = Get(merged, "settings");
    if (out.outname.empty() && (!settings || !Get(*settings, "name")))
        return l.Fail(parts[modelPart], "", "the model has no settings.name");

    Json lowered;
    if (!ReshapeModel(l, merged, lowered)) return false;
    g_loaded = {modelId, merged, lowered, {}};
    for (const SourceFile& file : l.files) g_loaded.files.push_back(file.path);

    Emitter e{l};
    if (!Lower(e, lowered)) return false;

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
        if (*item->title) entry["title"] = item->title;
        if (*item->category) entry["category"] = item->category;
        if (item->children) {
            entry["children"] = Json::array();
            for (const Item* const* kind = item->children; *kind; ++kind)
                entry["children"].push_back((*kind)->name);
        }
        if (item->minChildren) entry["minChildren"] = item->minChildren;
        if (item->single) entry["single"] = true;
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
    const Json& m = g_loaded.lowered;
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
