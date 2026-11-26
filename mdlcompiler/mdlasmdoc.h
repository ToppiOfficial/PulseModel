// mdlasmdoc.h - the decoded contents of an .mdlasm model assembly.
//
// Mirrors what the Model Assembler saves (Studio libs/modeleditor/document.h). Only compiler-facing
// fields are kept; editor-only values are read and dropped. Values are in the space the .qc commands
// read them in, except where a field says otherwise.

#ifndef PULSEMODEL_MDLASMDOC_H
#define PULSEMODEL_MDLASMDOC_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "compile.h"
#include "math/math.h"
#include "meshedit.h"
#include "pulselimits.h"
#include "skeletonedit.h"
#include "source.h"

namespace pulse::mdlasm {

namespace limits = pulse::limits;

inline constexpr int kFormatVersion = 4;

using Uid = std::uint64_t;

// One edit block of a $rendermesh: a $meshedit or, with `skeleton`, a $skeletonedit.
struct MeshEditBlock
{
    std::uint64_t uid = 0;
    bool enabled = true;
    bool ignoreSelectWarn = false;
    bool skeleton = false;
    std::vector<source::EditMeshOperation> operations;
    std::vector<source::EditSkeletonOperation> skeletonOperations;
    std::string editorName;
};

// The options of one $rendermesh body, in script order.
struct RenderMeshEdit
{
    std::string name;
    source::MeshFilter filter;
    std::vector<MeshEditBlock> editBlocks;
    bool noMorph = false;
    bool skeletalAwareDecimation = false;
    std::vector<std::pair<float, int>> decimates; // (factor, face tag or -1 = all)
};

enum class SourceKind : std::uint32_t { Mesh, Animation, Physics, FlexRig, Joints };

// A source file embedded in the document. `path` and `modified` drive the automatic refresh from the original file.
struct SourceFile
{
    Uid uid = 0;
    SourceKind kind = SourceKind::Mesh;
    std::string path; // UTF-8 path of the original
    std::int64_t modified = 0; // file time ticks of the original
    std::shared_ptr<const std::string> bytes; // immutable, shared between document snapshots
};

// Auto follows the DMX up axis (rotationSet false in the compiler). Angles is `$transformmodel angles`
// and the rest are `$upaxis`; z and -z are the same rotation, so only one exists (qcloader.cpp:5007-5092).
enum class RotationMode : std::uint32_t { Auto, Angles, UpX, UpNegX, UpY, UpNegY, UpZ };

enum class ModelRenderPass : std::uint32_t { None, Opaque, MostlyOpaque };

struct RenderDefinition
{
    bool present = false;
    bool active = true;
    std::string editorName;
    ModelRenderPass pass = ModelRenderPass::None;
    bool ambientBoost = false;
    bool doNotCastShadows = false;
    bool castTextureShadows = false;
    bool constantDirectionalLight = false;
    float directionalLightScale = 0.0f;
    bool noForcedFade = false;
    bool forcePhonemeCrossfade = false;
};

// The children of Model Definition in outliner order. Eye and Render show only while present.
enum class DefinitionNode : std::uint8_t { Eye, Render, BoundingBox, ViewBox, LightingOrigin };
inline constexpr std::size_t DefinitionNodeCount = 5;

// ModelDefinition: model-wide values. `$bbox`, `$cbox` and `$illumposition` are opt-in (off lets the
// compiler derive them).
struct ModelDefinition
{
    std::string name;        // $modelname
    std::string surfaceProp; // $surfaceprop

    float scale = 1.0f;             // $scale
    math::Vector3 origin;           // $origin, source axes
    RotationMode rotationMode = RotationMode::Auto;
    math::Vector3 angles;           // degrees (pitch, yaw, roll), used by Angles
    compile::Archetype archetype = compile::Archetype::General;

    // Each node exists only after the user adds it; the `Set` flag is its switch.
    bool bboxPresent = false, cboxPresent = false, illumPresent = false;
    bool bboxSet = false;
    math::Vector3 bbox[2];
    bool cboxSet = false;
    math::Vector3 cbox[2];
    bool illumSet = false;
    math::Vector3 illum;
    std::string illumBone;          // empty = static position

    bool eyeEnabled = false;        // EyeDefinition node present; off writes no eye data
    bool eyeActive = true;          // the node is switched on
    math::Vector3 eyePosition;      // scaled by $scale
    bool eyeAutoHeight = false;
    float eyeMaxDeflection = 30;    // $maxeyedeflection in degrees; 30 is the engine's own default
    std::string eyeEditorName;
    RenderDefinition render;
    std::array<DefinitionNode, DefinitionNodeCount> nodeOrder{DefinitionNode::Eye, DefinitionNode::Render,
        DefinitionNode::BoundingBox, DefinitionNode::ViewBox, DefinitionNode::LightingOrigin};
    int weightFitPoses = limits::kWeightFitPoses;
    compile::BoneCullType boneCullMethod = compile::BoneCullType::Aggressive;
};

// One `$rendermesh`: the embedded source it reads and its edit options.
struct MeshEntry
{
    Uid uid = 0;
    Uid source = 0;
    bool enabled = true; // a disabled mesh is kept but not built
    RenderMeshEdit edit;
    Uid folder = 0;
    bool userEnabled = true;
};

// One choice of a body group: the render meshes it draws (fused when several), or blank when none. An empty
// name lets the compiler name the choice after its meshes.
struct ModelGroupChoice
{
    std::vector<Uid> meshes;
    std::string name;
};

// A `$modelgroup`: one body part with its choices.
struct ModelGroup
{
    Uid uid = 0;
    std::string name;
    bool enabled = true; // a disabled group is kept but not built
    std::vector<ModelGroupChoice> choices;
    Uid folder = 0;
    bool userEnabled = true;
};

struct MaterialRule
{
    std::string from, to;

    bool complete() const { return !from.empty() && !to.empty(); }
};

struct MaterialMarkup
{
    std::vector<std::string> directories;
    std::vector<MaterialRule> renames, collapses;
};

struct BoneDefinition
{
    struct Bind
    {
        std::string parent;
        math::Vector3 origin, angles;
        math::Vector3 realignOrigin, realignAngles;
        bool active = false, unlocked = false, preAligned = false;
    } bind;

    Uid uid = 0;
    std::string bone, legacyName;
    bool enabled = true;
    bool boneMerge = true;
    bool doNotCollapse = false, alwaysCollapse = false, noRealign = false;
    Uid folder = 0;
    bool userEnabled = true;

    bool targetsBone() const { return enabled && !bone.empty(); }
    bool marksBone() const { return targetsBone() && boneMerge; }
    bool keepsBone() const { return targetsBone() && (boneMerge || doNotCollapse); }
    bool collapsesBone() const { return targetsBone() && alwaysCollapse; }
    bool editsBindPose() const { return targetsBone() && bind.active; }
    bool hasOrderedEdits() const { return editsBindPose() || (targetsBone() && noRealign); }
    bool affectsSkeleton() const { return keepsBone() || collapsesBone() || hasOrderedEdits(); }
};

// One `$realignbone` edit. The nodes apply in list order after every `$definebone`; a Bone Definition with No
// realign then drops every one on its bone, as `$norealign` does.
struct RealignNode
{
    Uid uid = 0;
    std::string name, bone;
    bool enabled = true;
    compile::BoneRealignEdit edit;
    Uid folder = 0;
    bool userEnabled = true;

    bool targetsBone() const { return enabled && !bone.empty(); }
};

// One `$hierarchy`: `bone` moves under `parent` (a root when empty). The nodes apply in list order.
struct HierarchyNode
{
    Uid uid = 0;
    std::string bone, parent;
    bool enabled = true;
    Uid folder = 0;
    bool userEnabled = true;

    bool targetsBone() const { return enabled && !bone.empty(); }
};

struct Attachment
{
    Uid uid = 0;
    std::string name, bone;
    math::Vector3 origin, angles;
    bool enabled = true;
    bool absolute = false, rigid = false, worldAlign = false, anglesSet = false;
    Uid folder = 0;
    bool userEnabled = true;
};

// One `eyeball`. The origin is in source axes, scaled by `$scale`; with `center` it is an offset in final axes
// from the bounding box center of the vertices wearing the material.
// One pose of an eyelid: the morph that shapes the lid there and the height on the eye, in source units, where it
// applies.
struct EyelidPose
{
    std::string morph;
    float height = 0;
};

struct Eyeball
{
    Uid uid = 0;
    std::string name, bone, material;
    math::Vector3 origin;
    bool center = false;
    float diameter = 1, angle = 0, pupilScale = 1;
    bool enabled = true;
};

enum class EyelidType : std::uint32_t { Upper, Lower };

// One lid of the face, as `$eyelid` authors it. The lid follows the aimed eye by its height: the lowerer, neutral and
// raiser poses (in that order) blend as that height crosses their own. It binds to eyeballs by name: with both set it
// is a stereo lid whose right and left sides each get their own descs; with one set, a lid for that eye alone.
// `split` keeps only the vertices of each morph on one side of the midline, so one morph can serve both eyes; 0 uses
// the whole morph.
struct EyelidNode
{
    Uid uid = 0;
    std::string name;
    bool enabled = true;
    EyelidType type = EyelidType::Upper;
    std::string rightEye, leftEye;
    std::array<EyelidPose, 3> poses;
    float split = 0;

    bool authored() const
    {
        return std::ranges::any_of(poses, [](const EyelidPose& pose) { return !pose.morph.empty(); });
    }
};

struct Hitbox
{
    Uid uid = 0;
    std::string name, bone;
    math::Vector3 min{-4, -4, -4}, max{4, 4, 4}, angles;
    float radius = 2;
    int group = 0;
    bool enabled = true, capsule = false;
};

struct HitboxSet
{
    Uid uid = 0;
    std::string name;
    bool enabled = true;
    std::vector<Hitbox> boxes;
    Uid folder = 0;
    bool userEnabled = true;
};

enum class PhysicsShapeType : std::uint32_t { Box, Sphere, Capsule, FromFile, FromRenderMesh, FromRender };
enum class PhysicsAxisMode : std::uint32_t { Unset, Free, Limit, Fixed };
enum class PhysicsBodyMode : std::uint32_t { Default, Skip, Merge };
enum class PhysicsMassMode : std::uint32_t { Default, Fixed, Bias };

struct PhysicsShape
{
    PhysicsShapeType type = PhysicsShapeType::Box;
    std::string bone;
    math::Vector3 min{-4, -4, -4}, max{4, 4, 4}, angles, center;
    float radius = 2, endRadius = 2;
    int segments = 6;
    bool merge = true;

    Uid source = 0;
    std::string renderMesh;
    math::Vector3 offsetOrigin, offsetAngles;
    float importScale = 1;
    compile::PhysicsImportType importType = compile::PhysicsImportType::Skinned;
    int maxConvex = limits::kMaxConvexPieces;
    bool concave = false, remove2d = false;
    float decimationFactor = 0.22f, concavity = 0.04f, cullWeight = 0.42f;
    int maxHulls = 0, maxDepth = 4;
    std::vector<std::string> extraBones, excludedMeshes;
};

struct PhysicsAxis
{
    PhysicsAxisMode mode = PhysicsAxisMode::Unset;
    float min = 0, max = 0, friction = 1;
};

struct PhysicsJoint
{
    std::string bone;
    std::array<PhysicsAxis, 3> axes;
};

struct PhysicsBody
{
    std::string bone, mergeInto;
    PhysicsBodyMode mode = PhysicsBodyMode::Default;
    PhysicsMassMode massMode = PhysicsMassMode::Default;
    float mass = 1, massBias = 1, inertia = 1, damping = 0, rotationDamping = 0;
    bool inertiaSet = false, dampingSet = false, rotationDampingSet = false;
};

struct PhysicsPair
{
    std::string first, second;
    bool collide = true;
};

struct PhysicsNode
{
    Uid uid = 0;
    std::string name;
    bool enabled = true;
    std::variant<PhysicsShape, PhysicsJoint, PhysicsBody, PhysicsPair> value;
    Uid folder = 0;
    bool userEnabled = true;

};

struct PhysicsSettings
{
    std::string rootBone, fileName;
    math::Vector3 massCenter;
    float mass = 1, damping = 0, rotationDamping = 0, inertia = 1, drag = -1;
    float weldPosition = 0, weldNormal = 0.999f;
    bool autoMass = false, massCenterSet = false, noSelfCollisions = false, assumeWorldSpace = false;
    bool animatedFriction = false;
    int frictionMin = 0, frictionMax = 0;
    float frictionIn = 0, frictionOut = 0, frictionHold = 0;
};

enum class BoneMarkupKind : std::uint32_t { Jiggle, Procedural, Ik };

// `$jigglebone`: Flexibility picks is_flexible or is_rigid, Base picks has_base_spring or is_boing; the two
// choices are independent and each can be None. Defaults are the compiler's (JiggleBone in compile.h).
enum class JiggleStyle : std::uint32_t { Flexible, Rigid, None };
enum class JiggleBase : std::uint32_t { None, BaseSpring, Boing };

struct JiggleSettings
{
    JiggleStyle style = JiggleStyle::Flexible;
    JiggleBase base = JiggleBase::None;
    // `allow_length_flex` of is_flexible; the along spring only applies while it is on.
    bool lengthFlex = false;

    float length = 10, tipMass = 0;
    float yawStiffness = 100, yawDamping = 0, pitchStiffness = 100, pitchDamping = 0;
    float alongStiffness = 100, alongDamping = 0;
    float yawFriction = 0, yawBounce = 0, pitchFriction = 0, pitchBounce = 0;
    bool angleConstraint = false, yawConstraint = false, pitchConstraint = false;
    float angleLimit = 0, minYaw = 0, maxYaw = 0, minPitch = 0, maxPitch = 0;

    // A direction without its constraint keeps the compiler's -100 to 100 range and no friction.
    float baseMass = 0, baseStiffness = 100, baseDamping = 0;
    bool baseLeftConstraint = false, baseUpConstraint = false, baseForwardConstraint = false;
    float baseMinLeft = -100, baseMaxLeft = 100, baseLeftFriction = 0;
    float baseMinUp = -100, baseMaxUp = 100, baseUpFriction = 0;
    float baseMinForward = -100, baseMaxForward = 100, baseForwardFriction = 0;

    // Degrees. 45 stands for the compiler's literal 0.7071 cosine; any other angle is stored as its cosine.
    float boingImpactSpeed = 100, boingImpactAngle = 45, boingDampingRate = 0.25f;
    float boingFrequency = 30, boingAmplitude = 0.35f;
};


// Twist compiles three triggers: the pose, then the driver turned by +twistAngle and -twistAngle degrees about its
// own twistAxis, each with the helper pose turned the same way about the helper's own twistAxis, or the opposite
// way with invertHelperTwist.
enum class TriggerKind : std::uint32_t { Pose, Twist };
enum class TwistAxis : std::uint32_t { X, Y, Z };

// One `trigger` line of `$driverbone`: rotations are RadianEuler degrees (x roll, y pitch, z yaw), the
// helper position is in model units.
struct ProceduralTrigger
{
    bool enabled = true;
    float tolerance = 90;
    math::Vector3 driverAngles, helperAngles, helperPosition;
    TriggerKind kind = TriggerKind::Pose;
    TwistAxis twistAxis = TwistAxis::X;
    float twistAngle = 100;
    bool invertHelperTwist = false;
};

// Driver: `$driverbone`, the node's bone is the helper. AimAt: `$driveraimat`.
enum class ProceduralMode : std::uint32_t { Driver, AimAt };

// A further helper of a Driver entry. It compiles to its own `$driverbone` with the entry's driver, triggers and
// influence in, and its own influence out. An empty bone is skipped.
struct ProceduralHelper
{
    std::string bone;
    float influenceOutput = 1.0f;
};

struct ProceduralSettings
{
    ProceduralMode mode = ProceduralMode::Driver;
    std::string driver;
    bool local = false;
    math::Vector3 basePosition;
    // Scale the driver's rotation and the helper's pose from their bind poses. Input above 0, output 0 or more.
    float influenceInput = 1.0f, influenceOutput = 1.0f;
    std::vector<ProceduralTrigger> triggers;
    std::vector<ProceduralHelper> helpers;

    std::string target;
    math::Vector3 up{0, 0, 1}, aim{1, 0, 0};
};

// `$ikchain` with the optional `$ikautoplaylock` of the same chain. The node's bone is the chain's end bone;
// its two parents complete the three links.
struct IkSettings
{
    std::string chain;
    math::Vector3 knee, center;
    float height = 18, pad = 0, floor = 0;
    bool autoplayLock = false;
    float lockPosition = 1, lockRotation = 0.1f;
};

// One entry of the Bone Markup List: a bone is exactly one of a jigglebone, a procedural bone or an IK chain end,
// and no bone has two entries.
struct BoneMarkupNode
{
    Uid uid = 0;
    std::string bone;
    bool enabled = true;
    std::variant<JiggleSettings, ProceduralSettings, IkSettings> value;
    Uid folder = 0;
    bool userEnabled = true;

    BoneMarkupKind kind() const { return BoneMarkupKind(value.index()); }
    bool targetsBone() const { return enabled && !bone.empty(); }
};

// A Prefabs entry: a DMX whose rig is read in place of authoring it. Joints is `$datamodeljoints` (the four toggles
// pick what it imports; all off imports nothing), Physics is `$datamodelphysics`. Both follow their file.
enum class PrefabKind : std::uint32_t { Joints, Physics };

// $renamehboxset on a Joints entry: a hitbox set of its file imported under another name.
struct HitboxSetRename
{
    Uid uid = 0;
    bool enabled = true;
    std::string from, to;
    bool complete() const { return !from.empty() && !to.empty(); }
};

struct PrefabNode
{
    Uid uid = 0;
    std::string name;
    bool enabled = true;
    PrefabKind kind = PrefabKind::Joints;
    Uid source = 0;
    bool jigglebones = true, proceduralbones = true, hitboxes = true, attachments = true;
    std::vector<HitboxSetRename> hitboxSetRenames;

    bool imports() const { return enabled && source; }
};


// One model line of a `$lod` block. `target` names a render mesh or a bodygroup choice that fuses several meshes.
// A rule replaces the geometry (`replacemodel`, with a render mesh), removes it (`removemodel`) or simplifies it
// (`decimatemodel`, factor = share of triangles kept).
enum class LodAction : std::uint32_t { Decimate, Replace, Remove };

struct LodModelRule
{
    std::string target;
    LodAction action = LodAction::Decimate;
    std::string replacement;
    float factor = 0.5f;
};

struct LodBoneRule
{
    std::string bone, replacement;

    bool complete() const { return !bone.empty() && !replacement.empty(); }
};

// One `$lod` or `$shadowlod` block; the implicit root LOD has no node. Nodes keep script order.
struct LodNode
{
    Uid uid = 0;
    std::string name;
    bool enabled = true;
    bool shadow = false;
    float switchValue = 0.0f;

    std::vector<LodModelRule> models;
    bool decimateAll = false;
    float decimateAllFactor = 0.5f;
    bool skeletalAware = false;
    bool removeSmall = false;
    float smallSize = 1.0f;
    std::vector<std::string> smallMeshes; // empty = every model

    std::vector<LodBoneRule> boneReplacements;
    std::vector<std::string> boneTreeCollapses;
    bool collapseJiggle = false, collapseProcedural = false;

    std::vector<MaterialRule> materialReplacements, materialSubstringReplacements;
    std::vector<std::string> meshRemovals, meshSubstringRemovals;

    bool facial = true;
    bool shadowMaterials = false; // use_shadowlod_materials, shadow LOD only
};

// `$minlod` and `$allowrootlods`.
struct LodSettings
{
    int minLod = 0;
    int allowRootLods = 0;
};

// The Flex Graph: nodes and links of one graph for the whole model. Controller, Constant and the inputs' own
// values are numbers; names and the morph choice sit in `name`. flexgraph.h says which fields a type reads.
enum class FlexNodeType : std::uint32_t
{
    Controller, Constant, MorphValue, LocalVarGet,
    Add, Subtract, Multiply, Divide, Negate, Min, Max,
    Clamp, Invert, MapRange, Mix, Abs,
    FlexOutput, LocalVar,
    Reroute, Frame,
    // Added after the first release of the graph: later values keep older documents readable. Math is Add, Subtract,
    // Multiply, Divide, Minimum or Maximum by `mode`; the six single-operation types above remain loadable.
    Math, Crossfade,
    // Retired node of the first Flex Graph files; loading turns each into a FlexSplit and drops the node.
    MorphSplit,
    // Rig nodes: what the game's flex ops do (combo, dominate, 2-way, n-way, eyelid), written as the rule math they equal.
    Combination, Dominate, TwoWay, NWay, Eyelid
};

inline constexpr std::size_t FlexNodeValues = 5;

struct FlexNode
{
    Uid uid = 0;
    FlexNodeType type = FlexNodeType::Constant;
    float x = 0, y = 0;
    bool muted = false, collapsed = false;
    // The enclosing Frame node, 0 when loose.
    Uid parent = 0;
    // Controller, Local Var and Local Var Get: the variable. Flex Output and Morph Value: the morph. Frame: its label.
    std::string name;
    // Controller only: its `$flexcontroller` group.
    std::string group;
    float min = 0.0f, max = 1.0f;
    // Controller only: a left and a right controller, so the morphs it feeds become L/R pairs.
    bool stereo = false;
    // Math: which operation.
    int mode = 0;
    // What each input socket reads while it has no link; Constant keeps its number in [0].
    std::array<float, FlexNodeValues> values{};
    // Frame only: its size while it holds no node.
    float width = 240.0f, height = 160.0f;
};

struct FlexLink
{
    Uid from = 0, to = 0;
    int fromSocket = 0, toSocket = 0;
};

// `$wrinklescale <morph> <scale>`: how much of a morph's movement also drives the wrinkle map; 0 writes none.
struct FlexWrinkle
{
    std::string morph;
    float scale = 0.0f;
};

// `$morphsplitstereo <morph> <factor>`: the morph is listed as `<name>L` and `<name>R`. The mesh's own balance is
// kept when it has any, else one is generated across the midline with the factor (0 generates none).
struct FlexSplit
{
    std::string morph;
    float factor = 0.0f;
};


enum class FolderKind : std::uint32_t { Mesh, ModelGroup, BoneDefinition, Attachment, HitboxSet, Physics, BoneMarkup };

struct Folder
{
    Uid uid = 0;
    std::string name;
    FolderKind kind = FolderKind::Mesh;
    bool enabled = true;
    Uid parent = 0;
};


using AnimCommand = compile::CompileInput::InAnim::InCmd;
using AnimIkRule = compile::CompileInput::InIkRule;

enum class AnimNodeType : std::uint32_t
{
    Out, Clip, Animation, Sequence, WeightList, Command, Blend, Walk, Layer, PoseParam, ActivityModifier, IkLock, IkRule,
    IncludeModel, DeclareSequence
};

// What a socket carries. Animation is a named `$animation`, usable wherever a clip is and as a reference.
// Modifier, IkLock and IkRule are entries a Sequence or Animation lists; one entry node can feed several.
// Model and Declare feed only Animation Out: an `$includemodel` and a `$declaresequence` slot.
enum class AnimSocket : std::uint8_t { Clip, ClipSet, Animation, Weights, Sequence, Pose, Modifier, IkLock, IkRule, Model, Declare };

// A DMX clip: which embedded source, which clip inside it, and the frames taken from it.
struct AnimClipData
{
    Uid source = 0;
    std::string clip;      // `blockname`; empty takes the first clip that is not the bind pose
    bool bindPose = false; // no source: the model's bind pose
    int startFrame = 0;
    int endFrame = -1;     // -1 runs to the end of the clip
};

// The options of one `$animation`, shared by the Animation node and the Sequence node (its blend anim 0).
struct AnimProps
{
    float fps = 30.0f;
    bool loop = false, snap = false, post = false, noForceLoop = false;
    bool fudgeLoop = false, noAutoIk = false, noCull = false, ignoreScale = false;
    int startLoop = 0;
    float startLoopPercent = 0.0f;
    int motionType = 0;
    float motionRollback = 0.3f;
    int numFrames = 0; // 0 keeps the clip's own length
    bool originSet = false;
    math::Vector3 origin;
    bool anglesSet = false;
    math::Vector3 angles; // pitch, yaw, roll in degrees, as `angles` and `rotate` read them
    float scale = 1.0f;
    bool disableAnimBlocks = false, firstSectionLocal = false;
    float noStallFrames = 0.0f;
};

enum class AnimDeltaMode : std::uint32_t { None, Delta, PreDelta };

struct AnimSequenceData
{
    AnimProps anim;
    std::string activity;
    int activityWeight = 0; // 0 writes none
    bool hidden = false, autoPlay = false, realTime = false, snap = false, post = false;
    bool worldSpace = false, worldRelative = false;
    AnimDeltaMode delta = AnimDeltaMode::None;
    float fadeIn = 0.2f, fadeOut = 0.2f, exitPhase = 0.0f;
    std::string poseCycle, rootDriver;
    // `node` when both names are equal; `transition` otherwise, reversible for `rtransition`.
    std::string entryNode, exitNode;
    bool reversible = false;
    std::vector<compile::SeqEvent> events;
    std::vector<compile::SeqAnimTag> animTags;
};

struct AnimWeightEntry
{
    std::string bone;
    float weight = 0.0f;
    float posWeight = 0.0f;
    bool posSet = false; // false: the position weight follows the weight
};

struct AnimWeightData
{
    std::vector<AnimWeightEntry> entries;
};

// One pose parameter axis of a Blend node. `calc` measures the range from an attachment instead of min and max.
struct AnimBlendAxis
{
    std::string name;
    float min = 0.0f, max = 1.0f;
    bool wrap = false;
    float loop = 0.0f;
    bool calc = false;
    std::string attachment;
    int control = 0;
};

struct AnimBlendData
{
    int axes = 1; // 1 or 2
    AnimBlendAxis axis[2];
    int blendWidth = 0; // 0 lets the compiler choose
    std::string reference, compare, center;
};

// A 3x3 grid whose cells are the nine directions; `move` names the two pose parameters.
struct AnimWalkData
{
    std::string xName = "move_x", yName = "move_y";
    float min = -1.0f, max = 1.0f;
    bool swapAxes = false;
};

struct AnimLayerData
{
    bool add = false; // addlayer; otherwise blendlayer
    float start = 0, peak = 0, tail = 0, end = 0;
    bool xfade = false, spline = false, noBlend = false, local = false;
    bool usePose = false;
    std::string poseParameter;
};

struct AnimNode
{
    Uid uid = 0;
    AnimNodeType type = AnimNodeType::Clip;
    float x = 0, y = 0;
    bool muted = false, collapsed = false;
    // Animation, Sequence, Clip, Weight List and Blend: the name other nodes and the compiler know it by.
    // Pose Param: the declared pose parameter it stands for. Activity Modifier: the modifier itself.
    // Include Model: the .mdl path below models/. Declare Sequence: the included sequence it declares.
    std::string name;

    AnimClipData clip;
    AnimProps animation;
    AnimSequenceData sequence;
    AnimWeightData weights;
    AnimCommand command;
    AnimBlendData blend;
    AnimWalkData walk;
    AnimLayerData layer;
    compile::IkLock ikLock; // its `name` is the IK chain
    AnimIkRule ikRule;
};

// One declared pose parameter (`$poseparameter`). The order is the index the compiled model gives it.
struct AnimPoseParameter
{
    Uid uid = 0;
    std::string name;
    float min = 0.0f, max = 1.0f;
};

struct AnimLink
{
    Uid from = 0, to = 0;
    int fromSocket = 0, toSocket = 0;
};


struct Document
{
    std::string game;
    ModelDefinition definition;
    std::vector<SourceFile> sources;
    std::vector<MeshEntry> meshes;
    std::vector<ModelGroup> groups;
    MaterialMarkup materials;
    std::vector<BoneDefinition> bones;
    std::vector<RealignNode> realigns;
    std::vector<HierarchyNode> hierarchies;
    std::vector<Attachment> attachments;
    std::vector<Eyeball> eyeballs; // order is the eyeball index
    std::vector<EyelidNode> eyelids;
    std::vector<HitboxSet> hitboxSets;
    PhysicsSettings physics;
    std::vector<PhysicsNode> physicsNodes;
    std::vector<BoneMarkupNode> boneMarkups;
    std::vector<PrefabNode> prefabs;
    std::vector<LodNode> lods;
    LodSettings lodSettings;
    std::vector<FlexNode> flexNodes;
    std::vector<FlexLink> flexLinks;
    std::vector<FlexWrinkle> flexWrinkles;
    std::vector<FlexSplit> flexSplits;
    std::vector<std::string> flexLocalVars;
    std::vector<AnimNode> animNodes;
    std::vector<AnimLink> animLinks;
    std::vector<AnimPoseParameter> poseParameters;
    std::vector<Folder> folders;

    // Chunk tags this reader does not know and skipped; reported so nothing is dropped silently.
    std::vector<std::string> unknownChunks;

    const SourceFile* findSource(Uid uid) const;
    const MeshEntry* findMesh(Uid uid) const;
    const Folder* findFolder(Uid uid) const;
    const FlexNode* findFlexNode(Uid uid) const;
    const AnimNode* findAnimNode(Uid uid) const;
    HitboxSet* findHitboxSet(Uid uid);
    bool folderActive(Uid folder) const;
    // Sets every node's `enabled` from its own switch and its folders'.
    void refreshEnabled();
};

// Reads an .mdlasm. Throws std::runtime_error with a plain message when the file cannot be used.
Document LoadDocument(const std::string& path);

} // namespace pulse::mdlasm

#endif // PULSEMODEL_MDLASMDOC_H
