// compile.h - PulseMDL
//
// The compile stage: mirrors the reference SimplifyModel() pipeline
// Consumes the per-file Sources + script data, produces the
// CompiledModel the writers serialize.

#ifndef PULSEMDL_COMPILE_H
#define PULSEMDL_COMPILE_H

#include <memory>
#include <string>
#include <vector>

#include "math/math.h"
#include "pulselimits.h"
#include "source.h"

namespace pulse::compile {

namespace lim = pulse::limits;
using math::Vector2;
using math::Vector3;
using math::Vector4;
using math::Quaternion;
using math::RadianEuler;
using math::matrix3x4;

// attachment type bits. `type` is compile-time
// only and never written; `flags` goes to disk.
inline constexpr int kAttachIsAbsolute   = 0x0001; // "absolute" - local is in model space
inline constexpr int kAttachIsRigid      = 0x0002; // "rigid" - reparent to a vertex-bearing ancestor
inline constexpr int kAttachIsFromSource = 0x0004; // came from a source DMX, not the script
inline constexpr int kAttachIsDeclared   = 0x0008; // reserved by $declareattachment
inline constexpr int kAttachFlagWorldAlign = 0x10000; // ATTACHMENT_FLAG_WORLD_ALIGN

// one attachment point (reference s_attachment_t)
struct Attachment {
    std::string name;
    std::string bonename;
    int bone = -1;
    int type = 0;  // kAttachIs* bits, compile-only
    int flags = 0; // written to mstudioattachment_t.flags
    matrix3x4 local;

    // $attachment vertex selectors: every vertex a listed flex group / morph /
    // material touches is averaged into one point. With any of these set,
    // `local`'s translation is a model-space OFFSET from that average instead
    // of the final position - GenerateVertexAveragedAttachments resolves it.
    std::vector<std::string> flexgroups;
    std::vector<std::string> flexmorphs;
    std::vector<std::string> materials;

    bool HasVertexSelectors() const {
        return !flexgroups.empty() || !flexmorphs.empty() || !materials.empty();
    }
};

// one eyeball (reference s_eyeball_t). Authored values are the raw script
// numbers; the compile stage derives radius/zoffset/iris_scale and transforms
// org/up/forward into bone space.
struct Eyeball {
    std::string name;
    std::string bonename;
    int bone = -1;
    Vector3 org{};      // script model-space, then bone-space after setup
    bool center = false; // org is an offset (in final axes) from the material's bbox center
    std::string material;
    int model = -1;   // index into CompiledModel::models (owner)
    int mesh = -1;    // index into the owning model's meshes
    int texture = -1; // material index
    float radius = 0.0f;
    float zoffset = 0.0f;
    float iris_scale = 0.0f;
    Vector3 up{};
    Vector3 forward{};

    // lid data; -1 until an Eyelid binds them (or the dummy-desc fallback runs)
    int upperflexdesc[3] = {-1, -1, -1};
    int lowerflexdesc[3] = {-1, -1, -1};
    float uppertarget[3] = {0.0f, 0.0f, 0.0f};
    float lowertarget[3] = {0.0f, 0.0f, 0.0f};
    int upperlidflexdesc = -1;
    int lowerlidflexdesc = -1;
};

// one mouth (reference s_mouth_t)
struct Mouth {
    std::string controller; // flexdesc name
    std::string bonename;
    int bone = -1;
    int flexdesc = -1;
    Vector3 forward{};
};

// one $definebone entry (reference s_importbone_t)
struct ImportBone {
    std::string name;
    std::string parent; // "" = root
    matrix3x4 rawLocal;
    bool bPreAligned = false; // realign transform provided
    matrix3x4 srcRealign;     // identity unless bPreAligned
    bool bUnlocked = false;   // unlocked bones are appended AFTER the source union
};

// one bonemarkuplist entry: per-bone flags that used to be separate QC
// commands ($bonemerge, $donotcollapse). Both can be set on the same bone.
struct BoneMarkup {
    std::string name;
    bool doNotCollapse = false; // $donotcollapse - force-keep through the cull
    bool isBonemerge = false;   // $bonemerge - BONE_USED_BY_BONE_MERGE
};

// $transformbone <bone> [options]: a late bind-pose edit, applied by
// ApplyBoneTransformEdits after the global bone table is built and just before
// RealignBones. The default edit is pivot-only - the delta is folded into
// srcRealign, so neither the rest mesh nor any animation moves; only the bone's
// frame does. See compile.cpp for the mechanism
struct BoneTransformEdit {
    std::string name;
    bool hasAngles = false;     // `angles <p y r>`
    bool worldAngles = false;   // `worldangles` - rotate about world axes
    Vector3 angles{};           // euler DEGREES (pitch, yaw, roll)
    bool hasPosition = false;   // `position <x y z>`
    bool worldPosition = false; // `worldposition` - translate along world axes
    Vector3 pos{};
    // `transformweights <residualbone> [factor] [smoothing]`: hand this bone's
    // vertex weight to residualbone as a spatial ramp along the move axis
    bool hasMoveWeight = false;
    std::string residualbone;
    float moveWeightFactor = 1.0f;    // [0,1] band width (1 = whole span, 0 = hard cut)
    float moveWeightSmoothing = 0.0f; // [0,1] S-curve easing inside the band
    bool hasMoveWeightOffset = false; // `offset <x y z>` - shifts the ramp END only
    Vector3 moveWeightOffset{};
    bool transformVerts = false;  // carry the rigged verts instead of pivoting
    // `transformchildren`: move the bone's whole subtree rigidly with it, instead
    // of recompensating the children to stay where they were
    bool transformChildren = false;
    bool ignoreAnimation = false; // the edit never reaches any animation frame
    bool ignoreHitbox = false;    // keep this bone's authored $hbox where it was
    int line = 0;                 // script line, for diagnostics
};

// ---- procedural bones ----------------------------------------------------

// JiggleBone::flags (format/mdl.h JIGGLE_*, mirrored like the other flag sets
// in this header so compile.h stays free of the format headers).
inline constexpr int kJiggleIsFlexible = 0x01;
inline constexpr int kJiggleIsRigid = 0x02;
inline constexpr int kJiggleHasYawConstraint = 0x04;
inline constexpr int kJiggleHasPitchConstraint = 0x08;
inline constexpr int kJiggleHasAngleConstraint = 0x10;
inline constexpr int kJiggleHasLengthConstraint = 0x20;
inline constexpr int kJiggleHasBaseSpring = 0x40;
inline constexpr int kJiggleIsBoing = 0x80;

// one `jigglebonelist` entry (QC $jigglebone, reference s_jigglebone_t). The
// bone is simulated by the engine, so it is tagged BONE_ALWAYS_PROCEDURAL:
// nothing animates it and it is skipped by the srcbonetransform table.
//
// Field names and defaults mirror mstudiojigglebone_t / Cmd_JiggleBone exactly;
// the writer copies them straight across. Angles are stored in RADIANS (the
// script authors degrees), and boingImpactAngle is stored as a cosine.
struct JiggleBone {
    std::string bonename;
    int bone = -1; // resolved against the final bone table by MapJiggleBones

    int flags = 0;

    float length = 10.0f;
    float tipMass = 0.0f;

    float yawStiffness = 100.0f;
    float yawDamping = 0.0f;
    float pitchStiffness = 100.0f;
    float pitchDamping = 0.0f;
    float alongStiffness = 100.0f;
    float alongDamping = 0.0f;

    float angleLimit = 0.0f;

    float minYaw = 0.0f;
    float maxYaw = 0.0f;
    float yawFriction = 0.0f;
    float yawBounce = 0.0f;

    float minPitch = 0.0f;
    float maxPitch = 0.0f;
    float pitchFriction = 0.0f;
    float pitchBounce = 0.0f;

    float baseMass = 0.0f;
    float baseStiffness = 100.0f;
    float baseDamping = 0.0f;
    float baseMinLeft = -100.0f;
    float baseMaxLeft = 100.0f;
    float baseLeftFriction = 0.0f;
    float baseMinUp = -100.0f;
    float baseMaxUp = 100.0f;
    float baseUpFriction = 0.0f;
    float baseMinForward = -100.0f;
    float baseMaxForward = 100.0f;
    float baseForwardFriction = 0.0f;

    // Boing stays ZERO unless the script gives a `boing` block. Cmd_JiggleBone
    // seeds the fields above unconditionally but seeds these five inside
    // ParseBoingJiggle, so a non-boing jigglebone writes zeros here.
    float boingImpactSpeed = 0.0f;
    float boingImpactAngle = 0.0f; // a cosine, not an angle
    float boingDampingRate = 0.0f;
    float boingFrequency = 0.0f;
    float boingAmplitude = 0.0f;
};

// one `triggerlist` entry (reference s_quatinterpbone_t's parallel arrays).
// Everything here is already RADIANS/quaternions - the loader converts the
// authored degrees.
struct ProceduralBoneTrigger {
    // angular width of this trigger's influence, radians. The writer stores
    // 1/tolerance, computed in double.
    float tolerance = 0.0f;
    // driver orientation that fully activates this trigger, relative to the
    // driver's parent.
    Quaternion trigger{0.0f, 0.0f, 0.0f, 1.0f};
    // helper pose at full influence, relative to the helper's parent. Authored
    // as a DELTA from the helper's bind pose; MapProceduralBones folds the bind
    // pose in, so from that pass onward these are absolute parent-relative.
    Vector3 pos{};
    Quaternion quat{0.0f, 0.0f, 0.0f, 1.0f};
    // pos/quat are already parent-relative in full, so skip the bind-pose fold.
    // Per-trigger: one $driverbone block can mix `trigger` (the block's
    // relative/absolute mode) with `posetrigger` (always absolute).
    bool absolutePose = false;
};

// one `animconstraintlist` entry (QC $driverbone / VRD <helper>, reference
// s_quatinterpbone_t). The helper bone's pose is interpolated between the
// triggers according to how closely the driver bone matches each trigger
// orientation.
//
// Like a jigglebone the helper is tagged BONE_ALWAYS_PROCEDURAL - the engine
// owns it, so it carries no animation data and no srcbonetransform.
struct ProceduralBone {
    std::string helpername; // driven bone
    std::string drivername; // control bone whose rotation is tested
    int helper = -1;        // resolved by MapProceduralBones
    int driver = -1;

    // Parent names are optional in the script. MapProceduralBones fills an
    // empty one from the resolved skeleton; RemapProceduralBones then checks
    // the recorded parent still matches, which is what catches a bone that
    // collapsed out from under the constraint.
    std::string helperparentname;
    std::string driverparentname;

    // false lets a name match a skeleton bone's dotted suffix ("Bip01_R_Thigh"
    // resolves "ValveBiped.Bip01_R_Thigh") - the VRD form only.
    bool strictName = true;

    std::vector<ProceduralBoneTrigger> triggers;
};

// one aim-at procedural bone (QC $driveraimat / VRD <aimconstraint>, reference
// s_aimatbone_t). The engine rotates the bone so its aimvector points at a
// target attachment or bone, so like the other procedural kinds it is tagged
// BONE_ALWAYS_PROCEDURAL.
//
// The reference's g_aimatbones is a zero-initialized global, so an absent
// <aimvector>/<upvector> really does write zeros - the defaults below match.
struct AimAtBone {
    std::string bonename;   // the driven bone
    std::string parentname; // "" = derive from the resolved skeleton
    std::string aimname;    // an attachment name, else a bone name

    int bone = -1;
    int parent = -1;
    // An attachment target wins over a bone of the same name and picks
    // STUDIO_PROC_AIMATATTACH; aimBone (STUDIO_PROC_AIMATBONE) is the fallback.
    int aimAttach = -1;
    int aimBone = -1;

    Vector3 aimvector{};
    Vector3 upvector{};
    Vector3 basepos{};
    // $driveraimat does not ask for a base position: MapAimAtBones seeds it
    // from the bone's rest pose once the skeleton is final. The VRD form states
    // it outright, so it opts out. (Reference s_aimatbone_t::autobasepos.)
    bool autobasepos = false;
    // see ProceduralBone::strictName
    bool strictName = true;
};

// bone_cull_type: how hard the skeleton is pruned of bones nothing references.
// A bone is "referenced" when something other than plain vertex weighting needs
// it - animation, IK, an attachment, bonemerge, a flex driver, $donotcollapse.
// Skinning alone never saves a bone: a skin-only bone folds its weights into
// its parent.
enum class BoneCullType {
    Aggressive = 0, // drop every unreferenced bone, reparenting its children
    LeafOnly = 1,   // drop unreferenced bones only while they have no children
    None = 2,       // keep everything (non-skeletal bones excepted - exporter noise)
};

// global bone table entry (reference s_bonetable_t subset)
struct Bone {
    std::string name;
    int parent = -1;
    int flags = 0;
    int group = 0; // hitgroup
    bool isNonSkeletal = false;
    bool bPreDefined = false;  // created by a locked $definebone (not a source)
    bool bDontCollapse = false;
    bool bPreAligned = false;  // $definebone provided the realign - RealignBones skips it
    int physicsbone = 0;       // index into the .phy solid list; 0 with no physics
    Vector3 pos;      // local, rebuilt from boneToPose
    RadianEuler rot;  // local, rebuilt from boneToPose
    matrix3x4 rawLocal;
    matrix3x4 boneToPose;
    matrix3x4 poseToBone;
    // RealignBones always computes this, even with no $realignbones: it is
    // Invert(boneToPose)*boneToPose - a NOISY near-identity, not exact.
    matrix3x4 srcRealign;
    Vector3 posscale{1.0f / 32.0f, 1.0f / 32.0f, 1.0f / 32.0f};
    Vector3 rotscale{1.0f / 32.0f, 1.0f / 32.0f, 1.0f / 32.0f};
    // total world travel of this bone across every animation, per axis. Only
    // read by the automatic $bonesaveframe pass (minZeroFramePosDelta).
    Vector3 posrange{0.0f, 0.0f, 0.0f};
    Quaternion qAlignment{0, 0, 0, 0}; // zero unless $limitrotation
    Vector3 bmin, bmax; // auto-hitbox extents (bone local)
    std::string surfaceprop; // empty = model default
    int contents = 0;        // resolved word, filled by ApplyJointContents
};

// per-channel RLE data for one bone in one section (reference s_animanim_t)
struct AnimChannels {
    int num[6] = {};
    std::vector<uint16_t> data[6]; // raw mstudioanimvalue_t words
};

// $weightlist (reference s_weightlist_t). Entries as authored; the resolved
// per-global-bone arrays are built by buildAnimationWeights.
struct WeightList {
    std::string name;
    struct Entry {
        std::string bone;
        float weight = 0;
        float posweight = 0;
    };
    std::vector<Entry> entries;
    // resolved per global bone (index into CompiledModel::bones)
    std::vector<float> weight;
    std::vector<float> posweight;
};

// $poseparameter (reference s_poseparameter_t)
struct PoseParam {
    std::string name;
    float min = 0;
    float max = 0;
    int flags = 0; // STUDIO_LOOPING
    float loop = 0;
};

// $ikchain (reference s_ikchain_t); numlinks fixed at 3 by LinkIKChains
struct IkChain {
    std::string name;
    std::string bonename; // end bone (ankle/foot); links walk parents
    struct Link {
        int bone = -1;
        Vector3 kneeDir; // only link[0] ever authored
    };
    Link link[3];
    float height = 18.0f;
    float radius = 0.0f; // pad / 2
    float floor = 0.0f;
    Vector3 center;
};

// sequence iklock / $ikautoplaylock (reference s_iklock_t)
struct IkLock {
    std::string name; // chain name until resolved
    int chain = -1;
    float flPosWeight = 0;
    float flLocalQWeight = 0;
};

// per-frame ik error samples + compressed streams (reference
// s_animationstream_t / s_streamdata_t)
struct AnimStream {
    struct Sample {
        Vector3 pos;
        Quaternion q;
    };
    std::vector<Sample> error; // numerror entries
    float scale[6] = {};
    int numanim[6] = {};
    std::vector<uint16_t> data[6]; // raw mstudioanimvalue_t words
};

// one ik rule on an animation (reference s_ikrule_t)
struct IkRule {
    int chain = 0;
    int index = 0;
    int type = 0; // IK_*
    int slot = 0;
    std::string bonename;   // touch target (IK_SELF); "" = worldspace
    std::string attachment; // IK_ATTACHMENT world attachment name
    // calloc parity: the reference rules live in a calloc'd array, so bone/q/
    // contact stay ZERO unless a code path assigns them (IK_SELF and
    // fakeorigin/fakerotate set bone = -1; the QC parser sets contact = -1 on
    // every PARSED rule - mirrored by InIkRule - while auto IK_RELEASE rules
    // keep the zeros).
    int bone = 0;
    Vector3 pos;
    Quaternion q{0, 0, 0, 0};
    float height = 0;
    float floor = 0;
    float radius = 0;
    int start = 0, peak = 0, tail = 0, end = 0; // frames (-1 = unset)
    int contact = 0;
    bool usesequence = false;
    bool usesource = false;
    int flags = 0;
    AnimStream errorData;
};

// autolayer spec (reference s_autolayer_t)
struct AutoLayer {
    std::string name; // target sequence name
    int sequence = -1;
    int flags = 0; // STUDIO_AL_*
    // pose parameter index when STUDIO_AL_POSE; the reference callocs the
    // sequence, so a layer without a pose parameter writes iPose = 0 (not -1)
    int pose = 0;
    float start = 0, peak = 0, tail = 0, end = 0; // frames until write-time conversion
};

// `transformbone <bone> [origin x y z] [angles p y r]`: one bone's offset,
// applied to every frame of the clip and never weighted - it moves the whole
// clip rather than blending into part of it. Payload of AnimCmd::TransformBone.
struct AnimBoneTransform {
    std::string bone;
    Vector3 origin{};      // added to the local translation (parent space)
    RadianEuler angles{};  // composed onto the local rotation (the bone's space)
    bool originSet = false, anglesSet = false;
};

// per-animation command (reference s_animcmd_t subset)
struct AnimCmd {
    enum Kind { Weights, Subtract, Reverse, FixupLoop, Angle, Align, Match,
                MatchBlend, WorldspaceBlend, AppendAnim, BoneDriver,
                Motion, RefMotion, CopyPose, TransformBone, NumFrames,
                IkFixup } kind = Weights;
    int weightlistIndex = 0; // Weights: index into CompileInput::weightlists+1 space
    int numframes = 0;       // NumFrames: the length to clip or pad to
    int subtractAnim = -1;   // Subtract: index into CompiledModel::anims
    int subtractFrame = 0;
    int subtractFlags = 0; // STUDIO_POST for "subtract", 0 for "presubtract"
    int fixupStart = 0;    // FixupLoop: negative, frames back from the end
    int fixupEnd = 0;      // FixupLoop: positive, frames in from the start
    float angle = 0.0f;    // Angle: `rotateto`, degrees
    // Align/Match: the reference animation, plus (Align) which bone to line up,
    // which motion components to match, and the frame of each animation
    int refAnim = -1;      // Align/Match/AppendAnim
    std::string alignBone; // Align/BoneDriver: "" = the root bone
    int motiontype = 0;    // STUDIO_X/Y/Z/XR/YR/ZR bits
    int srcframe = 0;
    int destframe = 0;
    // BoneDriver: overwrite one position axis of a bone, flat across the clip
    // or smoothstep-ramped over start/peak/tail/end
    int driverAxis = 0;
    float driverValue = 1.0f;
    bool driverAll = true;
    int driverStart = 0, driverPeak = 0, driverTail = 0, driverEnd = 0;
    // MatchBlend: how many frames either side of destframe the match ramps over
    int matchPre = 0, matchPost = 0;
    // Motion/RefMotion (walkframe / walkalignto): extract motion up to endFrame
    int motionEndFrame = 0;
    // WorldspaceBlend: `worldspaceblendloop` pulls from a looping source, so it
    // samples by cycle instead of taking one fixed frame
    bool worldLoops = false;
    // CopyPose: hold refAnim's frame srcframe across destframe..copyEndFrame,
    // smoothstep-faded over fadeIn/fadeOut frames either side of that range.
    // copyPos/copyRot pick which channels are taken; both by default.
    // copyBindPose: the source is the compile's own bind pose, not a clip -
    // refAnim is unused and nothing can have edited it
    bool copyBindPose = false;
    int copyEndFrame = 0;
    float fadeIn = 0.0f, fadeOut = 0.0f;
    bool copyPos = true, copyRot = true;
    AnimBoneTransform xform; // TransformBone
    IkRule ikfixup;          // IkFixup: the rule this command bakes in
};

// one extracted motion segment (reference s_linearmove_t) - written as
// mstudiomovement_t. Produced by motion extraction, one per motion command.
struct LinearMove {
    int endframe = 0;
    int flags = 0;      // the motiontype that produced it
    float v0 = 0.0f;    // speed at the segment start
    float v1 = 0.0f;    // speed at the end
    Vector3 pos{};      // accumulated position at endframe
    RadianEuler rot{};  // accumulated rotation at endframe
    Vector3 vector{};   // normalized direction of travel
};

// $animationcullmethod: which $animations are dropped before the pipeline runs.
// Replaces the reference's -cullanims launch option, whose cull is what
// Aggressive does. See CullAnimations() in compile.cpp.
//
// Only animations are ever removed - a cull never drops a sequence, and a
// sequence whose animations all survive is bit-identical to an unculled build.
enum class AnimCullMethod {
    Aggressive = 0, // duplicates, then every animation no sequence reaches
    Duplicates = 1, // merge identically-defined animations only
    None = 2,       // ship every animation the script declared
};

// one $animation (reference s_animation_t subset)
struct Anim {
    std::string name;     // "@<seq>" for implied animations
    source::Source* src = nullptr;
    std::string animationname; // source anim to sample ("" = none)
    int startframe = 0;
    int endframe = 0;
    int numframes = 1;
    float fps = 30.0f;
    int flags = 0;
    int motiontype = 0;
    float scale = 1.0f;
    bool ignorescale = false;
    bool fudgeloop = false;          // append the missing duplicate end frame
    int looprestart = 0;             // realignLooping: rotate the clip
    float looprestartpercent = 0.0f;
    bool isOverride = false;  // $declareanimation slot
    bool doesOverride = false;
    bool isBindPose = false;  // sanim comes from the bone table, not from src
    bool noAutoIK = false;
    bool nocull = false;      // `nocull`: exempt from $animationcullmethod
    // `ignoretransformbone <angles|position>`: convert this clip with that
    // category of $transformbone bind-pose edits rolled back
    bool ignoreTransformAngles = false;
    bool ignoreTransformPosition = false;
    // motion extraction output (piecewisemove); motionrollback is how far back
    // a non-looping clip starts its extraction, in seconds
    std::vector<LinearMove> piecewisemove;
    float motionrollback = 0.3f;
    std::vector<AnimCmd> cmds; // executed in order by ProcessAnimations
    std::vector<IkRule> ikrules;
    Vector3 adjust;       // $origin shift
    RadianEuler rotation; // g_defaultrotation unless delta
    std::vector<float> weight;    // per global bone
    std::vector<float> posweight; // per global bone
    // sanim[frame][globalbone]
    std::vector<std::vector<source::SrcBonePose>> sanim;
    // demand loading ($animblocksize) - see CompileInput::animblocksize
    bool disableAnimblocks = false;
    bool isFirstSectionLocal = false;
    float numNostallFrames = 0.0f;
    // compression output
    int sectionframes = 0;
    int numsections = 1;
    std::vector<std::vector<AnimChannels>> anim; // [section][bone]
    Vector3 bmin, bmax;
};

// one $sequence `event <name> <frame> [options]` (mstudioevent_t). A numeric
// name is an old-style id; anything else is written by name with
// NEW_EVENT_STYLE. Shared by CompileInput::InSequence and Sequence - the writer
// needs exactly what the script said.
struct SeqEvent {
    std::string name;
    int frame = 0;
    std::string options; // char[64] on disk
};

// one $sequence `animtag <name> <cycle>` (mstudioanimtag_t). The tag id is
// resolved at runtime, so only the name and cycle come from the script.
struct SeqAnimTag {
    std::string name;
    float cycle = 0.0f;
};

struct Sequence {
    std::string name;
    std::string activityname; // written to szactivitynameindex
    int flags = 0;
    int activity = -1;
    int actweight = 0;
    float fadeintime = 0.2f;
    float fadeouttime = 0.2f;
    float exitphase = 0.0f;
    // rootdriver: the name is carried so the bone can be resolved once the
    // final bone table exists; the writer only sees the index
    std::string rootdriverBone;
    int rootDriverBone = 0;
    // calcblend, resolved: attachment index (-1 = authored `blend` range) and
    // the motion control the pose value is read from
    int paramattachment[2] = {-1, -1};
    int paramcontrol[2] = {0, 0};
    int paramanim = -1;     // blendref, index into CompiledModel::anims
    int paramcompanim = -1; // blendcomp
    int paramcenter = -1;   // blendcenter
    int groupsize[2] = {1, 1};
    int paramindex[2] = {-1, -1};
    float paramstart[2] = {0, 0};
    float paramend[2] = {0, 0};
    int numblends = 1;
    std::vector<int> animIndices; // groupsize[0]*groupsize[1] entries into anims
    std::vector<float> param0; // posekeys along x (per grid column)
    std::vector<float> param1; // posekeys along y (per grid row)
    std::vector<AutoLayer> autolayers;
    std::vector<IkLock> iklocks;
    std::vector<SeqEvent> events;
    std::vector<SeqAnimTag> animtags;
    std::vector<std::string> activitymodifiers; // lowercased at parse
    std::string keyvalues;                      // raw text block, "" = none
    // transition graph, 1-based node ids (0 = no node)
    int entrynode = 0;
    int exitnode = 0;
    int nodeflags = 0; // 1 = the transition is reversible ($rtransition)
    int numikrules = 0; // max over the blend grid's anims
    int cycleposeindex = 0;
    std::vector<float> weight;    // per global bone
    Vector3 bmin, bmax;
};

// ---- flex / morph (reference g_flexdesc / g_flexcontroller / g_flexrule /
// g_flexkey; built by the loader's registration pass, consumed by
// RemapVertexAnimations in the compile stage and the mdl writer) ----

// $flexcullmethod: how hard the flex tables are pruned after registration.
// Replaces the reference's -cullmorphs / -cullflex launch options, which are
// both folded into Aggressive. See CullFlex() in compile.cpp.
//
// Aggressive is reachability-based, and a morph is only reachable through a
// flex rule: a model whose delta states carry no rig (no $datamodelflexes, no
// hand-authored $flexrule) has nothing driving them and loses all of them.
enum class FlexCullMethod {
    Aggressive = 0, // duplicates, then drop everything nothing can reach
    Duplicates = 1, // merge same-name controllers, drop repeated rules only
    None = 2,       // ship everything the script and the DMX rig registered
    RulesOnly = 3,  // aggressive, but every flex controller survives (default)
};

struct FlexDesc {
    std::string name; // FACS name (string table)
};

struct FlexController {
    std::string name;
    std::string type; // flex group ("default"); falls back to the name
    float min = 0.0f;
    float max = 1.0f;
};

// one RPN op (matches mstudioflexop_t; d is a raw 4-byte union on disk)
struct FlexOp {
    int op = 0;
    union {
        int index;
        float value;
    } d{};
};

struct FlexRule {
    int flex = 0; // flexdesc index
    std::vector<FlexOp> ops;
};

// one flexkey = one morph target on one model (reference s_flexkey_t)
struct FlexKey {
    source::Source* source = nullptr;
    std::string animationname; // delta state (morph) name
    int imodel = 0;            // flattened model index (blanks included)
    int flexdesc = 0;
    int flexpair = 0; // stereo: flexdesc = "<name>L", flexpair = "<name>R"
    float target0 = 0.0f;
    float target1 = 1.0f;
    float target2 = 10.0f;
    float target3 = 11.0f;
    float split = 0.0f;
    float decay = 1.0f;
    int frame = 0;
    // filled by RemapVertexAnimations (compile stage)
    std::vector<source::SrcVertAnim> vanim;
    uint8_t vanimtype = 0; // STUDIO_VERT_ANIM_NORMAL/WRINKLE
};

// one `BoneMorphDriverControl` (reference CDmeBoneFlexDriverControl): a bone
// translation component drives a flex controller over [min,max] -> [0,1].
struct BoneFlexDriverControl {
    std::string controllername;
    int component = 0; // fmt::STUDIO_BONE_FLEX_TX/TY/TZ
    float min = 0.0f;
    float max = 1.0f;
    int flexControllerIndex = -1; // resolved by TagFlexDriverBones
};

// one `BoneMorphDriver` (reference CDmeBoneFlexDriver), keyed by bone name
struct BoneFlexDriver {
    std::string bonename;
    int bone = -1; // resolved by MapFlexDriverBonesToGlobalBoneTable
    std::vector<BoneFlexDriverControl> controls;
};

// ---- physics -------------------------------------------------------------
// The .pulsemdl schema drops QC's $collisionmodel / $collisionjoints split.
// There is one set of lists, and the compile stage auto-detects: collision
// geometry resolving to a single physics bone is a single body, more than one
// is a ragdoll.

// How a shape gets its geometry. Both kinds live in the same
// `physicsshapelist`; the element's class name picks the kind.
enum class PhysicsShapeKind {
    FromFile,   // "PhysicsShapeFromFile"   - an authored collision mesh
    FromRender, // "PhysicsShapeFromRender" - hulls generated off the render mesh
};

// How a `PhysicsShapeFromFile` maps its source's skinning onto the model.
enum class PhysicsImportType {
    Skinned = 0,   // keep the source's own joints - the mesh is already rigged
    ToOneBone = 1, // reskin every part onto parent_bone, whatever it was rigged to
    OneBoneOnly = 2, // keep only the geometry on parent_bone, discard the rest
};

// one shape in `physicsshapelist` - a source of collision geometry.
//
// FromFile and FromRender share enough (the offset/scale framing, the naming,
// the convex budget) that splitting them into two structs would duplicate the
// plumbing through the whole compile stage for the sake of a handful of unused
// fields. `kind` says which half is live.
struct PhysicsShape {
    PhysicsShapeKind kind = PhysicsShapeKind::FromFile;
    std::string name;

    // ---- shared framing, applied to the gathered geometry in source space ---
    Vector3 offsetOrigin{};
    Vector3 offsetAngles{};           // degrees, pitch yaw roll
    bool offsetAnglesSet = false;
    float importScale = 1.0f;

    // the bone the geometry parents to. Required for FromRender, and for
    // FromFile under import types 1 and 2.
    std::string parentBone;

    // ---- FromFile ---------------------------------------------------------
    source::Source* source = nullptr; // resolved from the rendermesh reference
    PhysicsImportType importType = PhysicsImportType::Skinned;
    bool concave = false;             // split disjoint mesh islands into pieces
    int maxConvex = lim::kMaxConvexPieces;
    bool remove2d = false;

    // ---- FromRender -------------------------------------------------------
    // Fraction of each generated hull's natural vertex count to keep; 1 keeps
    // the full hull (capped at lim::kMaxHullVerts).
    float decimationFactor = 0.22f;
    float concavity = 0.04f;          // [0..1], lower = tighter fit, more pieces
    int maxHulls = 0;                 // piece ceiling, not a target; 0 = uncapped
    // minimum weight a vertex must carry on a kept bone for its face to join
    // this body. Smooth-skinned meshes need this well below the default.
    float cullWeight = 0.42f;
    // bones whose weighted geometry joins this body beyond parentBone
    std::vector<std::string> extraSkinnedBones;
    // rendermeshes to leave out of the generation, resolved to their sources -
    // an exclusion list, so by default every body's render geometry is used
    std::vector<std::string> exceptionMeshNames;
    std::vector<const source::Source*> exceptionSources;
};

// one axis of a `PhysicsJoint` constraint
struct PhysicsJointAxis {
    int axis = 0;      // 0 = x, 1 = y, 2 = z
    int type = 0;      // 0 = free, 1 = limit, 2 = fixed
    float min = 0.0f;
    float max = 0.0f;
    float friction = 1.0f; // an omitted `friction` on an axis, not "no friction"
};

// one `PhysicsJoint` - ragdoll constraint on a bone. Parsed by the loader,
// consumed when the ragdoll is built.
struct PhysicsJoint {
    std::string bonename;
    int bone = -1; // resolved by BuildCollisionModel
    std::vector<PhysicsJointAxis> axes;
};

// one `PhysicsMarkup` - per-body override, keyed by bone. Every field except
// the bone is optional and falls back to the physicsmodifierlist default, so
// each carries a *Set flag: an authored 0 must beat the default.
//
// The ragdoll body-partitioning fields live here rather
// than on PhysicsJoint because "this bone does / does not get its own body" is
// a body decision, and a markup child is already the one-entry-per-bone place
// to say it. QC authored the merge on the parent ($jointmerge parent child);
// we author it on the child (`merge_into` names the parent) so a bone still
// only ever appears once in the list.
struct PhysicsMarkup {
    std::string name;
    std::string bonename;
    float massBias = 1.0f;    bool massBiasSet = false;
    float inertia = 1.0f;     bool inertiaSet = false;
    float damping = 0.0f;     bool dampingSet = false;
    float rotdamping = 0.0f;  bool rotdampingSet = false;
    bool skip = false;            // $jointskip - this bone gets no body
    std::string mergeInto;        // $jointmerge - fold this bone into that one
};

// one allowed collision pair ($jointcollide). Symmetric, so it is stored on
// neither bone's markup - the .phy holds a flat pair list and so do we.
struct PhysicsCollidePair {
    std::string a;
    std::string b;
};

// one built physics body - what becomes a `solid { }` in the .phy
struct PhysicsSolid {
    std::string name;
    std::string parent;      // empty for the root body
    int parentIndex = -1;    // parent's solid index, -1 = no parent
    int bone = 0;
    float volume = 0.0f;
    float surfaceArea = 0.0f;
    float massBias = 1.0f;
    float mass = 1.0f;    // computed at write time from volume share
    float damping = 0.0f;
    float rotdamping = 0.0f;
    float inertia = 1.0f;
    float drag = -1.0f;   // -1 = omit the key entirely
    std::string surfaceprop;
    std::vector<uint8_t> blob; // serialized IVP compact surface
    // ragdoll constraint on the joint between this body and its parent,
    // resolved from physicsjointlist (reference BuildRagdollConstraint).
    // An axis nobody constrained stays 0/0/0 - locked, matching the
    // reference's memset - so the defaults here are NOT "free".
    float axisMin[3] = {0, 0, 0};
    float axisMax[3] = {0, 0, 0};
    float axisFriction[3] = {0, 0, 0};
};

struct HitBox {
    std::string bonename; // authored; SetupHitBoxes resolves it into `bone`
    int bone = 0;
    int group = 0;
    Vector3 bmin, bmax;
    Vector3 angOffset;          // OBB rotation, (pitch yaw roll) DEGREES on disk
    float capsuleRadius = 0.0f; // 0 = box, not a capsule
    std::string name; // empty = unnamed
};

struct HitboxSet {
    std::string name;
    std::vector<HitBox> hitboxes;
};

// final per-model render data (reference s_loddata_t)
struct LodVertex {
    source::SrcBoneWeight boneweight; // capped to kMaxBoneWeights, sorted desc
    Vector3 position;
    Vector3 normal;
    Vector2 texcoord;
    Vector4 tangentS;
    int lodFlag = 1; // bit per LOD that references this vertex
};

// one entry of a $lod / $shadowlod block (reference CLodScriptReplacement_t).
// Which fields matter depends on the list it lives in.
struct LodReplacement {
    std::string src;                  // as authored, for messages
    std::string dst;
    source::Source* srcSource = nullptr; // replacemodel/decimatemodel: the model matched
    source::Source* source = nullptr;    // replacemodel: the geometry to swap in
    float decimation = 1.0f;             // decimatemodel factor
};

// one $lod / $shadowlod block (reference LodScriptData_t). Index 0 is the
// implicit root LOD the loader always creates.
struct ScriptLod {
    float switchValue = 0.0f; // -1 = $shadowlod
    std::vector<LodReplacement> modelReplacements;
    std::vector<LodReplacement> generateLods;      // decimatemodel
    std::vector<LodReplacement> boneReplacements;  // replacebone
    std::vector<LodReplacement> boneTreeCollapses; // bonetreecollapse
    std::vector<LodReplacement> materialReplacements;
    std::vector<LodReplacement> meshRemovals;
    // removemeshword: drop any mesh whose material base name CONTAINS the
    // stored word, case-insensitively
    std::vector<LodReplacement> meshWordRemovals;
    bool facialAnimation = true;     // cleared by nomorphs/nofacial
    // use_shadowlod_materials, only ever set on a $shadowlod block
    bool useShadowLodMaterials = false;
    float decimateAllFactor = -1.0f; // decimateallmodel, <= 0 = unset
    bool IsShadow() const { return switchValue < 0.0f; }
    bool HasDecimateAll() const { return decimateAllFactor > 0.0f; }
};

// One output mstudiomesh_t. Usually one per material, but a material holding
// more than lim::kMaxMeshVerts verts is split across several - origMeshVertID
// in the .vtx is a uint16 mesh-relative index, so a mesh cannot address past
// that. Several meshes sharing a material index is legal: nothing picks between
// meshes (only bodyparts are selectable), so the renderer draws them all.
struct OutMesh {
    int material = 0;
    int vertexoffset = 0; // into Model::vertices
    int numvertices = 0;
    // per script LOD, this mesh's triangles, mesh-relative to vertexoffset.
    // Empty where the LOD does not draw the mesh (removemesh, or the LOD's
    // source carries no geometry for the material).
    std::vector<std::vector<source::SrcFace>> lodFaces;
};

struct Model {
    std::string name; // "blank" for the empty choice
    source::Source* source = nullptr;
    // vertex dictionary shared by every LOD (reference s_loddata_t::vertex)
    std::vector<LodVertex> vertices;
    std::vector<source::SrcFace> faces;                 // LOD 0, mesh-relative
    std::vector<source::SrcMesh> meshes;                // by material, sized to kMaxSkins
    // what the writers emit: mesh order and vertex ranges, after any oversized
    // material has been split (BuildOutputMeshes)
    std::vector<OutMesh> outMeshes;
    float boundingradius = 0.0f;
    // per script LOD: the geometry that LOD draws (null = model removed there)
    std::vector<source::Source*> lodSources;
    // per script LOD: source vertex index -> mesh-relative dictionary index,
    // sized to that LOD source's globalVertices (reference pMeshVertIndexMaps)
    std::vector<std::vector<int>> meshVertIndexMaps;
};

struct BodyPart {
    std::string name;
    int base = 1;
    std::vector<int> modelIndices; // into CompiledModel::models
};

// $modelgrouppreset, resolved against the finished bodypart bases
struct BodyGroupPreset {
    std::string name;
    int value = 0;
    int mask = 0;
};

// one $bonesaveframe entry: which components of this bone are kept in the
// zero-frame cache that stands in for a not-yet-loaded .ani block.
struct BoneSaveFrame {
    std::string name;
    bool savePos = false;
    bool saveRot = false;   // Quaternion32, or 64 with `cachehighres`
    bool saveRot64 = false; // always Quaternion64
};

struct CompiledModel {
    std::string outname; // e.g. "testmodel/testcube" (no extension)
    int gflags = 0;
    float mass = 1.0f;
    std::string surfaceprop = "default";
    std::string keyvalues; // $keyvalues text block, "" = none
    int contents; // CONTENTS_SOLID
    Vector3 eyeposition;
    Vector3 illumposition;
    // studiohdr2: 1-based attachment index the illumination position follows,
    // 0 = the static illumposition above ($illumposition with a bone)
    int illumpositionattachment = 0;
    Vector3 bbox[2];      // hull, from sequence 0 unless $bbox
    bool bboxset = false;
    Vector3 cbox[2];      // view clip, zeros unless $cbox
    bool cboxset = false;
    float maxEyeDeflection = 0;

    std::vector<Bone> bones;
    std::vector<Attachment> attachments;
    std::vector<Eyeball> eyeballs; // script order; all on the single $model
    std::vector<Mouth> mouths;     // indexed by the script's mouth index
    std::vector<Anim> anims;
    std::vector<Sequence> sequences;
    // transition graph: node names, plus the numxnodes^2 "next node on the way
    // from i to j" byte matrix (MakeTransitions)
    std::vector<std::string> xnodenames;
    std::vector<uint8_t> xnode; // row-major, xnodenames.size() square
    std::vector<HitboxSet> hitboxsets;
    std::vector<Model> models;
    std::vector<BodyPart> bodyparts;
    std::vector<BodyGroupPreset> bodygrouppresets;
    // $lod / $shadowlod blocks, index 0 = the implicit root LOD
    std::vector<ScriptLod> scriptLods;
    std::vector<PoseParam> poseparams;
    std::vector<IkChain> ikchains;       // links resolved to global bones
    std::vector<IkLock> ikautoplaylocks; // chain resolved

    // flex/morph (moved from CompileInput; vanims filled by
    // RemapVertexAnimations)
    std::vector<FlexDesc> flexdescs;
    std::vector<FlexController> flexcontrollers;
    std::vector<FlexRule> flexrules;
    std::vector<FlexKey> flexkeys;
    // global remap list for mstudioflexcontrollerui_t (AddBodyFlexRemaps)
    std::vector<source::ControllerRemap> flexControllerRemaps;
    // bonemorphdriverlist, pruned + resolved by the compile stage
    std::vector<BoneFlexDriver> boneflexdrivers;
    // jigglebonelist, pruned to the bones that survived and resolved to bone
    // indices by MapJiggleBones. Order is the writer's order.
    std::vector<JiggleBone> jigglebones;
    // animconstraintlist, pruned + resolved by MapProceduralBones and then
    // realigned by RemapProceduralBones. Order is the writer's order.
    std::vector<ProceduralBone> proceduralbones;
    // aim-at helpers, pruned + resolved by MapAimAtBones. Writer's order.
    std::vector<AimAtBone> aimatbones;
    // $includemodel names, already "models/"-prefixed. Written as
    // mstudiomodelgroup_t; the sequences themselves are linked at runtime.
    std::vector<std::string> includeModels;

    // $animblocksize settings, copied verbatim from CompileInput - the writer
    // is what acts on them. animblocksize 0 = no .ani, everything stays local.
    int animblocksize = 0;
    bool animblockHighRes = false;
    bool animblockLowRes = false;
    int maxZeroFrames = 3;
    bool zeroFramesHighres = false;
    float minZeroFramePosDelta = 2.0f;
    std::vector<BoneSaveFrame> boneSaveFrames;

    // physics, built by BuildCollisionModel. Empty = no .phy is written.
    std::vector<PhysicsSolid> physSolids;
    // collision hull bounds, applied to hull_min/hull_max by the .mdl writer
    // (reference CollisionModel_ExpandBBox, called from WriteModelFiles)
    Vector3 physCollideMins{}, physCollideMaxs{};
    bool physCollideBoundsSet = false;
    float physTotalMass = 1.0f;
    std::string physRootName;
    std::string physName;        // .phy filename override, empty = outname
    bool physConcave = false;    // reported in editparams
    bool physNoSelfCollisions = false;
    // ragdoll. Pairs are resolved to solid indices once the solid
    // order is final, so the writer only formats them.
    std::vector<std::pair<int, int>> physCollisionPairs;
    std::vector<std::pair<std::string, std::string>> physJointMerges; // parent,child
    bool physHasAnimatedFriction = false;
    int physAnimFrictionMin = 0;
    int physAnimFrictionMax = 0;
    float physAnimFrictionTimeIn = 0.0f;
    float physAnimFrictionTimeOut = 0.0f;
    float physAnimFrictionTimeHold = 0.0f;

    source::MaterialTable* mats = nullptr;
    std::vector<std::string> cdtextures;
    int numskinfamilies = 1;
    int numskinref = 0;
    // skinref[family][ref], identity unless $texturegroup adds families
    std::vector<std::vector<int16_t>> skinref;

    int FindBone(const char* name) const; // case-insensitive, -1 if absent
};

// model_archetype values (schema decision, 2026-07-16)
enum class Archetype {
    General, // "character" / "general" - normal skeletal model
    Static,  // "static" - $staticprop: bake verts, single "static_prop" bone
    Simple,  // "simple" - $simpleprop: single "prop_root" bone, verts NOT baked
};

// Script-level inputs the compile stage needs (filled by the .pulsemdl loader).
struct CompileInput {
    std::string outname;
    Archetype archetype = Archetype::General;
    // vtx_archetype: 0 = legacy StripGroup (TF2/L4D2), 1 = full (SFM/CS:GO).
    // int, not bool, to leave room for future .vtx variants.
    int vtxArchetype = 0;
    float scale = 1.0f;      // $scale
    Vector3 adjust;          // $origin translation (translatemodel)
    // rotatemodel, pre-converted to the reference's g_defaultrotation radians:
    // RadianEuler(DEG2RAD(x), DEG2RAD(y), DEG2RAD(z + 90)). (0,0,pi/2) = stock.
    RadianEuler rotation{0, 0, math::kPiF / 2.0f};
    bool rotationSet = false; // script gave rotatemodel (wins over upAxisY)
    bool upAxisY = false;    // from the DMX loader (switches defaultrotation)
    // render_pass: 0 = neither flag, 1 = $opaque (FORCE_OPAQUE),
    // 2 = $mostlyopaque (TRANSLUCENT_TWOPASS)
    int renderPass = 0;
    bool ambientBoost = false; // $ambientboost -> STUDIOHDR_FLAGS_AMBIENT_BOOST
    // $donotcastshadows -> STUDIOHDR_FLAGS_DO_NOT_CAST_SHADOWS
    bool doNotCastShadows = false;
    // $forcephonemecrossfade -> STUDIOHDR_FLAGS_FORCE_PHONEME_CROSSFADE
    bool forcePhonemeCrossfade = false;
    bool realignBones = false; // $realignbones: realign every single-child chain
    // $lockbonelengths: pin every bone to its bind-pose local translation, then
    // re-solve each ik chain so its end bone keeps its authored world position
    bool lockBoneLengths = false;
    // $skipboneinbbox (reference useBoneInBBox): the auto-hitbox bone extents
    // start empty instead of at the bone origin
    bool skipBoneInBBox = false;

    // $modelbudget: per-model ceilings, capped at the pulselimits.h values (a
    // budget above those is rejected at parse time). Bones default to 255 - the
    // last bone a uint8 weight/RLE index can reach - not the 1024 hard cap.
    int budgetBones = 255;
    int budgetMaterials = lim::kMaxSkins;

    // $bbox / $cbox, raw script values - no scale or rotation is applied
    // (Cmd_BBox/Cmd_CBox just read six floats). Unset leaves the writer's
    // defaults: the hull comes from sequence 0, the view box stays zero.
    Vector3 bbox[2];
    bool bboxSet = false;
    Vector3 cbox[2];
    bool cboxSet = false;

    // $illumposition, the point the engine samples lighting at. Unset leaves it
    // at sequence 0's box center. Like $bbox it takes no $scale. The bone form
    // also emits an attachment; this holds its name, "" = the position is
    // static (the compile stage resolves the name to illumpositionattachment).
    Vector3 illumposition;
    bool illumpositionSet = false;
    std::string illumpositionAttachment;

    // $eyeposition, the point the engine looks from. Unlike $illumposition it
    // IS scaled by $scale, applied in the compile stage.
    Vector3 eyeposition;
    // `autoheight`: Z is an offset from the rounded mean |z| of the eyeballs
    bool eyepositionAutoHeight = false;
    // $maxeyedeflection, pre-converted to the cosine the header stores
    // (Cmd_MaxEyeDeflection). 0 = unset, which the engine reads as cos(30).
    float maxEyeDeflection = 0.0f;

    std::vector<Attachment> attachments;   // script order
    // $hitboxset / $hbox, script order. Empty = the compile stage generates the
    // one "default" set from the bone-weighted vertex bounds and sets
    // STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX.
    std::vector<HitboxSet> hitboxsets;
    // facemarkuplist, in script order (order is significant: eyeballs must
    // register before any eyelid naming them, and it sets the eyeball index)
    std::vector<Eyeball> eyeballs;
    std::vector<Mouth> mouths;
    std::vector<ImportBone> importbones;   // script order
    // bonemarkuplist under "skeleton": $bonemerge + $donotcollapse merged
    std::vector<BoneMarkup> bonemarkups;
    // $alwayscollapse: force-collapse these bones. $donotcollapse wins.
    std::vector<std::string> alwaysCollapse;
    // $renamebone <from> <to>, script order. Applied to the final bone table at
    // the very end of the compile, so every other command still names the
    // source bone.
    std::vector<std::pair<std::string, std::string>> boneRenames;
    // $transformbone, script order. Applied as one batch after the bone table
    // is built, so the command is position-independent.
    std::vector<BoneTransformEdit> boneTransformEdits;
    // $hierarchy: reparent a bone once the global bone table exists. An empty
    // parent makes the child a root.
    struct ForcedHierarchy { std::string child, parent; };
    std::vector<ForcedHierarchy> forcedHierarchy;
    // bonemarkuplist container properties
    BoneCullType boneCullType = BoneCullType::Aggressive; // bone_cull_type
    // $root: the bone motion extraction, $alignto and $angle work from.
    // Unset (or unresolvable) means bone 0.
    std::string primaryRootBone;

    // weightlists (index 0 = the default list, entries from the
    // script start at 1), pose parameters, ik data
    // $defaultweightlist: authored entries for slot 0 (empty = plain all-1s)
    std::vector<WeightList::Entry> defaultWeights;
    std::vector<WeightList> weightlists;
    std::vector<PoseParam> poseparams;
    std::vector<IkChain> ikchains;
    std::vector<IkLock> ikautoplaylocks;

    // flex/morph global tables, fully registered by the loader
    // (auto per-body DMX combination data + top-level morphcontrollerlist /
    // morphrulelist). Compile() moves them into CompiledModel and fills the
    // per-key vanims (RemapVertexAnimations).
    std::vector<FlexDesc> flexdescs;
    std::vector<FlexController> flexcontrollers;
    std::vector<FlexRule> flexrules;
    std::vector<FlexKey> flexkeys;
    std::vector<source::ControllerRemap> flexControllerRemaps;
    // $flexcullmethod, applied by CullFlex() at the end of the compile stage
    FlexCullMethod flexCullMethod = FlexCullMethod::RulesOnly;

    // $animationcullmethod, applied by CullAnimations() right after the script
    // build - before the bone pipeline, so a dropped animation cannot keep a
    // bone alive (reference culls before RemapBones for the same reason)
    AnimCullMethod animCullMethod = AnimCullMethod::Aggressive;

    // $setbindpose <file> <frame>: re-skin every source vertex to that frame of
    // the file's animation, baking the pose into the rest mesh (reference
    // ApplyStaticPropPose). Loaded as an ANIMATION source, so
    // the pose file contributes no geometry and no materials. Null = unset; the
    // frame is clamped to the clip.
    source::Source* bindPoseSource = nullptr;
    int bindPoseFrame = 0;
    // $setflex <morph> <strength>: fixed morph amounts baked into the rest mesh,
    // in script order, on top of any $setbindpose. Strength clamps to [0,1].
    struct FixedFlex {
        std::string name;
        float value = 1.0f;
        int line = 0;
    };
    std::vector<FixedFlex> fixedFlexes;

    // skeleton/bonemorphdriverlist ($boneflexdriver)
    std::vector<BoneFlexDriver> boneflexdrivers;

    // skeleton/jigglebonelist ($jigglebone)
    std::vector<JiggleBone> jigglebones;
    // animconstraintlist ($driverbone / VRD quatinterp helpers)
    std::vector<ProceduralBone> proceduralbones;
    // aim-at helpers ($driveraimat / VRD <aimconstraint>)
    std::vector<AimAtBone> aimatbones;

    // physics. physicsshapelist / physicsjointlist /
    // physicsmarkuplist. No shapes = no .phy.
    std::vector<PhysicsShape> physShapes;
    std::vector<PhysicsJoint> physJoints;
    // physicsmarkuplist children
    std::vector<PhysicsMarkup> physMarkups;
    // $physicscollide - emitted in declaration order
    std::vector<PhysicsCollidePair> physCollidePairs;
    // physicsmarkuplist container scalars - the per-body DEFAULTS a child
    // overrides, so they share the child's spelling. Values match the
    // reference CJointedModel constructor.
    float physMass = 1.0f;
    bool physAutoMass = false;
    Vector3 physMassCenter{};
    bool physMassCenterSet = false;
    std::string physRootBone;
    bool physNoSelfCollisions = false;
    // $assumeworldspace - the collision source's verts are already in model
    // space, so skip the remap onto the compiled skeleton's bind pose.
    bool physAssumeWorldspace = false;
    float physDamping = 0.0f;
    float physRotdamping = 0.0f;
    float physInertia = 1.0f;
    float physDrag = -1.0f;
    float physWeldPosition = 0.0f;
    float physWeldNormal = 0.999f;
    std::string physName;
    // physicsmarkuplist/animatedfriction - ragdoll joint friction ramp
    bool physHasAnimatedFriction = false;
    int physAnimFrictionMin = 0;
    int physAnimFrictionMax = 0;
    float physAnimFrictionTimeIn = 0.0f;
    float physAnimFrictionTimeOut = 0.0f;
    float physAnimFrictionTimeHold = 0.0f;

    // gamedatalist - model-level game data
    std::string surfaceprop;
    std::string keyvalues; // $keyvalues, copied to the studiohdr verbatim
    // $contents, resolved to the flag word by the loader. Starts at
    // CONTENTS_SOLID (0x1) like the reference's s_nDefaultContents.
    int contents = 1;
    // $jointcontents: bone name -> flag word, also resolved by the loader
    // (reference s_JointContents). Applied once the bone table is final; a
    // bone with no entry of its own inherits the nearest parent that has one,
    // else the model word.
    std::vector<std::pair<std::string, int>> jointContents;

    // $includemodel, authored in animationlist. Already prefixed "models/".
    std::vector<std::string> includeModels;

    // $cdmaterials / cdmaterialslist. Stored by the front ends but not yet
    // consumed: every DMX texture is relative-path flagged, so SetSkinValues
    // writes the single empty cdtexture entry regardless.
    std::vector<std::string> cdmaterials;

    // $texturegroup - skin families. One entry per $set block, in script order:
    // the first $set is skin family 1, family 0 being the base materials.
    // Each replacement names both materials directly, so a family only lists
    // what it changes.
    struct SkinReplace {
        std::string from; // material to replace, as the model already knows it
        std::string to;   // its replacement in this family
    };
    std::vector<std::vector<SkinReplace>> skinFamilies;

    std::vector<std::unique_ptr<source::Source>> sources; // all loaded DMX
    source::MaterialTable mats;

    struct InModel {
        std::string name;     // choice name or "blank"
        source::Source* source = nullptr; // null = blank
    };
    struct InBodyPart {
        std::string name;
        std::vector<InModel> models;
    };
    std::vector<InBodyPart> bodyparts;

    // $modelgrouppreset, resolved to value/mask in Compile once the bases exist
    struct InBodyGroupPreset {
        std::string name;
        std::vector<std::pair<std::string, int>> choices; // $modelgroup name -> index
    };
    std::vector<InBodyGroupPreset> bodygrouppresets;

    // $lod / $shadowlod. Compile() prepends the implicit root LOD, so what the
    // front end stores here is LOD 1 and up, in script order.
    std::vector<ScriptLod> scriptLods;

    struct InIkRule {
        std::string chain;
        std::string type;      // footstep|touch|attachment|release
        std::string touchBone; // IK_SELF target ("" = worldspace)
        std::string attachment;
        float height = 0, floor = 0, radius = 0;
        bool heightSet = false, floorSet = false, radiusSet = false;
        int contact = -1;
        int startframe = -1, peakframe = -1, tailframe = -1, endframe = -1;
        bool usesequence = false, usesource = false;
        Vector3 fakeorigin;
        Vector3 fakerotate; // degrees
        bool fakeoriginSet = false, fakerotateSet = false;
    };
    struct InAnim {
        std::string name; // "@<seq>" when implied
        source::Source* source = nullptr;
        float fps = 30.0f;
        int flags = 0; // STUDIO_DELTA etc.
        bool isDeclare = false;      // $declareanimation slot (STUDIO_OVERRIDE)
        // $bindposeanimation / $bindposesequence: no source file - the clip is
        // the compile's own bind pose, held for the whole frame range.
        bool bindpose = false;
        // ordered command list, script order (studiomdl s_animation_t::cmds).
        // Weights/Subtract carry names, resolved in the compile stage.
        struct InCmd {
            enum Kind { Weights, Subtract, Reverse, FixupLoop, Angle, Align, Match,
                MatchBlend, WorldspaceBlend, AppendAnim, BoneDriver,
                Motion, RefMotion, CopyPose, TransformBone, NumFrames,
                IkFixup } kind = Weights;
            std::string name;   // Weights: weightlist; Subtract/Align/Match: animation
                                // CopyPose: an animation OR a sequence
            int frame = 0;      // Subtract: reference frame
            int numframes = 0;  // NumFrames: the length to clip or pad to
            bool presubtract = false;
            int fixupStart = 0, fixupEnd = 0;
            float angle = 0.0f;
            std::string alignBone; // Align/BoneDriver: "" = the root bone
            int motiontype = 0;    // Align: which components to match
            int srcframe = 0, destframe = 0;
            int driverAxis = 0;    // BoneDriver
            float driverValue = 1.0f;
            bool driverAll = true;
            int driverStart = 0, driverPeak = 0, driverTail = 0, driverEnd = 0;
            int matchPre = 0, matchPost = 0; // MatchBlend
            int motionEndFrame = 0;          // Motion/RefMotion
            bool worldLoops = false;         // WorldspaceBlend
            // CopyPose: srcframe of `name` held across destframe..copyEndFrame,
            // faded over fadeIn/fadeOut frames either side. copyBindPose: the
            // source was the reserved name `bindpose`, so `name` is unused.
            bool copyBindPose = false;
            int copyEndFrame = 0;
            float fadeIn = 0.0f, fadeOut = 0.0f;
            bool copyPos = true, copyRot = true;
            AnimBoneTransform xform; // TransformBone
            InIkRule ikfixup;        // IkFixup
        };
        std::vector<InCmd> cmds;
        // clip trim (`frame <a> <b>` / `framestart <a>`). endframe -1 = run to
        // the source clip's end; both are clamped to the clip either way.
        int startframe = 0;
        int endframe = -1;
        // `blockname <name>`: which clip inside the source to sample. "" picks
        // the first non-BindPose clip, the way an untagged animation does.
        std::string blockname;
        // per-animation transform, overriding the model-wide one. Consumed by
        // ConvertAnimation exactly like the model defaults are.
        Vector3 adjust;               // `origin x y z`
        bool adjustSet = false;
        RadianEuler rotation;         // `rotate <deg>` / `angles p y r`
        bool rotationSet = false;
        float scale = 1.0f;           // `scale <f>`
        bool ignorescale = false;     // `ignorescale`
        // loop fixups. fudgeloop appends the missing duplicate end frame;
        // looprestart/looprestartpercent rotate the clip so it loops elsewhere.
        bool fudgeloop = false;
        bool noAutoIK = false; // `noautoik`: skip the auto IK_RELEASE pass
        // `ikrule` is an animation option, not a sequence one - a sequence body
        // routes it to blend anim 0 like every other animation token.
        std::vector<InIkRule> ikrules;
        bool nocull = false;   // `nocull`: exempt from $animationcullmethod
        // `ignoretransformbone <angles|position>`, see Anim
        bool ignoreTransformAngles = false;
        bool ignoreTransformPosition = false;
        int motiontype = 0;    // motion controls (LX/LY/LZ/LXR/.../LM/LQ)
        float motionrollback = 0.3f; // `motionrollback`, see Anim
        int looprestart = 0;
        float looprestartpercent = 0.0f;
        // demand loading ($animblocksize). `noanimblock` keeps the whole clip
        // in the .mdl; `noanimblockstall` keeps its first numNostallFrames
        // frames there so playback can start before the .ani block arrives.
        bool disableAnimblocks = false;
        bool isFirstSectionLocal = false;
        float numNostallFrames = 0.0f; // 0 = fps * preloadTime
    };
    struct InBlendParam {
        std::string parameter;
        float min = 0, max = 0;
        bool calc = false; // calcblend: min/max are measured, not authored
    };
    struct InAutoLayer {
        std::string sequence;
        int flags = 0; // STUDIO_AL_*
        std::string poseparameter; // when STUDIO_AL_POSE
        float start = 0, peak = 0, tail = 0, end = 0; // frames
    };
    struct InSequence {
        std::string name;
        int animIndex = -1;            // into inAnims (single-anim case)
        std::vector<int> blendAnims;   // multi-anim grid (overrides animIndex)
        int blendwidth = 0;
        int flags = 0;
        bool isDeclare = false;        // $declaresequence slot
        std::string activityname;
        int actweight = 0;
        float fadeintime = 0.2f;
        float fadeouttime = 0.2f;
        std::string weightlist;        // applied to the sequence's animations
        InBlendParam blendparams[2];
        int numblendparams = 0;
        std::vector<InAutoLayer> autolayers;
        std::vector<SeqEvent> events;
        std::vector<SeqAnimTag> animtags;
        std::vector<std::string> activitymodifiers;
        std::string keyvalues;
        int entrynode = 0;
        int exitnode = 0;
        int nodeflags = 0;
        float exitphase = 0.0f;
        std::string posecycle;      // `posecycle <param>`, "" = none
        std::string rootdriverBone; // `rootdriver <bone>`, "" = none
        // calcblend: measure an attachment's movement across the blend grid to
        // derive the pose parameter range, instead of `blend`'s authored one.
        std::string paramattachment[2]; // "" = this axis uses `blend`
        int paramcontrol[2] = {0, 0};   // STUDIO_X/Y/Z/XR/YR/ZR
        std::string paramanim;      // blendref: what "zero" looks like
        std::string paramcompanim;  // blendcomp: base for delta grid anims
        std::string paramcenter;    // blendcenter: the grid's neutral cell
        std::vector<IkLock> iklocks;   // chain by name, resolved in LinkIKLocks
    };
    // $sectionframes: animation sectioning thresholds. An animation of at least
    // minSectionFrameLimit frames is split into sectionFrames-long sections.
    int sectionFrames = 30;
    int minSectionFrameLimit = 30;

    // --- $animblocksize: demand-loaded animation (the external .ani) ---------
    // Non-zero moves animation payload out of the .mdl into models/<name>.ani,
    // cut into blocks of roughly this many bytes. Turning it on also switches
    // every animation to the frame-major STUDIO_FRAMEANIM encoding (unless
    // `lowres`) and makes the writer emit per-animation zero frames.
    int animblocksize = 0;
    bool noAnimblockStall = false; // `nostall`
    bool animblockHighRes = false; // `highres`: full float positions
    bool animblockLowRes = false;  // `lowres`: keep the RLE encoding
    int maxZeroFrames = 3;         // `numframes <1..4>`
    bool zeroFramesHighres = false;   // `cachehighres`: Quaternion64 zero frames
    float minZeroFramePosDelta = 2.0f; // `posdelta <f>`
    float preloadTime = 1.0f;          // seconds of nostall frames, when unstated

    // $bonesaveframe: which bones get a zero-frame cache entry. Empty means the
    // writer picks automatically (roots + bones that travel far enough).
    std::vector<BoneSaveFrame> boneSaveFrames;

    // transition graph. Node names are in declaration order, so a sequence's
    // 1-based entrynode/exitnode index this list + 1 (LookupXNode).
    std::vector<std::string> xnodes;
    std::vector<std::pair<int, int>> xnodeskips; // $skiptransition, 1-based ids
    bool multistageGraph = false;                // $calctransitions

    std::vector<InAnim> anims;
    std::vector<InSequence> sequences;

    std::string mdlPath; // full output path, for cdtexture derivation parity
};

// The studio name a bodygroup choice gets when the script does not spell one
// out: the render-mesh names it draws, lowercased, joined with '_' (anything
// outside [a-z0-9_-] becomes '_' too). This is what shows up in SFM's model
// list, so both front ends derive it the same way.
std::string ChoiceName(const std::vector<std::string>& meshRefs);

// Run the pipeline. Returns false + err on hard errors (limits, bad references).
// `input` is mutated: with no sequences and no $includemodel a "reference"
// bind-pose sequence is appended, since Source needs one or the other to load.
bool Compile(CompileInput& input, CompiledModel& out, std::string* err);

} // namespace pulse::compile

#endif // PULSEMDL_COMPILE_H
