// source.h - PulseMDL
//
// The compiler's per-file "source": a structural mirror of the reference
// s_source_t (studied for behavior, re-declared
// clean). One Source corresponds to one loaded DMX. The compile stage
// (compile.cpp) walks these in the same order SimplifyModel walks s_source_t;
// the writers depend on that order, so this stays close to the original
// layout rather than a "clean" redesign.
//
// Key structural facts mirrored from the reference load path:
// - localBone[] is a DFS over ALL of the skeleton root's child dags (including
//   mesh dags), plus a trailing weightless "defaultRoot"; both are flagged
//   isNonSkeletal.
// - Vertices are unified per (position, material, normal, texcoord) tuple in
//   first-seen order, then sorted by material with C qsort (unstable sort -
//   same comparator, same libc = same order).
// - faces are material-sorted; per-face indices are MESH-relative.
// - anims[] holds "BindPose" (frame 0 = bind transform list) plus one entry
//   per DmeChannelsClip, sampled per frame. Rotations are RadianEuler.

#ifndef PULSEMDL_SOURCE_H
#define PULSEMDL_SOURCE_H

#include <cstdint>
#include <string>
#include <vector>

#include "math/math.h"
#include "pulselimits.h"

namespace pulse::source {

using math::Vector2;
using math::Vector3;
using math::Vector4;
using math::RadianEuler;
using math::matrix3x4;

// Source-stage skinning: up to kMaxSrcBoneWeights influences (collapsed to
// kMaxBoneWeights after global bone collapse, in BalanceGlobalBoneWeights).
struct SrcBoneWeight {
    int numbones = 0;
    int bone[pulse::limits::kMaxSrcBoneWeights] = {};
    float weight[pulse::limits::kMaxSrcBoneWeights] = {};
};

// One unified vertex (reference s_vertexinfo_t, single UV channel).
struct SrcVertex {
    Vector3 position; // bind space, pre-scaled
    Vector3 normal;
    Vector2 texcoord;
    Vector4 tangentS; // built by CalcModelTangentSpaces; w = +/-1
    SrcBoneWeight boneweight;
    int material = 0; // GLOBAL material index (MaterialTable)
};

// Triangle; indices are relative to the owning mesh's vertexoffset.
struct SrcFace {
    uint32_t a = 0, b = 0, c = 0;
};

// Per-material vertex/face range (reference s_mesh_t); indexed by material.
struct SrcMesh {
    int numvertices = 0;
    int vertexoffset = 0;
    int numfaces = 0;
    int faceoffset = 0;
};

struct LocalBone {
    std::string name;
    int parent = -1;
    bool isNonSkeletal = false; // mesh dag or the synthetic defaultRoot (exporter noise)
};

struct SrcBonePose {
    Vector3 pos;     // pre-scaled
    RadianEuler rot; // MatrixAngles of the dag transform
};

struct SourceAnim {
    std::string name; // "BindPose" or the DmeChannelsClip name
    int startframe = 0;
    int endframe = 0;
    int numframes = 1;
    // frames[frame][localbone]
    std::vector<std::vector<SrcBonePose>> frames;
};

// ---- flex / morph (loaded only for rendermesh sources) ----------------------
// Reference: delta states -> BuildVertexAnimations creates one
// 1-frame "newStyleVertexAnimations" s_sourceanim_t per delta state. We keep
// them in a separate array (morphs) instead of anims so bone-anim name lookups
// can never hit a vanim-only entry.

// one vertex-morph delta (reference s_vertanim_t)
struct SrcVertAnim {
    int vertex = 0;   // unified vertex index (post material sort, model-relative)
    float speed = 0;  // per-vertex morph speed (FIELD_MORPH_SPEED, default 1)
    float side = 0;   // stereo balance (FIELD_BALANCE, default 1)
    Vector3 pos;      // position delta, bind-rotated + scaled
    Vector3 normal;   // normal delta, bind-rotated
    float wrinkle = 0;
};

// one delta state (morph target) of the source
struct SrcMorphAnim {
    std::string name;
    std::vector<SrcVertAnim> vanims;
};

// flexkey signal captured at load (reference AddFlexKey: s_flexkey_t subset)
struct SrcFlexKey {
    std::string name;   // delta state name
    bool stereo = false; // IsDeltaStateStereo -> flexpair split signal
    float target1 = 1.0f; // s_flexkey_t.target1 (the VTA `position` option)
};

// combination-operator capture (reference s_combinationcontrol_t /
// s_combinationrule_t / s_flexcontrollerremap_t / s_dmeflexrule_t)
struct CombinationControl {
    std::string name; // raw control name
};

struct CombinationRule {
    int flex = -1;                        // index into Source::flexkeys
    std::vector<int> combination;         // raw control indices
    std::vector<std::vector<int>> dominators; // each entry: dominant raw indices
};

// mirrors FlexControllerRemapType_t values (written to disk as remaptype)
enum class RemapType { PassThru = 0, TwoWay = 1, NWay = 2, Eyelid = 3 };

struct ControllerRemap {
    std::string name;      // input control name
    std::string flexgroup; // "" = none ("default" written by DMX exporters)
    bool stereo = false;
    bool eyelid = false;
    bool hasMinMax = false; // control element had flexMin/flexMax
    float min = 0.0f;
    float max = 1.0f;
    RemapType type = RemapType::PassThru;
    std::vector<std::string> rawControls;
    std::string eyesUpDownFlexName; // EYELID only ("eyes_updown" default)
    // resolved during loader flex registration (AddFlexControllers port)
    int index = -1;      // non-stereo controller
    int leftIndex = -1;  // stereo pair
    int rightIndex = -1;
    int multiIndex = -1; // NWAY/EYELID value controller
    int blinkController = -1;
    int eyesUpDownFlexController = -1;
};

// captured DmeFlexRules entry (expression/passthrough/localvar), DMX order
struct SrcFlexRule {
    std::string name;  // flexdesc name
    std::string expr;  // infix expression ("" for localvar; name for passthrough)
    bool isLocalVar = false;
};

// A whole flex rig lifted out of one DMX's combination operator, detached from
// the geometry it was authored against ($datamodelflexes). Delta states stay
// NAMES here: the rig is stamped onto whichever render mesh carries a matching
// morph, which need not be the file it came from. Loading a render mesh never
// fills this - a mesh contributes morphs and flexkeys only.
struct FlexRig {
    struct Corrective {                           // one combination rule
        std::string delta;                        // delta-state / flexkey name
        std::vector<int> combination;             // indices into `controls`
        std::vector<std::vector<int>> dominators; // each entry: dominant indices
    };
    std::vector<CombinationControl> controls; // raw controls, flattened
    std::vector<ControllerRemap> remaps;      // one per input control
    std::vector<Corrective> correctives;
    std::vector<SrcFlexRule> rules;           // DmeFlexRules capture, DMX order
    bool hasRules = false;                    // a DmeFlexRules target existed

    bool empty() const {
        return controls.empty() && remaps.empty() && correctives.empty() && rules.empty();
    }
};

// Global texture/material registry shared by every loaded source (reference
// g_texture/g_material + LookupTexture/UseTextureAsMaterial). Load order
// defines the .mdl texture order.
struct MaterialTable {
    struct Texture {
        std::string name;
        int material = -1; // -1 until used by a face
        // reference RELATIVE_TEXTURE_PATH_SPECIFIED: DMX textures carry a full
        // relative path; SMD v1/v3 textures are bare names flagged non-relative
        // (SetSkinValues derives a cdmaterials path when NO relative texture
        // exists - see compile stage). DMX callers keep the default.
        bool relative = true;
    };
    std::vector<Texture> textures;
    std::vector<int> materialToTexture;

    // find-or-add. `relative` mirrors the reference LookupTexture flag; a query
    // matches an entry of the other flag class only on its base name.
    int LookupTexture(const char* name, bool relative = true);
    int UseTextureAsMaterial(int textureIdx); // find-or-assign material index
};

// What a reference asked the loader for. The front-end source caches key on
// filename alone, so this is what decides whether a later reference to the same
// path may REUSE a loaded entry: the values are ordered by completeness, and a
// request may reuse a cached entry whose kind is at least as complete as what it
// needs. Getting this wrong is silent - drawing geometry off a Collision entry
// would resolve material indices against the wrong table.
enum class LoadKind {
    Animation = 0, // $animation / $sequence: skeleton + channel clips, no mesh
    Collision = 1, // a $physicsmodel `file`: geometry, but materials in a
                   // PRIVATE table - collision hulls are never drawn
    Model = 2,     // a render mesh or LOD replacement: geometry + the
                   // compile-wide material table
};

struct Source {
    std::string filename;

    // how this Source was loaded - see LoadKind. Set by the front end, which is
    // the only thing that knows which role the reference was filling.
    LoadKind kind = LoadKind::Model;

    // skeleton
    std::vector<LocalBone> localBone; // includes trailing "defaultRoot"
    int numbones = 0;                 // == localBone.size()
    std::vector<matrix3x4> boneToPose; // Build_Reference("BindPose") result

    // animation ("BindPose" + channel clips)
    std::vector<SourceAnim> anims;

    // geometry (unified + material-sorted)
    std::vector<SrcVertex> vertex;
    // A $modelgroup `mesh a b` fused several $rendermesh sources into this one,
    // interleaved by material - so which part a vertex came from is the only
    // surviving identity. Empty when this Source is not a fusion.
    std::vector<const Source*> mergedParts;
    std::vector<int> vertexPart; // parallel to vertex; index into mergedParts
    std::vector<SrcFace> face;
    // Indexed by material id, so sized to kMaxSkins by whichever loader fills
    // them. Left empty on a Source with no geometry (an animation) - at
    // kMaxSkins that is ~655 KB a source not to allocate.
    std::vector<SrcMesh> mesh;
    std::vector<int> meshindex; // used materials, ascending
    int nummeshes = 0;

    // ---- flex / morph (filled only when loaded as a rendermesh) ----
    std::vector<SrcMorphAnim> morphs;   // per delta state, DMX order
    std::vector<SrcFlexKey> flexkeys;   // AddFlexKeys result
    std::vector<CombinationControl> combinationControls; // raw controls, flattened
    std::vector<CombinationRule> combinationRules;
    std::vector<ControllerRemap> controllerRemaps;
    std::vector<SrcFlexRule> dmeFlexRules; // DmeFlexRules capture, DMX order
    bool hasDmeFlexRules = false; // any DmeFlexRules target existed (even if empty)
    // any FIELD_BALANCE value != 1 in the mesh. Whether a stereo split is real
    // is only decidable once the rig is known, so the check lives in flexreg.
    bool hasBalanceData = false;
    // filled by the loader's flex registration (AddFlexControllers /
    // AddBodyFlexData ports in pulseloader.cpp)
    std::vector<int> rawToRemapSource; // per raw control -> remap index
    std::vector<int> rawToRemapLocal;  // per raw control -> index within remap
    std::vector<int> leftRemapToGlobal;  // per remap -> global controller index
    std::vector<int> rightRemapToGlobal;
    int keyStartIndex = 0;             // first global flexkey of this source
    bool combinationRegistered = false; // AddCombination equivalent ran once

    // ---- filled by the compile stage (compile.cpp) ----
    bool isActiveModel = false; // referenced by a bodygroup (has render mesh)
    std::vector<int> boneflags; // per local bone
    std::vector<int> boneref;   // per local bone (flags unioned up the parents)
    std::vector<int> boneLocalToGlobal;
    std::vector<int> boneGlobalToLocal; // sized by global bone count
    std::vector<SrcVertex> globalVertices; // RemapVerticesToGlobalBones output
};

SourceAnim* FindSourceAnim(Source& src, const char* name);

// reference CalcModelTangentSpaces: per-mesh tangent basis from
// face UV gradients. Shared by the DMX and SMD mesh loaders.
void CalcModelTangentSpaces(Source& src);

// reference SortAndBalanceBones (dup-collapse, weight-desc bubble sort, drop
// tiny weights, clip to iMaxCount, renormalize). Shared by loader + compile.
int SortAndBalanceBones(int iCount, int iMaxCount, int bones[], float weights[]);

} // namespace pulse::source

#endif // PULSEMDL_SOURCE_H
