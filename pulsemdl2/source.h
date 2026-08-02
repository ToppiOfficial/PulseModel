// source.h - one loaded source file (mirrors the reference s_source_t layout).
// The compile stage walks these in load order and the writers depend on that
// order, so the layout stays close to the reference rather than "clean".

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

// Collapsed to kMaxBoneWeights after global bone collapse (BalanceGlobalBoneWeights).
struct SrcBoneWeight {
    int numbones = 0;
    int bone[pulse::limits::kMaxSrcBoneWeights] = {};
    float weight[pulse::limits::kMaxSrcBoneWeights] = {};
};

// One unified vertex, single UV channel.
struct SrcVertex {
    Vector3 position; // bind space, pre-scaled
    Vector3 normal;
    Vector2 texcoord;
    Vector4 tangentS; // built by CalcModelTangentSpaces; w = +/-1
    SrcBoneWeight boneweight;
    int material = 0; // GLOBAL material index (MaterialTable)
};

// Indices are relative to the owning mesh's vertexoffset.
struct SrcFace {
    uint32_t a = 0, b = 0, c = 0;
};

// Per-material vertex/face range, indexed by material.
struct SrcMesh {
    int numvertices = 0;
    int vertexoffset = 0;
    int numfaces = 0;
    int faceoffset = 0;
};

// localBone[] is a DFS over all of the skeleton root's child dags (mesh dags
// included) plus a trailing weightless "defaultRoot"; both are isNonSkeletal.
struct LocalBone {
    std::string name;
    int parent = -1;
    bool isNonSkeletal = false; // mesh dag or the synthetic defaultRoot
};

struct SrcBonePose {
    Vector3 pos;     // pre-scaled
    RadianEuler rot;
};

// "BindPose" (frame 0 = bind transforms) plus one entry per DmeChannelsClip.
struct SourceAnim {
    std::string name;
    int startframe = 0;
    int endframe = 0;
    int numframes = 1;
    std::vector<std::vector<SrcBonePose>> frames; // [frame][localbone]
};

// ---- flex / morph (loaded only for rendermesh sources) ----------------------
// Kept in `morphs` rather than `anims` so bone-anim name lookups can never hit
// a vertex-animation-only entry.

struct SrcVertAnim {
    int vertex = 0;   // unified vertex index (post material sort, model-relative)
    float speed = 0;  // morph speed (FIELD_MORPH_SPEED, default 1)
    float side = 0;   // stereo balance (FIELD_BALANCE, default 1)
    Vector3 pos;      // position delta, bind-rotated + scaled
    Vector3 normal;   // normal delta, bind-rotated
    float wrinkle = 0;
};

// One delta state (morph target).
struct SrcMorphAnim {
    std::string name;
    std::vector<SrcVertAnim> vanims;
};

struct SrcFlexKey {
    std::string name;     // delta state name
    bool stereo = false;  // flexpair split signal
    float target1 = 1.0f;
};

struct CombinationControl {
    std::string name; // raw control name
};

struct CombinationRule {
    int flex = -1;                            // index into Source::flexkeys
    std::vector<int> combination;             // raw control indices
    std::vector<std::vector<int>> dominators; // each entry: dominant raw indices
};

// Mirrors FlexControllerRemapType_t (written to disk as remaptype).
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
    // resolved during loader flex registration
    int index = -1;      // non-stereo controller
    int leftIndex = -1;  // stereo pair
    int rightIndex = -1;
    int multiIndex = -1; // NWAY/EYELID value controller
    int blinkController = -1;
    int eyesUpDownFlexController = -1;
};

// DmeFlexRules entry (expression/passthrough/localvar), DMX order.
struct SrcFlexRule {
    std::string name;  // flexdesc name
    std::string expr;  // infix expression ("" for localvar; name for passthrough)
    bool isLocalVar = false;
};

// A flex rig lifted out of one DMX ($datamodelflexes), detached from the
// geometry it was authored against. Delta states stay NAMES: the rig is stamped
// onto whichever render mesh carries a matching morph, which need not be the
// file it came from. Loading a render mesh never fills this.
struct FlexRig {
    struct Corrective {                           // one combination rule
        std::string delta;                        // delta-state / flexkey name
        std::vector<int> combination;             // indices into `controls`
        std::vector<std::vector<int>> dominators;
    };
    std::vector<CombinationControl> controls; // raw controls, flattened
    std::vector<ControllerRemap> remaps;      // one per input control
    std::vector<Corrective> correctives;
    std::vector<SrcFlexRule> rules;
    bool hasRules = false;                    // a DmeFlexRules target existed

    bool empty() const {
        return controls.empty() && remaps.empty() && correctives.empty() && rules.empty();
    }
};

// Global registry shared by every loaded source; load order defines the .mdl
// texture order.
struct MaterialTable {
    struct Texture {
        std::string name;
        int material = -1; // -1 until used by a face
        // DMX textures carry a relative path; SMD v1/v3 textures are bare names
        // flagged non-relative (SetSkinValues derives a cdmaterials path only
        // when NO relative texture exists). DMX callers keep the default.
        bool relative = true;
    };
    std::vector<Texture> textures;
    std::vector<int> materialToTexture;

    // Find-or-add; a query matches an entry of the other `relative` class only
    // on its base name.
    int LookupTexture(const char* name, bool relative = true);
    int UseTextureAsMaterial(int textureIdx); // find-or-assign material index
};

// Source caches key on filename alone, so this decides whether a later
// reference to the same path may REUSE a loaded entry: values are ordered by
// completeness, and a request may reuse a cached entry at least as complete as
// it needs. Getting this wrong is silent - drawing geometry off a Collision
// entry would resolve material indices against the wrong table.
enum class LoadKind {
    Animation = 0, // $animation / $sequence: skeleton + channel clips, no mesh
    Collision = 1, // a $physicsmodel `file`: geometry, materials in a PRIVATE
                   // table - collision hulls are never drawn
    Model = 2,     // render mesh or LOD replacement: geometry + the
                   // compile-wide material table
};

struct Source {
    std::string filename;
    LoadKind kind = LoadKind::Model; // set by the front end, the only thing
                                     // that knows the reference's role

    // skeleton
    std::vector<LocalBone> localBone; // includes trailing "defaultRoot"
    int numbones = 0;                 // == localBone.size()
    std::vector<matrix3x4> boneToPose; // Build_Reference("BindPose") result

    std::vector<SourceAnim> anims;

    // geometry - unified per (position, material, normal, texcoord) in
    // first-seen order, then material-sorted with C qsort (unstable sort; same
    // comparator + same libc = same order).
    std::vector<SrcVertex> vertex;
    // A $modelgroup `mesh a b` fused several $rendermesh sources into this one,
    // interleaved by material - so which part a vertex came from is the only
    // surviving identity. Empty when this Source is not a fusion.
    std::vector<const Source*> mergedParts;
    std::vector<int> vertexPart; // parallel to vertex; index into mergedParts
    std::vector<SrcFace> face;
    // Indexed by material id, so sized to kMaxSkins by whichever loader fills
    // it. Left empty on a geometry-less Source - at kMaxSkins that is ~655 KB.
    std::vector<SrcMesh> mesh;
    std::vector<int> meshindex; // used materials, ascending
    int nummeshes = 0;

    // ---- flex / morph (filled only when loaded as a rendermesh) ----
    std::vector<SrcMorphAnim> morphs;   // per delta state, DMX order
    std::vector<SrcFlexKey> flexkeys;
    std::vector<CombinationControl> combinationControls; // raw, flattened
    std::vector<CombinationRule> combinationRules;
    std::vector<ControllerRemap> controllerRemaps;
    std::vector<SrcFlexRule> dmeFlexRules;
    bool hasDmeFlexRules = false; // a DmeFlexRules target existed (even if empty)
    // Any FIELD_BALANCE != 1 in the mesh. Whether a stereo split is real is only
    // decidable once the rig is known, so that check lives in flexreg.
    bool hasBalanceData = false;
    // filled by the loader's flex registration
    std::vector<int> rawToRemapSource; // per raw control -> remap index
    std::vector<int> rawToRemapLocal;  // per raw control -> index within remap
    std::vector<int> leftRemapToGlobal;  // per remap -> global controller index
    std::vector<int> rightRemapToGlobal;
    int keyStartIndex = 0;              // first global flexkey of this source
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

// Per-mesh tangent basis from face UV gradients. Shared by DMX + SMD loaders.
void CalcModelTangentSpaces(Source& src);

// Dup-collapse, weight-desc bubble sort, drop tiny weights, clip to iMaxCount,
// renormalize. Shared by loader + compile.
int SortAndBalanceBones(int iCount, int iMaxCount, int bones[], float weights[]);

// One triangle corner handed to BuildUnifiedMeshes.
struct TriCorner {
    Vector3 position; // bind space, pre-scaled
    Vector3 normal;
    Vector2 texcoord; // V already inverted
    SrcBoneWeight boneweight;
    // The caller's own vertex id, so a morph authored against the source's
    // vertices can find the unified verts it became. -1 = unused.
    int srcVertex = -1;
};

struct TriInput {
    int material = 0; // GLOBAL material index
    TriCorner v[3];   // authored winding; the builder emits a, c, b
};

// Unify corners sharing (material, exact pos, exact uv) with a fuzzy normal +
// exact weight match, material-sort, then fill out.vertex/face/mesh/meshindex
// and tangents. Shared by the SMD and FBX loaders; DMX has its own exact unify.
//
// `srcToUnified`, when given, comes back indexed by TriCorner::srcVertex: the
// final unified vertices each source vertex produced. One source vertex fans
// out to several whenever a material, UV or normal seam splits it.
void BuildUnifiedMeshes(const std::vector<TriInput>& tris, Source& out,
                        std::vector<std::vector<int>>* srcToUnified = nullptr);

} // namespace pulse::source

#endif // PULSEMDL_SOURCE_H
