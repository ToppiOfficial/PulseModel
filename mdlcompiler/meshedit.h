// meshedit.h - PulseMDL
//
// The `$rendermesh <name> <file> { ... }` edit options: what a modelling tool
// would have done to the file before exporting it, done at load time instead.
//
// Edits are scoped to one $rendermesh. A $rendermesh with a body loads its own
// private Source, so an edit never reaches another reference to the same file -
// the compile is unaffected outside this render mesh.

#ifndef PULSEMDL_MESHEDIT_H
#define PULSEMDL_MESHEDIT_H

#include <string>
#include <vector>

#include "source.h"

namespace pulse::source {

// $exceptionlist <exclusive|inclusive> { <name> ... } - a DmeMesh name filter
// applied as the DMX loads, so an excluded mesh contributes no geometry, no
// materials and no delta shapes. `inclusive` (the default) keeps only what is
// listed; `exclusive` drops it. Names are case-insensitive and match the
// DmeMesh's own name or its dag's.
struct MeshFilter {
    struct Entry {
        std::string name;
        bool matched = false; // a name that matches nothing is a hard error
    };
    std::vector<Entry> names;
    bool exclusive = false;
    // $removemeshword: drop every face whose material name contains one of these
    // (case-insensitive substring). Repeatable and independent of $exceptionlist.
    std::vector<std::string> removeWords;
    // $removemesh: drop every face whose material name equals one of these
    // (case-insensitive, path and extension ignored). Unmatched is a hard error.
    struct RemoveEntry {
        std::string name;
        mutable bool matched = false;
    };
    std::vector<RemoveEntry> removeNames;

    bool empty() const { return names.empty(); } // $exceptionlist only

    // Filter one mesh dag, recording which entries hit.
    bool Keep(const std::string& meshName, const std::string& dagName);

    // First entry that never matched a mesh, or null.
    const std::string* Unmatched() const;

    // True if the material is named by $removemesh or contains a $removemeshword keyword.
    bool MaterialRemoved(const std::string& materialName) const;

    // $decimatemesh: mesh names -> face tag (1-based per command, 0 = untagged).
    struct TagEntry {
        std::string name;
        int tag = 0;
        bool matched = false;
    };
    std::vector<TagEntry> tags;
    std::vector<TagEntry> inflateTags; // $inflate mesh: same shape, packed into the high byte
    int Tag(const std::string& meshName, const std::string& dagName);

    struct EditStream {
        std::string name;
        bool found = false;
    };
    std::vector<EditStream> editStreams;
    std::string editStreamError;
    bool keepMeshIdentity = false, selectMeshStreams = false;
};

enum class SelectionCombine { Add, Subtract, Intersect };
enum class MeshOperation {
    SelectStream, SelectMaterial, SelectMesh, SelectMeshByStream,
    SelectWeights, SetWeight, ShiftWeights,
    SelectAll, SelectNone, InvertSelection, DeleteFaces,
    DeleteVertices, Duplicate, ReplaceMaterial, FlipNormals, Weld, Inflate
};

struct EditMeshOperation {
    MeshOperation kind = MeshOperation::SelectAll;
    SelectionCombine combine = SelectionCombine::Add;
    std::string name;
    std::string residualBone;
    Vector3 offset{}, angles{};
    std::vector<std::string> materials;
    float value = 0.5f, thicknessClamp = 0.0f;
    bool replaceSelection = false, hull = true, sharpEdges = false;
    bool selectChildren = false;
    bool worldAxes = false;
    int line = 0;
};

bool ApplyEditMesh(Source& src, MaterialTable& mats,
                   const std::vector<EditMeshOperation>& operations,
                   std::string* err, int* errorLine);

// $wrinklescale <morph> <scale> - what dmxedit's SetWrinkleScale would have
// baked into the DMX, minus the controller argument: wrinkle bakes per morph,
// so the combination control a scale hangs off never reaches the output.
struct WrinkleScaleOption {
    std::string shape;
    float scale = 0.0f;
    int line = 0;
    bool matched = false; // a name no source has is a hard error
};

// Bake wrinkle onto every matching morph, per vertex: |posDelta| * scale /
// maxDeflection (reference GenerateWrinkleDelta). Runs after every source is
// loaded, so it overwrites whatever the file itself carried or generated.
void ApplyWrinkleScales(Source& src, std::vector<WrinkleScaleOption>& opts);

// $skinnedbonecull <aggressive|tree> - drop every bone no vertex of this source
// weights, on top of (and before) the compile stage's own collapse pass. Tree
// keeps the parent chain of a skinned bone, so the hierarchy above it survives;
// Aggressive keeps only the skinned bones themselves and folds each dropped
// parent's transform into what is left below it.
enum class SkinnedBoneCull { None, Tree, Aggressive };

void CullUnskinnedBones(Source& src, SkinnedBoneCull mode);

// Merge selected vertices across UV seams, within a material and identical
// bone weights. sharpEdges preserves normal splits at each merged centroid.
void WeldVertices(Source& src, std::vector<char>& selection,
                  float distance = 0.0001f, bool sharpEdges = false);

// Offset vertices and morph targets along their unit normals, in model units.
// `meshTag` > 0 limits it to faces with that $inflate mesh group, `materials` to
// vertices of those materials (path and extension ignored); neither = all.
// False if the selection matched no vertex.
bool InflateVertices(Source& src, float amount, int meshTag = 0,
                     const std::vector<std::string>& materials = {},
                     const MaterialTable* mats = nullptr);

// $scalebone: scale a bone's subtree in its bind-space axes, deforming skinned
// vertices by their subtree weight and moving descendant bone origins. False if no such bone.
bool ScaleBone(Source& src, const std::string& bone, const Vector3& scale);

// Reverse normals, morph normal deltas and triangle winding.
void FlipNormals(Source& src);

// meshoptimizer decimation to `factor` (0,1] of each mesh's index count.
// dst may alias src - faces are rebuilt only after every mesh is simplified.
// The vertex pool is untouched, so the result stays rigged. `skipMaterial`, if
// given, is indexed by material ID and keeps those meshes at full detail.
// Faces simplify per Source::faceTag group; `onlyTag` >= 0 limits it to that group.
// `rigAware` adds skin centroids as an attribute so bone boundaries resist collapse.
void SimplifyFaces(Source& dst, const Source& src, float factor, bool lockBorder,
                   const std::vector<bool>* skipMaterial = nullptr, int onlyTag = -1,
                   bool rigAware = false);

// Drop disconnected face islands smaller than limit on every axis.
void RemoveSmallMeshes(Source& src, float limit);

// Fuse several loaded render meshes into ONE drawable Source - what exporting
// them together out of a single scene would have produced. Bones unify by name
// (the first part that names a bone sets its bind pose), vertices and faces are
// concatenated and re-grouped by material, and delta shapes sharing a name fuse
// into one morph so a flex authored on both halves drives both.
//
// Each part keeps the tangents it was loaded with: they were computed against
// its own faces, and being drawn beside another mesh does not change them.
//
// Two parts rigged against DIFFERENT bind transforms for the same bone name
// cannot both be right - vertices are baked into bind space at load - so the
// first one wins and the other deforms. Merge meshes off one rig.
bool MergeSources(const std::vector<Source*>& parts, Source& out, std::string* err);

} // namespace pulse::source

#endif // PULSEMDL_MESHEDIT_H
