// meshedit.h - PulseMDL
//
// The `$rendermesh <name> <file> { ... }` edit options: what a modelling tool
// would have done to the file before exporting it, done at load time instead.
//
// Both are scoped to ONE $rendermesh. A $rendermesh with a body loads its own
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

    bool empty() const { return names.empty(); } // $exceptionlist only

    // Filter one mesh dag, recording which entries hit.
    bool Keep(const std::string& meshName, const std::string& dagName);

    // First entry that never matched a mesh, or null.
    const std::string* Unmatched() const;

    // True if the material name contains any $removemeshword keyword.
    bool MaterialRemoved(const std::string& materialName) const;
};

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

// $weld [seams] - merge vertices that share an exact position, within one
// material and with identical bone weights. Normals are averaged, so hard-edge
// splits collapse. `seams` also merges across differing UVs, which fuses
// texture seams - without it a UV split still keeps its own vertex.
enum class WeldMode { None, KeepSeams, All };

void WeldVertices(Source& src, WeldMode mode);

// Offset vertices and morph targets along their unit normals, in model units.
void InflateVertices(Source& src, float amount);

// Reverse normals, morph normal deltas and triangle winding.
void FlipNormals(Source& src);

// meshoptimizer decimation to `factor` (0,1] of each mesh's index count.
// dst may alias src - faces are rebuilt only after every mesh is simplified.
// The vertex pool is untouched, so the result stays rigged. `skipMaterial`, if
// given, is indexed by material ID and keeps those meshes at full detail.
void SimplifyFaces(Source& dst, const Source& src, float factor, bool lockBorder,
                   const std::vector<bool>* skipMaterial = nullptr);

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
