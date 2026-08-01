// dmxwrite.h - rebuilds the render meshes the .mdl/.vvd/.vtx were compiled from.

#ifndef MDLDECOMPILE_DMXWRITE_H
#define MDLDECOMPILE_DMXWRITE_H

#include <string>
#include <vector>

#include "mdlfile.h"

namespace mdldecompile {

// One LOD of the model, as the .vtx describes it. Index 0 is the root LOD that
// no $lod block writes; a negative switch point is the $shadowlod.
struct LodInfo {
    float switchPoint = 0.0f;
    // old material -> replacement, both bare names ($lod replacematerial)
    std::vector<std::pair<std::string, std::string>> materialReplacements;
};

// One <name>.dmx in `dir`/meshes per non-blank model, named by the $rendermesh
// alias WriteBodyParts handed out - names[bodypart][model], "" for a blank body.
// Lower LODs land beside it as <name>_lod<n>.dmx. `mdlPath` locates the sibling
// .vvd/.vtx. Returns one entry per LOD (empty when the meshes could not be
// read); never fails the decompile - the .pulseqc stands on its own.
std::vector<LodInfo> WriteRenderMeshes(const Mdl& m, const std::string& mdlPath,
                                       const std::string& dir,
                                       const std::vector<std::vector<std::string>>& names);

struct PhysicsMeshInfo {
    bool written = false;
    // some body carries more than one hull, so the reimport needs `concave`
    bool concave = false;
};

// `dir`/meshes/<name>.dmx: every convex hull in the sibling .phy, back in model
// space and rigged to the bone it drives, so a perjoint reimport rebuilds the
// same bodies. Never fails the decompile - the .pulseqc stands on its own.
PhysicsMeshInfo WritePhysicsMesh(const Mdl& m, const std::string& mdlPath,
                                 const std::string& dir, const std::string& name);

} // namespace mdldecompile

#endif // MDLDECOMPILE_DMXWRITE_H
