// dmxwrite.h - rebuilds the render meshes the .mdl/.vvd/.vtx were compiled from.

#ifndef MDLDECOMPILE_DMXWRITE_H
#define MDLDECOMPILE_DMXWRITE_H

#include <string>
#include <vector>

#include "mdlfile.h"

namespace mdldecompile {

// -dmxencoding ("binary" | "keyvalues2") and -dmxmodel (the `format model`
// version). Returns an error message when either is not one we write, else
// nullptr. Defaults are binary + model 15, which is binary encoding version 4;
// model 1 is binary encoding version 2, model 18 version 5, model 22 (modeldoc)
// version 9.
const char* SetDmxOutput(const std::string& encoding, int formatModel);

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

// One bone's parent-relative pose in one frame, as animwrite.cpp decodes it.
struct AnimPose {
    pm::Vector3 pos{};
    pm::RadianEuler rot{};
};

// `path`: the skeleton plus one DmeChannelsClip holding `frames` at `fps`. The
// joints are written at frame 0, the way an SMD's skeleton block doubles as its
// bind pose, and every bone gets a channel so no bone falls back on that pose.
bool WriteAnimationDmx(const Mdl& m, const std::string& path, const std::string& clipName, int fps,
                       const std::vector<std::vector<AnimPose>>& frames);

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
