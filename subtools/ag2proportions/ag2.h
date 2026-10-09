// ag2.h - ag2proportions: a CS2 AnimGraph2 wrapper that applies a model's
// bone proportions on top of Valve's worldmodel graph.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "dmx/dmx.h"
#include "keyvalues3/kv3.h"
#include "math/math.h"

namespace ag2 {

namespace fs = std::filesystem;
namespace kv = pulse::keyvalues3;
namespace pm = pulse::math;

inline constexpr const char* kStockGraph = "animation/graphs/worldmodel/worldmodel.vnmgraph";
inline constexpr const char* kStockUiGraph = "animation/graphs/ui/uimodel.vnmgraph";
inline constexpr const char* kStockSkeleton = "animation/skeletons/characters/worldmodel.vnmskel";

// Generated assets, relative to the output folder.
inline constexpr const char* kWorldGraphFile = "graphs/proportions_worldmodel.vnmgraph";
inline constexpr const char* kUiGraphFile = "graphs/proportions_uimodel.vnmgraph";
inline constexpr const char* kClipFile = "anims/proportions.vnmclip";

// One model to wrap. Paths are absolute once loaded; empty means "use the default".
struct Job {
    fs::path vmdl;
    fs::path modelDmx;    // default: the VMDL's SkeletonFile import
    fs::path proportions; // default: the model DMX's bind pose
    fs::path vnmskel;     // default: read from the installed pak01 VPK
    bool writeModel = false;
};

struct Options {
    std::string compilerRunner; // "" or "wine"
    bool generateOnly = false;
    bool debug = false; // also write debug/ (rig, descriptor, report, notes)
};

// Where a job's files live. `relative` is the output folder as a resource path.
struct Paths {
    fs::path cs2, addon, output;
    std::string relative;
};

struct Bone {
    std::string name;
    int parent = -1;
    pm::Vector3 position;
    pm::Quaternion rotation;
};

// The installed worldmodel skeleton, in DMX axes.
struct StockSkeleton {
    std::vector<Bone> bones;
    int lowLodCount = 0; // bones [0, lowLodCount) are sampled at low LOD
    kv::Value data;      // the compiled resource, for the source descriptor
};

// rig.cpp
// A Bone node that changed a rig bone it replaced.
struct NodeOverride {
    std::string name;
    float moved = 0;  // summed position change
    bool reparented = false;
};
// Model weapon helpers (wpnPivot, wpn) a rig lacks: `inRig` were added to the DMX,
// `asNodes` have a parent that only exists as a VMDL Bone node.
struct HelperPlan {
    std::vector<Bone> inRig, asNodes;
};

// A SkeletonFile (or the body render mesh) in VMDL order.
struct RigFile {
    pulse::dmx::Datamodel* dm;
    bool overwrite; // merge_behavior "overwrite_existing"
};

// ModelDoc's skeleton from the enabled Bone nodes and files; `overrides` reports
// Bone nodes that a file disagrees with.
std::unique_ptr<pulse::dmx::Datamodel> BuildRig(const std::vector<Bone>& nodes, const std::vector<RigFile>& files,
                                                std::vector<NodeOverride>* overrides = nullptr);
std::vector<std::string> BoneNames(pulse::dmx::Datamodel& dm);
bool IsCoreBone(const std::string& name);
std::string BoneParent(pulse::dmx::Datamodel& dm, const std::string& name); // "" for a root or missing bone
size_t CountBones(pulse::dmx::Datamodel& dm); // CS2 core bones in the rig, 0 if none
std::vector<Bone> ModelBoneBindings(pulse::dmx::Datamodel& dm, const std::vector<Bone>& stock);
// `modelRig`: the VMDL's merged rig (needs root_motion), not a proportions DMX.
std::vector<Bone> TargetPose(pulse::dmx::Datamodel& pose, const std::vector<Bone>& stock,
                             std::vector<std::string>& warnings, bool modelRig);
// Helpers named in `present` are skipped; one whose parent is only in `nodeNames`
// (or another node helper) goes to asNodes.
HelperPlan AddWeaponHelpers(pulse::dmx::Datamodel& model, const std::vector<Bone>& stock,
                            const std::vector<std::string>& present, const std::vector<std::string>& nodeNames);
void WriteHeldPose(const fs::path& path, const std::vector<Bone>& bones, int lowLodCount);

// graph.cpp
StockSkeleton ReadStockSkeleton(const std::vector<uint8_t>& resource);
void RequireSameSkeleton(const StockSkeleton& installed, const StockSkeleton& supplied);
kv::Value SkeletonDescriptor(const StockSkeleton& stock, const std::string& referenceDmx);
kv::Value ProportionClip(const std::string& relative);
kv::Value WrapperGraph(const kv::Value& stockGraph, const std::string& stockPath, const std::string& clip);

// workflow.cpp
void Generate(const Job& job, const Options& options);

// compiler.cpp
void Compile(const Options& options, const Paths& paths, const std::vector<uint8_t>& skeleton,
             const std::vector<uint8_t>& graph, const std::vector<uint8_t>& uiGraph, const kv::Value& descriptor);

} // namespace ag2
