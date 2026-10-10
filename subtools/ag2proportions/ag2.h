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
inline constexpr const char* kStockViewSkeleton = "animation/skeletons/characters/viewmodel.vnmskel";

// Generated assets, relative to the output folder.
inline constexpr const char* kWorldGraphFile = "graphs/proportions_worldmodel.vnmgraph";
inline constexpr const char* kUiGraphFile = "graphs/proportions_uimodel.vnmgraph";
inline constexpr const char* kClipFile = "anims/proportions.vnmclip";
inline constexpr const char* kProportionsFile = "anims/proportions.dmx";
inline constexpr const char* kReferenceFile = "anims/reference.dmx";
inline constexpr const char* kSkeletonFile = "dmx/ag2_skeleton.dmx";

// One model to wrap. Paths are absolute once loaded; empty means "use the default".
struct Job {
    fs::path vmdl;
    fs::path modelDmx;    // default: the body RenderMeshFile import
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
// The model's complete skeleton, parent-first in local space.
struct SkeletonBuild {
    std::vector<Bone> bones;
    std::vector<std::string> added;      // stock bones the rig lacked
    std::vector<std::string> reparented; // "bone -> stock parent"
    std::vector<std::string> fromNodes;  // non-stock VMDL Bone nodes carried over
};

size_t CountBones(pulse::dmx::Datamodel& dm); // CS2 core bones in the rig, 0 if none
// A DMX's joints parent-first in local space, scaled; `held` takes a pose's first keys.
std::vector<Bone> ReadRig(pulse::dmx::Datamodel& dm, float scale, bool held);
// Stock bones get stock parents at their rig model-space pose; missing ones take
// their stock local offset. Other rig bones keep their parents.
// True when every bone's distance to its parent matches each stock skeleton it
// shares with the rig, whatever the posture; needs at least three such pairs.
bool HasStockLengths(const std::vector<Bone>& rig, const std::vector<const std::vector<Bone>*>& skeletons);
SkeletonBuild BuildSkeleton(const std::vector<Bone>& rig, const std::vector<Bone>& stock,
                            const std::vector<Bone>& nodes);
std::vector<Bone> ModelStock(const std::vector<Bone>& stock); // stock plus the agent-only bones
std::vector<Bone> TargetPose(const std::vector<Bone>& skeleton, const std::vector<Bone>& stock,
                             std::vector<std::string>& warnings);
void WriteSkeleton(const fs::path& path, const std::vector<Bone>& bones);
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
