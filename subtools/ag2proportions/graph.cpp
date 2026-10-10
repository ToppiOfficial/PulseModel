// graph.cpp - the AnimGraph2 side: the installed worldmodel skeleton, the
// wrapper graphs, the additive clip and the skeleton source descriptor.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>

#include "ag2.h"
#include "templates.h"

namespace ag2 {
namespace {

using std::runtime_error;

void Walk(kv::Value& value, const std::function<void(kv::Value&)>& visit) {
    if (value.IsObject()) {
        visit(value);
        for (auto& member : value.Members())
            Walk(member.second, visit);
    } else if (value.IsArray()) {
        for (auto& item : value.Items())
            Walk(item, visit);
    }
}

std::string ClassOf(const kv::Value& node) {
    return node.Has("_class") ? node.At("_class").String() : std::string();
}

float Component(const kv::Value& v) { return static_cast<float>(v.Number()); }

// A GUID string derived from `key`, so a rerun writes the same graph.
std::string StableGuid(const std::string& key) {
    uint64_t h[2] = {14695981039346656037ull, 0x9e3779b97f4a7c15ull};
    for (uint64_t& v : h)
        for (unsigned char c : "ag2proportions:" + key)
            v = (v ^ c) * 1099511628211ull;
    char out[40];
    std::snprintf(out, sizeof out, "%08x-%04x-%04x-%04x-%012llx", static_cast<unsigned>(h[0] >> 32),
                  static_cast<unsigned>(h[0] >> 16) & 0xffffu, static_cast<unsigned>(h[0]) & 0xffffu,
                  static_cast<unsigned>(h[1] >> 48), static_cast<unsigned long long>(h[1] & 0xffffffffffffull));
    return out;
}

} // namespace

// NM skeletons are authored Y-up. root_motion takes the inverse of the axis fix
// and its direct children take the fix itself; deeper bones are already local.
StockSkeleton ReadStockSkeleton(const std::vector<uint8_t>& resource) {
    StockSkeleton s;
    s.data = kv::ReadResource(resource);
    const auto& names = s.data.At("m_boneIDs").Items();
    const auto& parents = s.data.At("m_parentIndices").Items();
    const auto& poses = s.data.At("m_parentSpaceReferencePose").Items();
    if (names.empty() || names.size() != parents.size() || names.size() != poses.size())
        throw runtime_error("malformed worldmodel skeleton");

    const pm::Quaternion fix{-0.5f, -0.5f, -0.5f, 0.5f}, unfix{0.5f, 0.5f, 0.5f, 0.5f};
    const pm::matrix3x4 fixMatrix = pm::QuaternionMatrix(fix, pm::Vector3{});
    std::set<std::string> seen;
    for (size_t i = 0; i < names.size(); ++i) {
        Bone bone;
        bone.name = names[i].String();
        bone.parent = static_cast<int>(parents[i].Integer());
        if (!seen.insert(bone.name).second || bone.parent < -1 || bone.parent >= static_cast<int>(i))
            throw runtime_error("invalid worldmodel skeleton hierarchy at " + bone.name);

        // position xyz, an optional uniform scale, then the quaternion
        const auto& p = poses[i].Items();
        if (p.size() != 7 && p.size() != 8)
            throw runtime_error("unsupported skeleton transform layout");
        if (p.size() == 8 && std::abs(p[3].Number() - 1.0) > 1e-5)
            throw runtime_error("scaled skeleton bones are not supported");
        const size_t q = p.size() - 4;
        bone.position = {Component(p[0]), Component(p[1]), Component(p[2])};
        bone.rotation = {Component(p[q]), Component(p[q + 1]), Component(p[q + 2]), Component(p[q + 3])};
        pm::QuaternionNormalize(bone.rotation);

        if (bone.name == "root_motion") {
            pm::QuaternionMult(bone.rotation, unfix, bone.rotation);
        } else if (bone.parent >= 0 && s.bones[bone.parent].name == "root_motion") {
            bone.position = pm::VectorRotate(bone.position, fixMatrix);
            pm::QuaternionMult(fix, bone.rotation, bone.rotation);
        }
        s.bones.push_back(bone);
    }

    s.lowLodCount = static_cast<int>(s.data.At("m_numBonesToSampleAtLowLOD").Integer());
    if (s.lowLodCount < 0 || s.lowLodCount > static_cast<int>(s.bones.size()))
        throw runtime_error("invalid worldmodel skeleton LOD bone count");
    return s;
}

void RequireSameSkeleton(const StockSkeleton& installed, const StockSkeleton& supplied) {
    if (installed.bones.size() != supplied.bones.size())
        throw runtime_error("the supplied vnmskel has a different bone count from the installed worldmodel skeleton");
    for (size_t i = 0; i < installed.bones.size(); ++i) {
        const Bone& a = installed.bones[i];
        const Bone& b = supplied.bones[i];
        const float moved = std::abs(a.position.x - b.position.x) + std::abs(a.position.y - b.position.y) +
                            std::abs(a.position.z - b.position.z);
        const float dot = std::abs(a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
                                   a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w);
        if (a.name != b.name || a.parent != b.parent || moved > 0.001f || dot < 0.99999f)
            throw runtime_error("the supplied vnmskel differs from the installed worldmodel skeleton at " + a.name);
    }
}

// The stock skeleton's source form, rebuilt over the reference pose. Only the
// isolated compiler workspace uses it; the installed skeleton is never replaced.
kv::Value SkeletonDescriptor(const StockSkeleton& stock, const std::string& referenceDmx) {
    kv::Value::Array highLod;
    for (size_t i = static_cast<size_t>(stock.lowLodCount); i < stock.bones.size(); ++i)
        highLod.push_back(stock.bones[i].name);
    return kv::Value::Object{
        {"m_sourceFilename", referenceDmx},
        {"m_rootBoneName", "root_motion"},
        {"m_flGlobalScale", 1.0},
        {"m_bIsAttachableProp", stock.data.At("m_bIsPropSkeleton")},
        {"m_secondarySkeletons", stock.data.At("m_secondarySkeletons")},
        {"m_highLODBones", highLod},
        {"m_boneMaskSetDefinitions", stock.data.At("m_maskDefinitions")},
    };
}

kv::Value ProportionClip(const std::string& relative) {
    return kv::Value::Object{
        {"m_sourceFilename", relative + "/" + kProportionsFile},
        {"m_animationSkeletonName", kStockSkeleton},
        {"m_additiveType", "RelativeToAnimationFrame"},
        {"m_additiveBaseFilename", relative + "/" + kReferenceFile},
        {"m_additiveBaseFrame", "FirstFrame"},
        {"m_nAdditiveBaseFrameIdx", 0},
        {"m_bonesToSampleInModelSpace", kv::Value::Array{}},
        {"m_eventTracks", kv::Value::Array{}},
    };
}

// The template's chain (referenced graph -> additive proportion layer -> SnapWeapon)
// over `stockPath`. The wrapper redeclares the stock graph's control parameters,
// which the referenced graph reads by name; Snap's inputs rewire by name.
kv::Value WrapperGraph(const kv::Value& stockGraph, const std::string& stockPath, const std::string& clip) {
    if (stockGraph.At("m_skeleton").String() != kStockSkeleton)
        throw runtime_error(stockPath + " uses a different skeleton");
    const auto& ids = stockGraph.At("m_controlParameterIDs").Items();
    std::map<int64_t, std::string> nodeClass;
    for (const auto& node : stockGraph.At("m_nodes").Items())
        nodeClass[node.At("m_nNodeIdx").Integer()] = node.At("_class").String();

    kv::Value wrapper = kv::ParseText(kWrapperTemplate);
    kv::Value& root = wrapper["m_pRootGraph"];
    std::map<std::string, kv::Value> prototypes; // doc class -> first template node of it
    std::map<std::string, std::string> templateNames; // template parameter node id -> name
    kv::Value::Array nodes;
    for (kv::Value& node : root["m_nodes"].Items()) {
        const std::string cls = ClassOf(node);
        if (cls.find("ControlParameterNode") == std::string::npos) {
            if (node.Has("m_pDefaultVariationData") && node["m_pDefaultVariationData"].Has("m_clip"))
                node["m_pDefaultVariationData"]["m_clip"] = clip;
            if (cls == "CNmGraphDocReferencedGraphNode")
                node["m_pDefaultVariationData"]["m_variation"] = stockPath;
            nodes.push_back(node);
            continue;
        }
        templateNames[node.At("m_ID").String()] = node.At("m_name").String();
        prototypes.emplace(cls, node);
    }

    // CNmControlParameter<Type>Node::CDefinition -> CNmGraphDoc<Type>ControlParameterNode
    std::map<std::string, std::pair<std::string, std::string>> declared; // name -> node id, pin id
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto it = nodeClass.find(static_cast<int64_t>(i));
        const std::string prefix = "CNmControlParameter", suffix = "Node::CDefinition";
        if (it == nodeClass.end() || it->second.rfind(prefix, 0) != 0 || it->second.size() <= prefix.size() + suffix.size())
            throw runtime_error(stockPath + " has an unreadable control parameter");
        const std::string type = it->second.substr(prefix.size(), it->second.size() - prefix.size() - suffix.size());
        const auto proto = prototypes.find("CNmGraphDoc" + type + "ControlParameterNode");
        if (proto == prototypes.end())
            throw runtime_error(stockPath + " has a " + type + " control parameter the wrapper template cannot declare");
        const std::string name = ids[i].String();
        kv::Value node = proto->second;
        node["m_ID"] = StableGuid("parameter:" + name);
        node["m_name"] = name;
        node["m_position"] = kv::Value::Array{-600.0, 120.0 * static_cast<double>(i)};
        node["m_outputPins"].Items().at(0)["m_ID"] = StableGuid("parameter pin:" + name);
        declared[name] = {node.At("m_ID").String(), node.At("m_outputPins").Items().at(0).At("m_ID").String()};
        nodes.push_back(node);
    }
    root["m_nodes"] = nodes;

    kv::Value::Array connections;
    for (kv::Value& link : root["m_connections"].Items()) {
        const auto from = templateNames.find(link.At("m_fromNodeID").String());
        if (from != templateNames.end()) {
            const auto to = declared.find(from->second);
            if (to == declared.end())
                continue; // the stock graph has no such parameter; the input keeps its default
            link["m_fromNodeID"] = to->second.first;
            link["m_outputPinID"] = to->second.second;
        }
        connections.push_back(link);
    }
    root["m_connections"] = connections;
    return wrapper;
}

} // namespace ag2
