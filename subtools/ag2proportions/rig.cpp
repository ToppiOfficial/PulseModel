// rig.cpp - the DMX side: a rig's joints, the generated skeleton, the target
// pose and the two held-pose animation DMXs.

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>

#include "ag2.h"
#include "strcompat.h"

namespace ag2 {
namespace {

namespace dmx = pulse::dmx;
using std::runtime_error;

struct NoCase {
    bool operator()(const std::string& a, const std::string& b) const {
        return _stricmp(a.c_str(), b.c_str()) < 0;
    }
};

const char* const kCoreBones[] = {
    "root_motion", "pelvis", "spine_0", "spine_1", "spine_2", "spine_3", "neck_0", "head_0",
    "clavicle_L", "clavicle_R", "arm_upper_L", "arm_upper_R", "arm_lower_L", "arm_lower_R",
    "hand_L", "hand_R", "leg_upper_L", "leg_upper_R", "leg_lower_L", "leg_lower_R",
    "ankle_L", "ankle_R",
};

// Weapon helpers in the graph's skeleton; the clip keeps their stock values.
const char* const kGraphHelpers[] = {
    "wpnPivot", "wpnAimIntent", "attachWorld", "wpn", "wpnHand_L",
    "wpnHand_R", "wpnTip", "wpnEnd", "attachHand_L", "attachHand_R",
};

// The helpers every shipped agent model carries (pak01 agents/models/*.vmdl_c).
const char* const kModelHelpers[] = {"wpnPivot", "wpn"};

bool IsGraphHelper(const std::string& name) {
    for (const char* helper : kGraphHelpers)
        if (_stricmp(helper, name.c_str()) == 0)
            return true;
    return false;
}

bool IsCoreBone(const std::string& name) {
    for (const char* core : kCoreBones)
        if (_stricmp(core, name.c_str()) == 0)
            return true;
    return false;
}

bool IsModelHelper(const std::string& name) {
    for (const char* helper : kModelHelpers)
        if (_stricmp(helper, name.c_str()) == 0)
            return true;
    return false;
}

pm::Quaternion Normalized(pm::Quaternion q) {
    const float length = pm::QuaternionNormalize(q);
    if (!std::isfinite(length) || length < 1e-8f)
        throw runtime_error("invalid bone quaternion");
    return q;
}

void RequireFinite(const pm::Vector3& v, const std::string& bone) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
        throw runtime_error("invalid position on " + bone);
}

// Degrees between two offsets; 0 when either is too short to have a direction.
float OffsetAngle(const pm::Vector3& a, const pm::Vector3& b) {
    const float la = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z), lb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    if (la < 0.05f || lb < 0.05f)
        return 0.0f;
    const float c = (a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb);
    return std::acos(std::fmin(1.0f, std::fmax(-1.0f, c))) * pm::kRad2Deg;
}

float RotationDot(const pm::Quaternion& a, const pm::Quaternion& b) {
    return std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

dmx::Element* TransformOf(const dmx::Element* joint) {
    dmx::Element* transform = joint->GetElement("transform");
    if (!transform)
        throw runtime_error("missing transform on " + joint->name);
    return transform;
}

pm::Vector3 PositionOf(const dmx::Element* transform) {
    const dmx::Vector3 p = transform->GetVector3("position");
    return {p.x, p.y, p.z};
}

pm::Quaternion RotationOf(const dmx::Element* transform) {
    const dmx::Attribute* a = transform->Get("orientation");
    const auto* q = a ? std::get_if<dmx::Quaternion>(&a->value) : nullptr;
    if (!q)
        throw runtime_error("missing orientation on " + transform->name);
    return Normalized({q->x, q->y, q->z, q->w});
}

pm::matrix3x4 Local(const Bone& bone) { return pm::QuaternionMatrix(bone.rotation, bone.position); }

// Model-space transforms of a parent-first bone list.
std::vector<pm::matrix3x4> WorldTransforms(const std::vector<Bone>& bones) {
    std::vector<pm::matrix3x4> world;
    for (const Bone& bone : bones)
        world.push_back(bone.parent >= 0 ? pm::ConcatTransforms(world[bone.parent], Local(bone)) : Local(bone));
    return world;
}

// A DMX's skeleton by bone name: the DmeModel's jointList without mesh dags.
class Rig {
public:
    explicit Rig(dmx::Datamodel& dm) {
        if (dm.root)
            model_ = dm.root->GetElement("skeleton") ? dm.root->GetElement("skeleton")
                                                     : dm.root->GetElement("model");
        if (!model_ || model_->className != "DmeModel")
            throw runtime_error("the DMX has no DmeModel skeleton");
        const auto* list = model_->GetElementArray("jointList");
        if (!list)
            throw runtime_error("the DmeModel has no jointList");
        // Valve's joints carry a DmeShape; only a DmeMesh shape marks a mesh dag
        for (dmx::Element* e : *list) {
            const dmx::Element* shape = e ? e->GetElement("shape") : nullptr;
            if (e && e->GetElement("transform") && !(shape && shape->className == "DmeMesh"))
                if (!joints_.emplace(e->name, e).second)
                    throw runtime_error("duplicate bone " + e->name);
        }
    }

    dmx::Element* Model() const { return model_; }
    dmx::Element* Find(const std::string& name) const {
        const auto it = joints_.find(name);
        return it == joints_.end() ? nullptr : it->second;
    }
    pm::matrix3x4 ModelTransform() const {
        const dmx::Element* transform = model_->GetElement("transform");
        return transform ? pm::QuaternionMatrix(RotationOf(transform), PositionOf(transform))
                         : pm::QuaternionMatrix(pm::Quaternion{}, pm::Vector3{});
    }

private:
    dmx::Element* model_ = nullptr;
    std::map<std::string, dmx::Element*, NoCase> joints_;
};

// The first key of each position channel. A held pose has one clip whose
// position logs carry a single layer; anything else is refused, not guessed.
std::map<const dmx::Element*, pm::Vector3> HeldPositions(const dmx::Datamodel& dm) {
    std::map<const dmx::Element*, pm::Vector3> held;
    const auto clips = std::count_if(dm.elements.begin(), dm.elements.end(),
                                     [](const auto& e) { return e->className == "DmeChannelsClip"; });
    if (clips > 1)
        throw runtime_error("the pose DMX holds " + std::to_string(clips) +
                            " clips; export a single held pose");
    for (const auto& e : dm.elements) {
        if (e->className != "DmeChannel")
            continue;
        const std::string* attribute = e->GetString("toAttribute");
        const dmx::Element* target = e->GetElement("toElement");
        if (!attribute || *attribute != "position" || !target)
            continue;
        const dmx::Element* log = e->GetElement("log");
        const auto* layers = log ? log->GetElementArray("layers") : nullptr;
        if (!layers || layers->size() != 1 || !layers->front())
            throw runtime_error("position channel " + e->name + " needs exactly one log layer");
        const auto* values = layers->front()->GetVector3Array("values");
        if (!values || values->empty())
            throw runtime_error("position channel " + e->name + " has no keys");
        const dmx::Vector3& v = values->front();
        if (!held.emplace(target, pm::Vector3{v.x, v.y, v.z}).second)
            throw runtime_error("two position channels drive " + target->name);
    }
    return held;
}

// Children of `parent` in compiled bone order. The skeleton compiler lays out
// low-LOD bones first, so siblings sort by their subtree's lowest low-LOD index,
// with high-LOD-only subtrees slotted in by their lowest high-LOD index.
std::vector<int> CompiledChildren(const std::vector<Bone>& bones, int lowLodCount, int parent,
                                  const std::vector<int>& minLow, const std::vector<int>& minHigh) {
    std::vector<int> low, high;
    for (int i = 0; i < static_cast<int>(bones.size()); ++i)
        if (bones[i].parent == parent)
            (minLow[i] == INT_MAX ? high : low).push_back(i);
    std::sort(low.begin(), low.end(), [&](int a, int b) { return minLow[a] < minLow[b]; });
    std::sort(high.begin(), high.end(), [&](int a, int b) { return minHigh[a] < minHigh[b]; });
    std::vector<int> order;
    size_t next = 0;
    for (int i : low) {
        if (minHigh[i] != INT_MAX)
            while (next < high.size() && minHigh[high[next]] < minHigh[i])
                order.push_back(high[next++]);
        order.push_back(i);
    }
    order.insert(order.end(), high.begin() + static_cast<std::ptrdiff_t>(next), high.end());
    return order;
}

// Every shipped agent binds wpnPivot at its stock offset unrotated and wpn back at
// root_motion's origin (pak01 agents/models/*.vmdl_c), not at the graph's pose.
Bone ModelHelperBind(const Bone& bone, const std::vector<Bone>& stock) {
    Bone b = bone;
    b.rotation = pm::Quaternion{};
    if (b.name == "wpn") {
        const pm::Vector3& pivot = stock[bone.parent].position;
        b.position = {-pivot.x, -pivot.y, -pivot.z};
    }
    return b;
}

// A model DMX holding `bones`; `children[i]` orders bone i's children and
// `children[n]` the roots. `held` adds a two-frame clip holding the pose still.
void SaveModelDmx(const fs::path& path, const char* modelName, const std::vector<Bone>& bones,
                  const std::vector<std::vector<int>>& children, bool held) {
    const int n = static_cast<int>(bones.size());
    dmx::Builder q("model", 22, "binary", 9, 0x414732);
    const dmx::Guid idRoot = q.NewId(), idTags = q.NewId(), idModel = q.NewId(), idAxis = q.NewId(),
                    idModelXform = q.NewId(), idBind = q.NewId(), idList = q.NewId(),
                    idClip = q.NewId(), idFrame = q.NewId();
    std::vector<dmx::Guid> idJoint(n), idXform(n), idChannel(n * 2);
    for (int i = 0; i < n; ++i) {
        idJoint[i] = q.NewId();
        idXform[i] = q.NewId();
    }
    for (dmx::Guid& id : idChannel)
        id = q.NewId();
    auto jointIds = [&](const std::vector<int>& indices) {
        std::vector<dmx::Guid> ids;
        for (int i : indices)
            ids.push_back(idJoint[i]);
        return ids;
    };

    q.Begin("DmElement", idRoot, "root");
    q.Ref("skeleton", idModel);
    if (held)
        q.Ref("animationList", idList);
    q.Ref("exportTags", idTags);

    q.Begin("DmeExportTags", idTags, "exportTags");
    q.Str("app", "sfm");
    q.Str("source", "ag2proportions");

    q.Begin("DmeModel", idModel, modelName);
    q.Ref("transform", idModelXform);
    q.Ref("axisSystem", idAxis);
    q.Bool("visible", true);
    q.RefArray("jointList", idJoint);
    q.RefArray("baseStates", {idBind});
    q.RefArray("children", jointIds(children[n]));

    q.Begin("DmeAxisSystem", idAxis, "axisSystem");
    q.Int("upAxis", 3);
    q.Int("forwardParity", 1);
    q.Int("coordSys", 0);

    q.Begin("DmeTransform", idModelXform, modelName);
    q.Vec3("position", pm::Vector3{});
    q.Quat("orientation", pm::Quaternion{});

    q.Begin("DmeTransformList", idBind, "bind");
    q.RefArray("transforms", idXform);

    for (int i = 0; i < n; ++i) {
        q.Begin("DmeJoint", idJoint[i], bones[i].name);
        q.Ref("transform", idXform[i]);
        q.RefArray("children", jointIds(children[i]));
        q.Begin("DmeTransform", idXform[i], bones[i].name);
        q.Vec3("position", bones[i].position);
        q.Quat("orientation", bones[i].rotation);
    }

    if (held) {
        const float frame = 1.0f / 30.0f;
        q.Begin("DmeAnimationList", idList, "animationList");
        q.RefArray("animations", {idClip});

        q.Begin("DmeChannelsClip", idClip, "proportions");
        q.Ref("timeFrame", idFrame);
        q.Float("frameRate", 30.0f);
        q.RefArray("channels", idChannel);

        q.Begin("DmeTimeFrame", idFrame, "timeFrame");
        q.Time("start", 0.0f);
        q.Time("duration", frame);
        q.Time("offset", 0.0f);
        q.Float("scale", 1.0f);

        for (int i = 0; i < n * 2; ++i) {
            const Bone& bone = bones[i / 2];
            const bool position = i % 2 == 0;
            const dmx::Guid idLog = q.NewId(), idLayer = q.NewId();
            q.Begin("DmeChannel", idChannel[i], bone.name + (position ? "_p" : "_o"));
            q.Ref("toElement", idXform[i / 2]);
            q.Str("toAttribute", position ? "position" : "orientation");
            q.Int("mode", 3);
            q.Ref("log", idLog);

            q.Begin(position ? "DmeVector3Log" : "DmeQuaternionLog", idLog, bone.name);
            q.RefArray("layers", {idLayer});

            q.Begin(position ? "DmeVector3LogLayer" : "DmeQuaternionLogLayer", idLayer, bone.name);
            q.TimeArray("times", {0.0f, frame});
            if (position)
                q.V3Array("values", std::vector<pm::Vector3>{bone.position, bone.position});
            else
                q.QuatArray("values", std::vector<pm::Quaternion>{bone.rotation, bone.rotation});
        }
    }

    std::string err;
    if (!q.Save(path.u8string(), &err))
        throw runtime_error(err);
}

} // namespace

size_t CountBones(dmx::Datamodel& dm) {
    try {
        const Rig rig(dm);
        size_t n = 0;
        for (const char* core : kCoreBones)
            n += rig.Find(core) ? 1 : 0;
        return n;
    } catch (const std::exception&) {
        return 0;
    }
}

std::vector<Bone> ReadRig(dmx::Datamodel& dm, float scale, bool held) {
    const Rig rig(dm);
    const auto keys = held ? HeldPositions(dm) : std::map<const dmx::Element*, pm::Vector3>{};
    const pm::matrix3x4 model = rig.ModelTransform();
    std::vector<Bone> bones;
    std::map<const dmx::Element*, int> index;
    std::function<void(const dmx::Element*)> visit = [&](const dmx::Element* from) {
        const auto* children = from->GetElementArray("children");
        if (!children)
            return;
        for (const dmx::Element* child : *children) {
            if (!child || rig.Find(child->name) != child)
                continue;
            if (index.count(child))
                throw runtime_error("bone " + child->name + " has more than one parent");
            const dmx::Element* transform = TransformOf(child);
            const auto key = keys.find(transform);
            Bone bone;
            bone.name = child->name;
            bone.parent = from == rig.Model() ? -1 : index.at(from);
            bone.position = key != keys.end() ? key->second : PositionOf(transform);
            bone.rotation = RotationOf(transform);
            if (bone.parent < 0)
                pm::MatrixAngles(pm::ConcatTransforms(model, Local(bone)), bone.rotation, bone.position);
            bone.position = {bone.position.x * scale, bone.position.y * scale, bone.position.z * scale};
            RequireFinite(bone.position, bone.name);
            index[child] = static_cast<int>(bones.size());
            bones.push_back(bone);
            visit(child);
        }
    };
    visit(rig.Model());
    return bones;
}

bool HasStockLengths(const std::vector<Bone>& rig, const std::vector<const std::vector<Bone>*>& skeletons) {
    std::map<std::string, int, NoCase> inRig;
    for (size_t i = 0; i < rig.size(); ++i)
        inRig.emplace(rig[i].name, static_cast<int>(i));
    const std::vector<pm::matrix3x4> world = WorldTransforms(rig);
    auto origin = [&](int i) { return pm::Vector3{world[i].m[0][3], world[i].m[1][3], world[i].m[2][3]}; };
    int pairs = 0;
    for (const std::vector<Bone>* skeleton : skeletons) {
        const std::vector<Bone>& stock = *skeleton;
        for (const Bone& s : stock) {
            // root_motion children sit wherever the rig was exported, not at a bone length
            if (s.parent < 0 || IsGraphHelper(s.name) || IsGraphHelper(stock[s.parent].name) ||
                stock[s.parent].name == "root_motion")
                continue;
            const auto bone = inRig.find(s.name), parent = inRig.find(stock[s.parent].name);
            if (bone == inRig.end() || parent == inRig.end())
                continue;
            const pm::Vector3 a = origin(bone->second), b = origin(parent->second);
            const float mine = std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
            const float theirs = std::sqrt(s.position.x * s.position.x + s.position.y * s.position.y +
                                           s.position.z * s.position.z);
            if (std::abs(mine - theirs) > 0.1f)
                return false;
            ++pairs;
        }
    }
    return pairs >= 3;
}

SkeletonBuild BuildSkeleton(const std::vector<Bone>& rig, const std::vector<Bone>& stock,
                            const std::vector<Bone>& nodes) {
    std::map<std::string, int, NoCase> inRig, isStock, placed;
    for (size_t i = 0; i < rig.size(); ++i)
        if (!inRig.emplace(rig[i].name, static_cast<int>(i)).second)
            throw runtime_error("two rig bones are named " + rig[i].name);
    for (size_t i = 0; i < stock.size(); ++i)
        isStock.emplace(stock[i].name, static_cast<int>(i));
    for (const char* core : kCoreBones)
        if (std::string(core) != "root_motion" && !inRig.count(core))
            throw runtime_error(std::string("the rig lacks CS2 core bone ") + core);

    const std::vector<pm::matrix3x4> rigWorld = WorldTransforms(rig);
    SkeletonBuild out;
    std::vector<pm::matrix3x4> world;
    // `fixed`: keep this model-space transform under the new parent
    auto place = [&](Bone bone, int parent, const pm::matrix3x4* fixed) {
        bone.parent = parent;
        if (fixed) {
            const pm::matrix3x4 local =
                parent >= 0 ? pm::ConcatTransforms(pm::MatrixInvert(world[parent]), *fixed) : *fixed;
            pm::MatrixAngles(local, bone.rotation, bone.position);
        }
        world.push_back(fixed ? *fixed : parent >= 0 ? pm::ConcatTransforms(world[parent], Local(bone)) : Local(bone));
        placed[bone.name] = static_cast<int>(out.bones.size());
        out.bones.push_back(bone);
    };
    auto parentName = [](const std::vector<Bone>& bones, const Bone& bone) {
        return bone.parent >= 0 ? bones[bone.parent].name : std::string();
    };

    // stock order is parent-first; graph-only helpers stay out of the model
    for (const Bone& s : stock) {
        const auto r = inRig.find(s.name);
        if (r == inRig.end() && IsGraphHelper(s.name) && !IsModelHelper(s.name))
            continue;
        int parent = -1;
        if (s.parent >= 0) {
            const auto p = placed.find(stock[s.parent].name);
            if (p == placed.end())
                continue; // under a graph-only helper: a rig bone keeps its rig parent below
            parent = p->second;
        }
        if (r == inRig.end()) {
            place(IsModelHelper(s.name) ? ModelHelperBind(s, stock) : s, parent, nullptr);
            out.added.push_back(s.name);
            continue;
        }
        const Bone& bone = rig[r->second];
        const std::string want = parent >= 0 ? out.bones[parent].name : std::string();
        if (_stricmp(parentName(rig, bone).c_str(), want.c_str()) == 0) {
            place(bone, parent, nullptr);
        } else {
            place(bone, parent, &rigWorld[r->second]);
            out.reparented.push_back(bone.name + " -> " + (want.empty() ? "no parent" : want));
        }
    }
    // rig order is parent-first, so a custom bone's parent is placed before it
    for (const Bone& bone : rig)
        if (!placed.count(bone.name))
            place(bone, bone.parent >= 0 ? placed.at(rig[bone.parent].name) : -1, nullptr);
    for (const Bone& node : nodes) {
        if (placed.count(node.name) || isStock.count(node.name))
            continue;
        const auto p = node.parent >= 0 ? placed.find(nodes[node.parent].name) : placed.end();
        place(node, p == placed.end() ? -1 : p->second, nullptr);
        out.fromNodes.push_back(node.name);
    }
    return out;
}

std::vector<Bone> TargetPose(const std::vector<Bone>& skeleton, const std::vector<Bone>& stock,
                             std::vector<std::string>& warnings) {
    std::map<std::string, const Bone*, NoCase> byName;
    for (const Bone& bone : skeleton)
        byName.emplace(bone.name, &bone);
    std::vector<Bone> target = stock;
    for (size_t i = 0; i < stock.size(); ++i) {
        const Bone& bone = stock[i];
        const auto it = byName.find(bone.name);
        if (it == byName.end() || bone.name == "root_motion" || IsGraphHelper(bone.name))
            continue;
        const Bone& mine = *it->second;
        if (bone.parent >= 0 && (mine.parent < 0 || _stricmp(skeleton[mine.parent].name.c_str(),
                                                             stock[bone.parent].name.c_str()) != 0))
            throw runtime_error("the skeleton hierarchy differs from stock at " + bone.name);
        // The clip moves translations along stock axes. A T/A-pose or body shape
        // turns an offset by well under 75 degrees; swapped or flipped bone axes
        // turn it by about 90 or 180.
        const float angle = OffsetAngle(mine.position, bone.position);
        if (angle > 75.0f)
            warnings.push_back("bone axes differ on " + bone.name + ": its offset from " +
                               stock[bone.parent].name + " points " + std::to_string(static_cast<int>(angle)) +
                               " degrees from stock, so the clip may place it off");
        target[i].position = mine.position;
    }
    return target;
}

void WriteSkeleton(const fs::path& path, const std::vector<Bone>& bones) {
    std::vector<std::vector<int>> children(bones.size() + 1);
    for (size_t i = 0; i < bones.size(); ++i)
        children[bones[i].parent >= 0 ? bones[i].parent : bones.size()].push_back(static_cast<int>(i));
    SaveModelDmx(path, "skeleton", bones, children, false);
}

// A two-frame clip holding `bones` still, on a copy of the stock skeleton.
void WriteHeldPose(const fs::path& path, const std::vector<Bone>& bones, int lowLodCount) {
    const int n = static_cast<int>(bones.size());
    std::vector<int> minLow(n, INT_MAX), minHigh(n, INT_MAX);
    for (int i = n - 1; i >= 0; --i) {
        (i < lowLodCount ? minLow[i] : minHigh[i]) = i;
        if (const int p = bones[i].parent; p >= 0) {
            minLow[p] = std::min(minLow[p], minLow[i]);
            minHigh[p] = std::min(minHigh[p], minHigh[i]);
        }
    }
    std::vector<std::vector<int>> children(n + 1);
    for (int i = -1; i < n; ++i)
        children[i < 0 ? n : i] = CompiledChildren(bones, lowLodCount, i, minLow, minHigh);
    SaveModelDmx(path, "worldmodel", bones, children, true);
}

} // namespace ag2
