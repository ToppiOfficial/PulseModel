// rig.cpp - the DMX side: a rig's joints, the target pose, weapon helper
// insertion and the two held-pose animation DMXs.

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

void Set(dmx::Element* e, const char* name, dmx::AttrType type, dmx::AttrValue value) {
    for (dmx::Attribute& a : e->attributes) {
        if (a.name == name) {
            a.type = type;
            a.value = std::move(value);
            return;
        }
    }
    e->attributes.push_back({name, type, std::move(value)});
}

std::vector<dmx::Element*>& ElementArray(dmx::Element* e, const char* name) {
    if (!e->Get(name))
        Set(e, name, dmx::AttrType::ElementArray, std::vector<dmx::Element*>{});
    for (dmx::Attribute& a : e->attributes)
        if (a.name == name)
            if (auto* refs = std::get_if<std::vector<dmx::Element*>>(&a.value))
                return *refs;
    throw runtime_error(e->className + "." + name + " is not an element array");
}

// Derived from `key` (two FNV-1a passes) so a rerun writes the same bytes.
dmx::Guid StableId(const std::string& key) {
    dmx::Guid id{};
    uint64_t h[2] = {14695981039346656037ull, 0x9e3779b97f4a7c15ull};
    for (uint64_t& v : h)
        for (unsigned char c : "ag2proportions:" + key)
            v = (v ^ c) * 1099511628211ull;
    for (int i = 0; i < 16; ++i)
        id[i] = static_cast<uint8_t>(h[i / 8] >> (8 * (i % 8)));
    return id;
}

dmx::Element* NewTransform(dmx::Datamodel& dm, const std::string& key, const std::string& name,
                           const pm::Vector3& position, const pm::Quaternion& rotation) {
    dmx::Element* t = dm.CreateElement(StableId(key), "DmeTransform");
    t->name = name;
    Set(t, "position", dmx::AttrType::Vector3, dmx::Vector3{position.x, position.y, position.z});
    Set(t, "orientation", dmx::AttrType::Quaternion,
        dmx::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});
    return t;
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
                Add(e);
        }
        Link(model_);
        for (const auto& joint : joints_)
            Link(joint.second);
    }

    dmx::Element* Model() const { return model_; }
    dmx::Element* Find(const std::string& name) const {
        const auto it = joints_.find(name);
        return it == joints_.end() ? nullptr : it->second;
    }
    // nullptr for a bone that hangs off the DmeModel directly
    dmx::Element* Parent(const dmx::Element* joint) const {
        const auto it = parents_.find(joint);
        return it == parents_.end() ? nullptr : it->second;
    }
    std::string JointClass() const {
        return joints_.empty() ? "DmeJoint" : joints_.begin()->second->className;
    }
    void Add(dmx::Element* joint) {
        if (!joints_.emplace(joint->name, joint).second)
            throw runtime_error("duplicate bone " + joint->name);
    }
    // nullptr `parent` hangs the joint off the DmeModel
    void Reparent(dmx::Element* joint, dmx::Element* parent) {
        auto& from = ElementArray(Parent(joint) ? Parent(joint) : model_, "children");
        from.erase(std::remove(from.begin(), from.end(), joint), from.end());
        ElementArray(parent ? parent : model_, "children").push_back(joint);
        parents_[joint] = parent;
    }

private:
    void Link(dmx::Element* parent) {
        const auto* children = parent->GetElementArray("children");
        if (!children)
            return;
        for (const dmx::Element* child : *children) {
            if (!child)
                continue;
            dmx::Element* p = parent == model_ ? nullptr : parent;
            const auto ins = parents_.emplace(child, p);
            if (!ins.second && ins.first->second != p)
                throw runtime_error("bone " + child->name + " has more than one parent");
        }
    }

    dmx::Element* model_ = nullptr;
    std::map<std::string, dmx::Element*, NoCase> joints_;
    std::map<const dmx::Element*, dmx::Element*> parents_;
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

} // namespace

// ModelDoc takes enabled Bone nodes first, then SkeletonFiles in order.
// Files add new names under their own parents. Existing bones keep their
// parents; overwrite_existing replaces their transforms.
std::unique_ptr<dmx::Datamodel> BuildRig(const std::vector<Bone>& nodes, const std::vector<RigFile>& files,
                                         std::vector<NodeOverride>* overrides) {
    auto rig = std::make_unique<dmx::Datamodel>();
    rig->encoding = "binary";
    rig->encoding_version = 9;
    rig->format = "model";
    rig->format_version = 22;
    rig->root = rig->CreateElement(StableId("root"), "DmElement");
    rig->root->name = "root";
    dmx::Element* model = rig->CreateElement(StableId("model"), "DmeModel");
    model->name = "model";
    Set(model, "jointList", dmx::AttrType::ElementArray, std::vector<dmx::Element*>{});
    Set(model, "children", dmx::AttrType::ElementArray, std::vector<dmx::Element*>{});
    Set(rig->root, "skeleton", dmx::AttrType::Element, model);

    std::map<std::string, dmx::Element*, NoCase> joints;
    std::map<const dmx::Element*, dmx::Element*> parentOf; // nullptr: under the DmeModel
    std::map<const dmx::Element*, bool> isNode;
    auto add = [&](const std::string& name, const pm::Vector3& position, const pm::Quaternion& rotation,
                   dmx::Element* parent) {
        if (joints.count(name))
            throw runtime_error("two Bone nodes are named " + name);
        dmx::Element* joint = rig->CreateElement(StableId("rig joint:" + name), "DmeJoint");
        joint->name = name;
        Set(joint, "transform", dmx::AttrType::Element,
            NewTransform(*rig, "rig transform:" + name, name, position, rotation));
        Set(joint, "children", dmx::AttrType::ElementArray, std::vector<dmx::Element*>{});
        ElementArray(model, "jointList").push_back(joint);
        ElementArray(parent ? parent : model, "children").push_back(joint);
        joints[name] = joint;
        parentOf[joint] = parent;
        return joint;
    };
    auto nameOf = [](const dmx::Element* e) { return e ? e->name : std::string(); };

    for (const Bone& node : nodes) {
        dmx::Element* parent = node.parent >= 0 ? joints.at(nodes[node.parent].name) : nullptr;
        isNode[add(node.name, node.position, node.rotation, parent)] = true;
    }

    for (const RigFile& file : files) {
        const Rig source(*file.dm);
        // parent-first walk of the file's hierarchy
        std::function<void(const dmx::Element*)> visit = [&](const dmx::Element* from) {
            const auto* children = from->GetElementArray("children");
            if (!children)
                return;
            for (dmx::Element* child : *children) {
                if (!child || source.Find(child->name) != child)
                    continue;
                const dmx::Element* transform = TransformOf(child);
                const auto it = joints.find(child->name);
                if (it == joints.end()) {
                    const dmx::Element* fileParent = source.Parent(child);
                    const auto parent = fileParent ? joints.find(fileParent->name) : joints.end();
                    add(child->name, PositionOf(transform), RotationOf(transform),
                        parent == joints.end() ? nullptr : parent->second);
                } else {
                    dmx::Element* existing = TransformOf(it->second);
                    if (isNode[it->second] && overrides) {
                        const pm::Vector3 a = PositionOf(existing), b = PositionOf(transform);
                        NodeOverride o{it->second->name,
                                       file.overwrite ? 0.0f
                                                      : std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z),
                                       file.overwrite &&
                                           _stricmp(nameOf(parentOf[it->second]).c_str(),
                                                    nameOf(source.Parent(child)).c_str()) != 0};
                        if (o.moved > 1e-3f || o.reparented)
                            overrides->push_back(o);
                    }
                    if (file.overwrite) {
                        const pm::Vector3 position = PositionOf(transform);
                        const pm::Quaternion rotation = RotationOf(transform);
                        Set(existing, "position", dmx::AttrType::Vector3, dmx::Vector3{position.x, position.y, position.z});
                        Set(existing, "orientation", dmx::AttrType::Quaternion,
                            dmx::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w});
                    }
                }
                visit(child);
            }
        };
        visit(source.Model());
    }
    return rig;
}

std::vector<std::string> BoneNames(dmx::Datamodel& dm) {
    const Rig rig(dm);
    std::vector<std::string> names;
    for (const dmx::Element* e : *rig.Model()->GetElementArray("jointList"))
        if (e && rig.Find(e->name) == e)
            names.push_back(e->name);
    return names;
}

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

std::string BoneParent(dmx::Datamodel& dm, const std::string& name) {
    const Rig rig(dm);
    const dmx::Element* joint = rig.Find(name);
    const dmx::Element* parent = joint ? rig.Parent(joint) : nullptr;
    return parent ? parent->name : std::string();
}

bool IsCoreBone(const std::string& name) {
    for (const char* core : kCoreBones)
        if (_stricmp(core, name.c_str()) == 0)
            return true;
    return false;
}

std::vector<Bone> TargetPose(dmx::Datamodel& pose, const std::vector<Bone>& stock,
                             std::vector<std::string>& warnings, bool modelRig) {
    const Rig rig(pose);
    // root_motion always keeps its stock value, so only the model needs one
    if (modelRig && !rig.Find("root_motion"))
        throw runtime_error("the rig has no root_motion; add a root_motion Bone node to the VMDL Skeleton");
    for (const char* core : kCoreBones)
        if (std::string(core) != "root_motion" && !rig.Find(core))
            throw runtime_error(std::string(modelRig ? "the rig" : "the proportions DMX") + " lacks CS2 core bone " + core);
    const auto held = HeldPositions(pose);

    std::vector<Bone> target = stock;
    for (size_t i = 0; i < stock.size(); ++i) {
        const Bone& bone = stock[i];
        const dmx::Element* joint = rig.Find(bone.name);
        if (!joint || bone.name == "root_motion" || IsGraphHelper(bone.name))
            continue;
        const dmx::Element* transform = TransformOf(joint);
        const dmx::Element* parent = rig.Parent(joint);
        const bool standalonePelvis = bone.name == "pelvis" && !parent;
        if (bone.parent >= 0 && !standalonePelvis &&
            (!parent || _stricmp(parent->name.c_str(), stock[bone.parent].name.c_str()) != 0))
            throw runtime_error("the pose hierarchy differs from stock at " + bone.name);

        if (RotationDot(RotationOf(transform), bone.rotation) < 0.999f)
            warnings.push_back("bind rotation differs on " + bone.name +
                               "; the clip changes translations only, check your rig axes");

        const auto it = held.find(transform);
        pm::Vector3 position = it != held.end() ? it->second : PositionOf(transform);
        // A pelvis with no root_motion above it is in model space: keep the
        // stock pelvis's horizontal origin and take only the custom height.
        if (standalonePelvis) {
            const Bone& root = stock[bone.parent];
            const pm::matrix3x4 rootToModel = pm::QuaternionMatrix(root.rotation, root.position);
            pm::Vector3 model = pm::VectorTransform(bone.position, rootToModel);
            model.z = position.z;
            position = pm::VectorITransform(model, rootToModel);
            warnings.push_back("standalone pelvis: custom height, stock horizontal origin");
        }
        RequireFinite(position, bone.name);
        target[i].position = position;
    }
    return target;
}

// Every shipped agent binds wpnPivot at its stock offset unrotated and wpn back at
// root_motion's origin (pak01 agents/models/*.vmdl_c), not at the graph's pose.
static Bone ModelHelperBind(const Bone& bone, const std::vector<Bone>& stock) {
    Bone b = bone;
    b.rotation = pm::Quaternion{};
    if (b.name == "wpn") {
        const pm::Vector3& pivot = stock[bone.parent].position;
        b.position = {-pivot.x, -pivot.y, -pivot.z};
    }
    return b;
}

HelperPlan AddWeaponHelpers(dmx::Datamodel& model, const std::vector<Bone>& stock,
                            const std::vector<std::string>& present, const std::vector<std::string>& nodeNames) {
    auto named = [](const std::vector<std::string>& names, const std::string& name) {
        for (const std::string& n : names)
            if (_stricmp(n.c_str(), name.c_str()) == 0)
                return true;
        return false;
    };
    for (const char* helper : kModelHelpers)
        if (std::none_of(stock.begin(), stock.end(), [&](const Bone& b) { return b.name == helper; }))
            throw runtime_error(std::string("the stock skeleton lacks weapon helper ") + helper);

    Rig rig(model);
    dmx::Element* dmeModel = rig.Model();
    std::vector<dmx::Element*>& jointList = ElementArray(dmeModel, "jointList");
    const std::string jointClass = rig.JointClass();
    HelperPlan plan;
    std::vector<std::string> nodeHelpers;

    // stock order is parent-first, so a helper's helper parent is placed before it
    for (const Bone& stockBone : stock) {
        if (!IsModelHelper(stockBone.name) || rig.Find(stockBone.name) || named(present, stockBone.name))
            continue;
        const Bone bone = ModelHelperBind(stockBone, stock);
        const std::string parentName = bone.parent >= 0 ? stock[bone.parent].name : std::string();
        dmx::Element* parent = parentName.empty() ? nullptr : rig.Find(parentName);
        if (!parent) {
            if (!named(nodeNames, parentName) && !named(nodeHelpers, parentName))
                throw runtime_error("weapon helper " + bone.name + " needs parent " + parentName +
                                    ", which is in neither the rig nor the VMDL Bone nodes");
            plan.asNodes.push_back(bone);
            nodeHelpers.push_back(bone.name);
            continue;
        }

        dmx::Element* joint = model.CreateElement(StableId("joint:" + bone.name), jointClass);
        joint->name = bone.name;
        Set(joint, "transform", dmx::AttrType::Element,
            NewTransform(model, "transform:" + bone.name, bone.name, bone.position, bone.rotation));
        Set(joint, "children", dmx::AttrType::ElementArray, std::vector<dmx::Element*>{});
        // Bind states parallel jointList. One shorter than jointList falls back
        // on the live transforms for its tail, as the compiler's loader does.
        if (const auto* states = dmeModel->GetElementArray("baseStates")) {
            for (dmx::Element* state : *states) {
                std::vector<dmx::Element*>& transforms = ElementArray(state, "transforms");
                if (transforms.size() > jointList.size())
                    throw runtime_error("bind state " + state->name + " has more transforms than jointList");
                while (transforms.size() < jointList.size()) {
                    const dmx::Element* live = TransformOf(jointList[transforms.size()]);
                    const std::string key = "bind:" + dmx::GuidToString(state->id) + ":" + std::to_string(transforms.size());
                    transforms.push_back(NewTransform(model, key, live->name, PositionOf(live), RotationOf(live)));
                }
                transforms.push_back(NewTransform(model, "bind:" + dmx::GuidToString(state->id) + ":" + bone.name,
                                                  bone.name, bone.position, bone.rotation));
            }
        }
        jointList.push_back(joint);
        rig.Add(joint);
        ElementArray(parent, "children").push_back(joint);
        plan.inRig.push_back(bone);
    }
    return plan;
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
    q.Ref("animationList", idList);
    q.Ref("exportTags", idTags);

    q.Begin("DmeExportTags", idTags, "exportTags");
    q.Str("app", "sfm");
    q.Str("source", "ag2proportions");

    q.Begin("DmeModel", idModel, "worldmodel");
    q.Ref("transform", idModelXform);
    q.Ref("axisSystem", idAxis);
    q.Bool("visible", true);
    q.RefArray("jointList", idJoint);
    q.RefArray("baseStates", {idBind});
    q.RefArray("children", jointIds(CompiledChildren(bones, lowLodCount, -1, minLow, minHigh)));

    q.Begin("DmeAxisSystem", idAxis, "axisSystem");
    q.Int("upAxis", 3);
    q.Int("forwardParity", 1);
    q.Int("coordSys", 0);

    q.Begin("DmeTransform", idModelXform, "worldmodel");
    q.Vec3("position", pm::Vector3{});
    q.Quat("orientation", pm::Quaternion{});

    q.Begin("DmeTransformList", idBind, "bind");
    q.RefArray("transforms", idXform);

    for (int i = 0; i < n; ++i) {
        q.Begin("DmeJoint", idJoint[i], bones[i].name);
        q.Ref("transform", idXform[i]);
        q.RefArray("children", jointIds(CompiledChildren(bones, lowLodCount, i, minLow, minHigh)));
        q.Begin("DmeTransform", idXform[i], bones[i].name);
        q.Vec3("position", bones[i].position);
        q.Quat("orientation", bones[i].rotation);
    }

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

    std::string err;
    if (!q.Save(path.u8string(), &err))
        throw runtime_error(err);
}

} // namespace ag2
