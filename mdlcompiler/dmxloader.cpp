// dmxloader.cpp - DMX document -> per-file Source. See dmxloader.h.

#include "perf.h"
#include "dmxloader.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <unordered_map>
#include <vector>

#include "math/math.h"
#include "pulselimits.h"
namespace pulse::source {

namespace dmx = pulse::dmx;
namespace pm = pulse::math;
namespace lim = pulse::limits;

namespace {

pm::Vector3 ToV3(const dmx::Vector3& v) { return {v.x, v.y, v.z}; }
pm::Vector2 ToV2(const dmx::Vector2& v) { return {v.x, v.y}; }

const dmx::Vector3* DmxVec3(const dmx::Element* e, const char* n) {
    if (!e) return nullptr;
    const dmx::Attribute* a = e->Get(n);
    return a ? std::get_if<dmx::Vector3>(&a->value) : nullptr;
}
const dmx::Quaternion* DmxQuat(const dmx::Element* e, const char* n) {
    if (!e) return nullptr;
    const dmx::Attribute* a = e->Get(n);
    return a ? std::get_if<dmx::Quaternion>(&a->value) : nullptr;
}
const std::vector<dmx::Quaternion>* DmxQuatArray(const dmx::Element* e, const char* n) {
    if (!e) return nullptr;
    const dmx::Attribute* a = e->Get(n);
    return a ? std::get_if<std::vector<dmx::Quaternion>>(&a->value) : nullptr;
}

// model-version field-name split (model 22 uses position$0 etc.)
const std::vector<dmx::Vector3>* PickV3Array(const dmx::Element* e, const char* a, const char* b) {
    auto v = e->GetVector3Array(a);
    return v ? v : e->GetVector3Array(b);
}
const std::vector<int32_t>* PickIntArray(const dmx::Element* e, const char* a, const char* b) {
    auto v = e->GetIntArray(a);
    return v ? v : e->GetIntArray(b);
}
const std::vector<float>* PickFloatArray(const dmx::Element* e, const char* a, const char* b) {
    auto v = e->GetFloatArray(a);
    return v ? v : e->GetFloatArray(b);
}

// DmeTransform -> matrix (reference CDmeTransform::GetTransform =
// QuaternionMatrix(orientation, position)).
pm::matrix3x4 TransformMatrix(const dmx::Element* t) {
    pm::Vector3 pos{0, 0, 0};
    pm::Quaternion rot{0, 0, 0, 1};
    if (auto p = DmxVec3(t, "position")) pos = ToV3(*p);
    if (auto q = DmxQuat(t, "orientation")) rot = {q->x, q->y, q->z, q->w};
    return pm::QuaternionMatrix(rot, pos);
}

// sticky upAxis-Y detection (reference bSetUpAxis handling)
bool s_bUpAxisY = false;
bool s_bUpAxisChecked = false;

// ---------------------------------------------------------------------------
// Skeleton walk (reference AddDagJoint/LoadSkeleton). Creates a localBone for
// every child dag of the skeleton root (mesh dags included), then appends the
// weightless "defaultRoot".
// ---------------------------------------------------------------------------
struct BoneMap {
    std::vector<const dmx::Element*> transforms; // per mdl bone: its DmeTransform
    std::vector<const dmx::Element*> dags;       // per mdl bone: its dag
    std::vector<int> dmeModelToMdl;              // jointList idx -> mdl bone
    std::vector<int> mdlToDmeModel;              // mdl bone -> jointList idx
    int defaultRootNode = -1;
};

// model 15+ store jointList (joint dags); model 7 stores only jointTransforms
// (their DmeTransforms) in the same index space.
const std::vector<dmx::ElementPtr>* JointArray(const dmx::Element* model) {
    if (!model) return nullptr;
    if (auto j = model->GetElementArray("jointList")) return j;
    return model->GetElementArray("jointTransforms");
}

// the DmeTransform of a joint-array entry - the entry itself when it is one
const dmx::Element* JointTransform(const dmx::Element* j) {
    if (!j) return nullptr;
    return j->className == "DmeTransform" ? j : j->GetElement("transform");
}

// joint lookup by element identity (reference CDmeModel::GetJointIndex); matches
// the dag or its transform so one pass covers both spellings
int GetJointIndex(const std::vector<dmx::ElementPtr>* jointList, const dmx::Element* dag) {
    if (!jointList) return -1;
    const dmx::Element* xform = dag ? dag->GetElement("transform") : nullptr;
    for (size_t i = 0; i < jointList->size(); ++i)
        if ((*jointList)[i] == dag || ((*jointList)[i] && (*jointList)[i] == xform))
            return static_cast<int>(i);
    return -1;
}

bool AddDagJoint(const std::vector<dmx::ElementPtr>* jointList, const dmx::Element* dag,
                 Source& out, int parentIndex, BoneMap& boneMap) {
    const dmx::Element* transform = dag->GetElement("transform");
    if (!transform)
        return true;

    const int jointIndex = static_cast<int>(boneMap.transforms.size());
    boneMap.transforms.push_back(transform);
    boneMap.dags.push_back(dag);

    int found = GetJointIndex(jointList, dag);
    if (found >= 0) {
        if (found >= static_cast<int>(boneMap.dmeModelToMdl.size()))
            boneMap.dmeModelToMdl.resize(found + 1, -1);
        boneMap.dmeModelToMdl[found] = jointIndex;
    }
    boneMap.mdlToDmeModel.push_back(found);

    LocalBone lb;
    lb.name = dag->name;
    lb.parent = parentIndex;
    const dmx::Element* shape = dag->GetElement("shape");
    lb.isNonSkeletal = (shape && shape->className == "DmeMesh");
    out.localBone.push_back(lb);

    if (auto kids = dag->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c && !AddDagJoint(jointList, c, out, jointIndex, boneMap))
                return false;
    return true;
}

int LoadSkeleton(const dmx::Element* skeletonRoot, const dmx::Element* model,
                 Source& out, BoneMap& boneMap) {
    const std::vector<dmx::ElementPtr>* jointList = JointArray(model);

    // never create a joint for the root dag itself - just its children
    if (auto kids = skeletonRoot->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c) AddDagJoint(jointList, c, out, -1, boneMap);

    // default identity bone used for autoskinning when no joints are specified
    boneMap.defaultRootNode = static_cast<int>(boneMap.transforms.size());
    LocalBone lb;
    lb.name = "defaultRoot";
    lb.parent = -1;
    lb.isNonSkeletal = true; // synthetic - only survives if vertices actually land on it
    out.localBone.push_back(lb);

    return static_cast<int>(boneMap.transforms.size()) + 1;
}

// ---------------------------------------------------------------------------
// Bind pose (reference LoadBindPose): "BindPose" anim, one frame; per bone the
// bind base-state transform when the joint is in jointList, else the live dag
// transform. defaultRoot stays zeroed.
// ---------------------------------------------------------------------------
const dmx::Element* FindBaseState(const dmx::Element* model, const char* name) {
    if (!model) return nullptr;
    if (auto states = model->GetElementArray("baseStates"))
        for (const dmx::Element* s : *states)
            if (s && s->name == name) return s;
    return nullptr;
}

void LoadBindPose(const dmx::Element* model, float flScale, const BoneMap& boneMap, Source& out) {
    SourceAnim anim;
    anim.name = "BindPose";
    anim.startframe = 0;
    anim.endframe = 0;
    anim.numframes = 1;
    anim.frames.resize(1);
    anim.frames[0].resize(out.numbones); // zero-initialized (defaultRoot stays 0)

    const dmx::Element* bindPose = FindBaseState(model, "bind");
    const std::vector<dmx::ElementPtr>* bindTransforms =
        bindPose ? bindPose->GetElementArray("transforms") : nullptr;
    const std::vector<dmx::ElementPtr>* jointList = JointArray(model);

    const int boneCount = static_cast<int>(boneMap.transforms.size());
    for (int mdlBone = 0; mdlBone < boneCount; ++mdlBone) {
        const dmx::Element* transform = nullptr;
        const int dmeModelBone = boneMap.mdlToDmeModel[mdlBone];
        if (dmeModelBone < 0) {
            // no bind pose stored; use the joint's current transform
            transform = boneMap.transforms[mdlBone];
        } else if (bindTransforms && dmeModelBone < static_cast<int>(bindTransforms->size())) {
            transform = (*bindTransforms)[dmeModelBone];
        } else if (jointList && dmeModelBone < static_cast<int>(jointList->size())) {
            // no "bind" state: model's joint transform (jointTransforms)
            transform = JointTransform((*jointList)[dmeModelBone]);
        }

        if (transform) {
            pm::matrix3x4 m = TransformMatrix(transform);
            SrcBonePose& pose = anim.frames[0][mdlBone];
            pm::MatrixAngles(m, pose.rot, pose.pos);
            pose.pos.x *= flScale;
            pose.pos.y *= flScale;
            pose.pos.z *= flScale;
        }
    }

    out.anims.push_back(std::move(anim));

    // Build_Reference: source-local boneToPose from the BindPose frame
    out.boneToPose.resize(out.numbones);
    const SourceAnim& bp = out.anims.back();
    for (int i = 0; i < out.numbones; ++i) {
        pm::matrix3x4 m;
        pm::AngleMatrix(bp.frames[0][i].rot, bp.frames[0][i].pos, m);
        int parent = out.localBone[i].parent;
        if (parent == -1)
            out.boneToPose[i] = m;
        else
            out.boneToPose[i] = pm::ConcatTransforms(out.boneToPose[parent], m);
    }
}

// ---------------------------------------------------------------------------
// Animations (reference LoadAnimations/ComputeFramePose): sample every
// DmeChannelsClip at exact reference frame times; unchanneled transforms keep
// their static value.
// ---------------------------------------------------------------------------

// DmeTime helpers (tier1/timeutils facts): times are int ticks of 1/10000 s.
int RoundSecondsToTMS(float sec) {
    return static_cast<int>(floor(10000.0f * sec + 0.5f));
}
int CurrentFrameRoundDown(int tms, int fps) {
    // times within (frame*10000/fps - 1, frame*10000/fps] are on the frame
    long long prod = static_cast<long long>(tms) * fps;
    if (tms < 0)
        return static_cast<int>((prod - 10000 + fps) / 10000);
    return static_cast<int>((prod + fps) / 10000);
}

// Read a log layer's key times as ticks.
std::vector<int> ReadTimesTicks(const dmx::Element* layer) {
    std::vector<int> out;
    if (!layer) return out;
    if (auto ti = layer->GetIntArray("times")) {
        out.assign(ti->begin(), ti->end());
        return out;
    }
    if (const dmx::Attribute* a = layer->Get("times")) {
        if (auto tt = std::get_if<std::vector<dmx::Time>>(&a->value)) {
            out.reserve(tt->size());
            for (const dmx::Time& t : *tt)
                out.push_back(RoundSecondsToTMS(t.seconds));
        }
    }
    return out;
}

// last key with time <= t (reference CDmeLogLayer::FindKey)
int FindKey(const std::vector<int>& times, int t) {
    for (int i = static_cast<int>(times.size()) - 1; i >= 0; --i)
        if (times[i] <= t) return i;
    return -1;
}

struct SampledChannel {
    int bone = -1;
    bool isPos = false;
    std::vector<int> times;
    const std::vector<dmx::Vector3>* vpos = nullptr;
    const std::vector<dmx::Quaternion>* vrot = nullptr;

    // Reference CDmeTypedLogLayer::GetValue: no
    // exact-key early-out - a key hit interpolates with t=0. Interpolate<T>:
    // Vector is `t*b + (1-t)*a` (that op order);
    // Quaternion is QuaternionSlerp FOLLOWED BY QuaternionNormalize.
    pm::Vector3 SamplePos(int t, const pm::Vector3& def) const {
        if (!vpos || vpos->empty()) return def;
        int k = FindKey(times, t);
        if (k < 0) return ToV3((*vpos)[0]);
        if (k + 1 >= static_cast<int>(vpos->size()) || k + 1 >= static_cast<int>(times.size()))
            return ToV3((*vpos)[k]);
        float frac = static_cast<float>(t - times[k]) / static_cast<float>(times[k + 1] - times[k]);
        pm::Vector3 a = ToV3((*vpos)[k]);
        pm::Vector3 b = ToV3((*vpos)[k + 1]);
        return {frac * b.x + (1.0f - frac) * a.x, frac * b.y + (1.0f - frac) * a.y,
                frac * b.z + (1.0f - frac) * a.z};
    }
    pm::Quaternion SampleRot(int t, const pm::Quaternion& def) const {
        if (!vrot || vrot->empty()) return def;
        int k = FindKey(times, t);
        auto q = [&](int i) {
            const dmx::Quaternion& v = (*vrot)[i];
            return pm::Quaternion{v.x, v.y, v.z, v.w};
        };
        if (k < 0) return q(0);
        if (k + 1 >= static_cast<int>(vrot->size()) || k + 1 >= static_cast<int>(times.size()))
            return q(k);
        float frac = static_cast<float>(t - times[k]) / static_cast<float>(times[k + 1] - times[k]);
        pm::Quaternion r;
        pm::QuaternionSlerp(q(k), q(k + 1), frac, r);
        pm::QuaternionNormalize(r);
        return r;
    }
};

void LoadAnimations(const dmx::Element* animationList, float flScale,
                    const BoneMap& boneMap, Source& out) {
    auto clips = animationList->GetElementArray("animations");
    if (!clips) return;

    // transform element -> mdl bone
    std::map<const dmx::Element*, int> transformToBone;
    for (size_t i = 0; i < boneMap.transforms.size(); ++i)
        transformToBone[boneMap.transforms[i]] = static_cast<int>(i);

    for (const dmx::Element* clip : *clips) {
        if (!clip) continue;

        SourceAnim anim;
        anim.name = clip->name;

        int fps = clip->GetInt("frameRate", 0);
        if (fps <= 0) fps = 30;

        // clip start/end in ticks (timeFrame: start + duration)
        int startTms = 0;
        int durTms = 0;
        int offsetTms = 0;
        if (const dmx::Element* tf = clip->GetElement("timeFrame")) {
            auto readTicks = [&](const char* n) -> int {
                if (const dmx::Attribute* a = tf->Get(n)) {
                    if (auto t = std::get_if<dmx::Time>(&a->value))
                        return RoundSecondsToTMS(t->seconds);
                    if (auto i = std::get_if<int32_t>(&a->value)) return *i;
                }
                return 0;
            };
            startTms = readTicks("start");
            durTms = readTicks("duration");
            offsetTms = readTicks("offset");
        }
        int endTms = startTms + durTms;

        anim.startframe = CurrentFrameRoundDown(startTms, fps);
        anim.endframe = CurrentFrameRoundDown(endTms, fps);
        anim.numframes = anim.endframe - anim.startframe + 1;
        if (anim.numframes < 1) anim.numframes = 1;

        // resolve channels onto bones
        std::vector<SampledChannel> chans;
        if (auto channels = clip->GetElementArray("channels")) {
            for (const dmx::Element* c : *channels) {
                if (!c) continue;
                const dmx::Element* target = c->GetElement("toElement");
                const std::string* attr = c->GetString("toAttribute");
                if (!target || !attr) continue;
                auto it = transformToBone.find(target);
                if (it == transformToBone.end()) continue;
                const dmx::Element* log = c->GetElement("log");
                const dmx::Element* layer = nullptr;
                if (log)
                    if (auto layers = log->GetElementArray("layers"))
                        if (!layers->empty()) layer = (*layers)[0];
                if (!layer) continue;

                SampledChannel ch;
                ch.bone = it->second;
                ch.times = ReadTimesTicks(layer);
                if (*attr == "position") {
                    ch.isPos = true;
                    ch.vpos = layer->GetVector3Array("values");
                    if (!ch.vpos) continue;
                } else if (*attr == "orientation") {
                    ch.isPos = false;
                    ch.vrot = DmxQuatArray(layer, "values");
                    if (!ch.vrot) continue;
                } else {
                    continue;
                }
                chans.push_back(std::move(ch));
            }
        }

        // static defaults from each bone's transform element
        std::vector<pm::Vector3> defPos(boneMap.transforms.size());
        std::vector<pm::Quaternion> defRot(boneMap.transforms.size());
        for (size_t i = 0; i < boneMap.transforms.size(); ++i) {
            if (auto p = DmxVec3(boneMap.transforms[i], "position")) defPos[i] = ToV3(*p);
            if (auto q = DmxQuat(boneMap.transforms[i], "orientation"))
                defRot[i] = {q->x, q->y, q->z, q->w};
        }

        anim.frames.resize(anim.numframes);
        float flOOFrameRate = 1.0f / static_cast<float>(fps);
        for (int frame = 0; frame < anim.numframes; ++frame) {
            // exact reference frame-time construction
            int nSecond = frame / fps;
            int nFraction = frame - nSecond * fps;
            int t = startTms + nSecond * 10000 +
                    RoundSecondsToTMS(static_cast<float>(nFraction) * flOOFrameRate);
            // ToChildMediaTime: (t - timeFrame.start) + timeFrame.offset
            int childT = (t - startTms) + offsetTms;

            // current pose = static values overridden by channel samples
            std::vector<pm::Vector3> pos = defPos;
            std::vector<pm::Quaternion> rot = defRot;
            for (const SampledChannel& ch : chans) {
                if (ch.isPos)
                    pos[ch.bone] = ch.SamplePos(childT, defPos[ch.bone]);
                else
                    rot[ch.bone] = ch.SampleRot(childT, defRot[ch.bone]);
            }

            anim.frames[frame].resize(out.numbones); // zero-init, defaultRoot stays 0
            for (size_t i = 0; i < boneMap.transforms.size(); ++i) {
                pm::matrix3x4 m = pm::QuaternionMatrix(rot[i], pos[i]);
                SrcBonePose& p = anim.frames[frame][i];
                pm::MatrixAngles(m, p.rot, p.pos);
                p.pos.x *= flScale;
                p.pos.y *= flScale;
                p.pos.z *= flScale;
            }
        }

        out.anims.push_back(std::move(anim));
    }
}

// ---------------------------------------------------------------------------
// Mesh loading (reference LoadMeshes/LoadMesh/LoadVertices + UnifyIndices +
// BuildIndividualMeshes). Temp streams accumulate across every mesh dag of the
// source, then get unified + material-sorted.
// ---------------------------------------------------------------------------
struct TmpFace {
    int material;
    uint32_t a, na, ta;
    uint32_t b, nb, tb;
    uint32_t c, nc, tc;
};

struct MeshTemp {
    std::vector<pm::Vector3> vertex;   // transformed positions
    std::vector<SrcBoneWeight> bone;   // per position
    std::vector<pm::Vector3> normal;   // rotated normals
    std::vector<pm::Vector2> texcoord;
    std::vector<TmpFace> face;
};

// ---------------------------------------------------------------------------
// Flex/morph intermediates (reference statics s_DeltaStates /
// s_Balance / s_Speed / s_UniqueVertices / s_UniqueVerticesMap - reset per
// DMX in LoadModelAndSkeleton, here local to LoadDmxSource).
// ---------------------------------------------------------------------------
struct DeltaIndexEntry { // reference DeltaIndex_t
    int pos = -1;     // into DeltaTempState::posDeltas
    int normal = -1;  // into DeltaTempState::normalDeltas
    int wrinkle = -1; // into DeltaTempState::wrinkleDeltas
    int next = -1;    // linked list over unique vertices
    bool inList = false;
};

struct DeltaTempState { // reference DeltaState_t
    std::string name;
    std::vector<pm::Vector3> posDeltas;
    std::vector<pm::Vector3> normalDeltas;
    std::vector<float> wrinkleDeltas;
    std::vector<DeltaIndexEntry> indices; // parallel to FlexTemp::uniqueVertices
    int firstDelta = -1;
};

struct UniqueVert { // reference VertIndices_t (single UV channel)
    int v = -1, n = -1, t = -1; // global temp-stream indices
    int balance = 0, speed = 0; // into FlexTemp::balance / speed
};

struct FlexTemp {
    bool enabled = false; // rendermesh load - flex data is parsed at all
    std::vector<DeltaTempState> deltaStates;
    std::vector<float> balance; // FIELD_BALANCE data, 1.0f default per mesh
    std::vector<float> speed;   // FIELD_MORPH_SPEED data, 1.0f default per mesh
    std::vector<UniqueVert> uniqueVertices;
    std::vector<int> uniqueVerticesMap; // global corner index -> unique index
    // wrinkle generation input (combination op raw controls, parsed up front)
    struct WrinkleScale {
        std::string name;
        float scale = 0.0f;
    };
    std::vector<WrinkleScale> wrinkleScales;
};

// reference FindOrAddDeltaState (stricmp find, grow the parallel index array)
DeltaTempState* FindOrAddDeltaState(FlexTemp& flex, const std::string& name, size_t indexCount) {
    for (auto& ds : flex.deltaStates) {
        if (_stricmp(ds.name.c_str(), name.c_str()) == 0) {
            if (ds.indices.size() < indexCount)
                ds.indices.resize(indexCount);
            return &ds;
        }
    }
    flex.deltaStates.emplace_back();
    DeltaTempState& ds = flex.deltaStates.back();
    ds.name = name;
    ds.indices.resize(indexCount);
    return &ds;
}

// reference AddToDeltaList (list is PREPEND - traversal order is reverse
// first-touch order, and the written vanim order depends on it)
void AddToDeltaList(DeltaTempState& ds, int nUniqueVertex) {
    DeltaIndexEntry& e = ds.indices[nUniqueVertex];
    if (!e.inList) {
        e.next = ds.firstDelta;
        ds.firstDelta = nUniqueVertex;
        e.inList = true;
    }
}

// data index -> list of corner indices, ascending (reference
// CDmeVertexDataBase::FindVertexIndicesFromDataIndex inverse map)
std::vector<std::vector<int>> BuildInverseMap(const std::vector<int32_t>& indices,
                                              int dataCount) {
    std::vector<std::vector<int>> inv(dataCount);
    for (size_t i = 0; i < indices.size(); ++i) {
        int32_t idx = indices[i];
        if (idx >= 0 && idx < dataCount)
            inv[idx].push_back(static_cast<int>(i));
    }
    return inv;
}

} // namespace

// reference SortAndBalanceBones; declared in source.h
int SortAndBalanceBones(int iCount, int iMaxCount, int bones[], float weights[]) {
    int i;

    // collapse duplicate bone weights
    for (i = 0; i < iCount - 1; i++) {
        for (int j = i + 1; j < iCount; j++) {
            if (bones[i] == bones[j]) {
                weights[i] += weights[j];
                weights[j] = 0.0;
            }
        }
    }

    // do sleazy bubble sort
    int bShouldSort;
    do {
        bShouldSort = false;
        for (i = 0; i < iCount - 1; i++) {
            if (weights[i + 1] > weights[i]) {
                int j = bones[i + 1]; bones[i + 1] = bones[i]; bones[i] = j;
                float w = weights[i + 1]; weights[i + 1] = weights[i]; weights[i] = w;
                bShouldSort = true;
            }
        }
    } while (bShouldSort);

    // throw away insignificant weights
    while (iCount > 1 && weights[iCount - 1] < lim::kMinBoneWeight) {
        iCount--;
    }

    // clip to the top iMaxCount bones
    if (iCount > iMaxCount) {
        iCount = iMaxCount;
    }

    float t = 0;
    for (i = 0; i < iCount; i++) {
        t += weights[i];
    }
    if (t <= 0.0f) {
        // missing weights - evenly share
        t = 1.0f / iCount;
        for (i = 0; i < iCount; i++) {
            weights[i] = t;
        }
    } else {
        // scale to sum 1
        t = 1.0f / t;
        for (i = 0; i < iCount; i++) {
            weights[i] = weights[i] * t;
        }
    }
    return iCount;
}

namespace {

// reference CDmeMesh::CollapseRedundantNormals. The no-delta
// mesh takes the AGGRESSIVE pure-value dedup; a mesh with delta states takes
// the per-position collapse that produces a normalMap so the delta states'
// normal indices can be remapped consistently.

inline float NormalDot(const dmx::Vector3& a, const dmx::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// NormalizeNormals (Valve VectorNormalize semantics)
void NormalizeDmxNormals(std::vector<dmx::Vector3>& normals) {
    for (auto& n : normals) {
        pm::Vector3 v{n.x, n.y, n.z};
        pm::VectorNormalize(v);
        n = {v.x, v.y, v.z};
    }
}

// Uniform grid over the unit sphere for the aggressive collapse. Normals are unit
// (NormalizeDmxNormals ran) or zero, so dot > cos(2 deg) implies a chord under
// 2*sin(1 deg) = 0.0349; a cell edge of 0.04 means the 3x3x3 neighborhood is
// always a superset of the match set. A zero normal dots to 0 and never
// matches, so it needs no bound.
struct NormalGrid {
    static constexpr float kCell = 0.04f;
    // one axis spans [-1,1] -> cell in [-25,24]; a non-unit (zero) normal
    // clamps into the grid and still fails the dot test.
    static constexpr int kAxis = 50;
    std::vector<int> head = std::vector<int>(kAxis * kAxis * kAxis, -1);
    std::vector<int> next; // per stored id, chained within its cell

    static int Axis(float f) {
        int i = static_cast<int>(std::floor(f / kCell)) + kAxis / 2;
        return i < 0 ? 0 : (i >= kAxis ? kAxis - 1 : i);
    }
    static int CellOf(const dmx::Vector3& v) {
        return (Axis(v.x) * kAxis + Axis(v.y)) * kAxis + Axis(v.z);
    }
    void Add(const dmx::Vector3& v, int id) {
        if (static_cast<int>(next.size()) <= id)
            next.resize(id + 1, -1);
        const int c = CellOf(v);
        next[id] = head[c];
        head[c] = id;
    }

    // lowest matching index, which is what the linear scan's first hit was
    int FindFirst(const std::vector<dmx::Vector3>& data, const dmx::Vector3& v,
                  float flNormalBlend) const {
        const int cx = Axis(v.x), cy = Axis(v.y), cz = Axis(v.z);
        int best = -1;
        for (int x = cx - 1; x <= cx + 1; ++x) {
            if (x < 0 || x >= kAxis) continue;
            for (int y = cy - 1; y <= cy + 1; ++y) {
                if (y < 0 || y >= kAxis) continue;
                for (int z = cz - 1; z <= cz + 1; ++z) {
                    if (z < 0 || z >= kAxis) continue;
                    for (int id = head[(x * kAxis + y) * kAxis + z]; id != -1; id = next[id])
                        if ((best == -1 || id < best) &&
                            NormalDot(v, data[id]) > flNormalBlend)
                            best = id;
                }
            }
        }
        return best;
    }
};

// reference CollapseRedundantBaseNormalsAggressive
void CollapseBaseNormalsAggressive(std::vector<dmx::Vector3>& normals,
                                   std::vector<int32_t>& normalIndices,
                                   float flNormalBlend) {
    std::vector<int> normalMap(normals.size());
    std::vector<dmx::Vector3> newNormals;
    NormalGrid grid;
    for (size_t i = 0; i < normals.size(); ++i) {
        const dmx::Vector3& vNormal = normals[i];
        int j = grid.FindFirst(newNormals, vNormal, flNormalBlend);
        if (j != -1) {
            normalMap[i] = j;
            continue;
        }
        normalMap[i] = static_cast<int>(newNormals.size());
        grid.Add(vNormal, static_cast<int>(newNormals.size()));
        newNormals.push_back(vNormal);
    }

    if (newNormals.size() >= normals.size())
        return; // nothing collapsed

    for (auto& idx : normalIndices)
        idx = normalMap[idx];
    normals = std::move(newNormals);
}

// reference CollapseRedundantBaseNormals: merge normals
// that are similar AROUND EACH POSITION. Produces outNormalMap (old normal
// data index -> new) for the delta remap. Returns false (and changes nothing)
// if the result would not be smaller.
bool CollapseBaseNormalsWithMap(const std::vector<std::vector<int>>& posInverse,
                                std::vector<dmx::Vector3>& normals,
                                std::vector<int32_t>& normalIndices,
                                float flNormalBlend,
                                std::vector<int>& outNormalMap) {
    if (normals.empty() || normalIndices.empty())
        return false;

    const size_t nCorners = normalIndices.size();
    std::vector<int32_t> newNormalIndices(nCorners, -1);
    std::vector<dmx::Vector3> newNormalData;

    for (const std::vector<int>& corners : posInverse) {
        int nNewNormalDataIndex = static_cast<int>(newNormalData.size());
        for (int corner : corners) {
            if (corner < 0 || corner >= static_cast<int>(nCorners))
                continue;
            int32_t oldIdx = normalIndices[corner];
            if (oldIdx < 0 || oldIdx >= static_cast<int32_t>(normals.size()))
                continue;
            const dmx::Vector3& vNormal = normals[oldIdx];

            bool bUnique = true;
            for (int k = nNewNormalDataIndex; k < static_cast<int>(newNormalData.size()); ++k) {
                if (NormalDot(vNormal, newNormalData[k]) > flNormalBlend) {
                    newNormalIndices[corner] = k;
                    bUnique = false;
                    break;
                }
            }
            if (!bUnique)
                continue;
            newNormalIndices[corner] = static_cast<int32_t>(newNormalData.size());
            newNormalData.push_back(vNormal);
        }
    }

    // corners not reached through any position group keep their own value
    for (size_t i = 0; i < nCorners; ++i) {
        if (newNormalIndices[i] == -1) {
            newNormalIndices[i] = static_cast<int32_t>(newNormalData.size());
            newNormalData.push_back(normals[normalIndices[i]]);
        }
    }

    // if it's the same or more, don't do anything
    if (newNormalData.size() >= normals.size())
        return false;

    outNormalMap.assign(normals.size(), -1);
    for (size_t i = 0; i < nCorners; ++i) {
        if (outNormalMap[normalIndices[i]] == -1)
            outNormalMap[normalIndices[i]] = newNormalIndices[i];
    }

    normals = std::move(newNormalData);
    normalIndices = std::move(newNormalIndices);
    return true;
}

// reference CollapseRedundantDeltaNormals: remap a delta
// state's normal entries through the base normalMap, keeping the FIRST entry
// per new index.
void CollapseDeltaNormals(const std::vector<int>& normalMap,
                          std::vector<dmx::Vector3>& deltaNormals,
                          std::vector<int32_t>& deltaNormalIndices) {
    std::vector<bool> done(normalMap.size(), false);
    std::vector<dmx::Vector3> newData;
    std::vector<int32_t> newIndices;

    for (size_t i = 0; i < deltaNormalIndices.size(); ++i) {
        int32_t oldIdx = deltaNormalIndices[i];
        if (oldIdx < 0 || oldIdx >= static_cast<int32_t>(normalMap.size()))
            continue;
        int nNewIndex = normalMap[oldIdx];
        if (nNewIndex < 0 || done[nNewIndex])
            continue;
        done[nNewIndex] = true;
        newData.push_back(deltaNormals[i]);
        newIndices.push_back(nNewIndex);
    }

    deltaNormals = std::move(newData);
    deltaNormalIndices = std::move(newIndices);
}

// reference CDmeMesh::ComputeTriangulatedIndices: pick the fan corner with the
// smallest summed distance to the non-adjacent verts, fan from there.
void ComputeTriangulatedIndices(const std::vector<dmx::Vector3>& positions,
                                const std::vector<int32_t>& positionIndices,
                                const std::vector<int32_t>& faceCorners, int firstIndex,
                                int vertexCount, std::vector<int>& outIndices) {
    float flMinDistance = FLT_MAX;
    int nMinIndex = 0;

    int nLoopCount = vertexCount;
    if (vertexCount <= 3)
        nLoopCount = 0;
    else if (vertexCount == 4)
        nLoopCount = 2;

    auto positionOf = [&](int corner) -> pm::Vector3 {
        int c = faceCorners[corner];
        int pi = (c >= 0 && c < static_cast<int>(positionIndices.size())) ? positionIndices[c] : 0;
        if (pi < 0 || pi >= static_cast<int>(positions.size())) return {0, 0, 0};
        return ToV3(positions[pi]);
    };

    for (int i = 0; i < nLoopCount; ++i) {
        float flDistance = 0.0f;
        pm::Vector3 vecCenter = positionOf(firstIndex + i);
        for (int j = 2; j < vertexCount - 1; ++j) {
            int vi = (i + j) % vertexCount;
            pm::Vector3 vecEdge = positionOf(firstIndex + vi);
            float dx = vecEdge.x - vecCenter.x;
            float dy = vecEdge.y - vecCenter.y;
            float dz = vecEdge.z - vecCenter.z;
            flDistance += sqrtf(dx * dx + dy * dy + dz * dz);
        }
        if (flDistance < flMinDistance) {
            nMinIndex = i;
            flMinDistance = flDistance;
        }
    }

    outIndices.clear();
    for (int i = 1; i < vertexCount - 1; ++i) {
        outIndices.push_back(faceCorners[firstIndex + nMinIndex]);
        outIndices.push_back(faceCorners[firstIndex + ((nMinIndex + i) % vertexCount)]);
        outIndices.push_back(faceCorners[firstIndex + ((nMinIndex + i + 1) % vertexCount)]);
    }
}

struct LoadMeshInfo {
    Source* source = nullptr;
    const dmx::Element* model = nullptr;
    const std::vector<dmx::ElementPtr>* jointList = nullptr;
    float flScale = 1.0f;
    const BoneMap* boneMap = nullptr;
    MeshTemp* tmp = nullptr;
    MaterialTable* mats = nullptr;
    FlexTemp* flex = nullptr; // enabled only for rendermesh loads
    MeshFilter* filter = nullptr; // $exceptionlist, null when unfiltered
    std::vector<pm::matrix3x4> bindPose; // per jointList entry
};

// reference DefineUniqueVertices: register this mesh's
// corner tuples into the source-wide unique-vertex list. Offsets are the
// GLOBAL temp-stream starts captured before this mesh's data is appended;
// balance/speed bases are the array sizes before this mesh appends its data
// (or the single 1.0f default).
void DefineUniqueVertices(FlexTemp& flex,
                          const std::vector<int32_t>& positionIndices,
                          const std::vector<int32_t>& normalIndices,
                          const std::vector<int32_t>* texcoordIndices,
                          const std::vector<int32_t>* balanceIndices,
                          const std::vector<int32_t>* speedIndices,
                          int nStartingVertex, int nStartingNormal, int nStartingTexCoord) {
    const int nBalanceBase = static_cast<int>(flex.balance.size());
    const int nSpeedBase = static_cast<int>(flex.speed.size());
    const size_t nCorners = positionIndices.size();
    const bool bHasNormals = !normalIndices.empty();
    const bool bHasTexcoords = texcoordIndices && !texcoordIndices->empty();
    const bool bHasBalance = balanceIndices && !balanceIndices->empty();
    const bool bHasSpeed = speedIndices && !speedIndices->empty();

    // (v, n, t) -> unique index; first-seen insertion order matches the
    // reference's AddToTail order (the lookup only accelerates the find)
    struct KeyHash {
        size_t operator()(const std::tuple<int, int, int>& k) const {
            uint64_t h = 0xcbf29ce484222325ull;
            h = (h ^ static_cast<uint32_t>(std::get<0>(k))) * 0x100000001b3ull;
            h = (h ^ static_cast<uint32_t>(std::get<1>(k))) * 0x100000001b3ull;
            h = (h ^ static_cast<uint32_t>(std::get<2>(k))) * 0x100000001b3ull;
            return static_cast<size_t>(h ^ (h >> 32));
        }
    };
    std::unordered_map<std::tuple<int, int, int>, int, KeyHash> lookup;
    lookup.reserve(nCorners);

    for (size_t i = 0; i < nCorners; ++i) {
        UniqueVert vert;
        vert.v = nStartingVertex + positionIndices[i];
        vert.n = bHasNormals ? nStartingNormal + normalIndices[i] : -1;
        vert.t = bHasTexcoords ? nStartingTexCoord + (*texcoordIndices)[i] : -1;
        vert.balance = nBalanceBase + (bHasBalance ? (*balanceIndices)[i] : 0);
        vert.speed = nSpeedBase + (bHasSpeed ? (*speedIndices)[i] : 0);

        auto key = std::make_tuple(vert.v, vert.n, vert.t);
        auto it = lookup.find(key);
        if (it == lookup.end()) {
            int k = static_cast<int>(flex.uniqueVertices.size());
            flex.uniqueVertices.push_back(vert);
            flex.uniqueVerticesMap.push_back(k);
            lookup.emplace(key, k);
        } else {
            flex.uniqueVerticesMap.push_back(it->second);
        }
    }
}

// reference LoadVertices: positions/weights/normals/uvs into the temp streams
void LoadVertices(const LoadMeshInfo& info, const dmx::Element* dag, const dmx::Element* bindState,
                  const pm::matrix3x4& mat, int nBoneAssign,
                  const std::vector<dmx::Vector3>& normals /*post-collapse*/) {
    MeshTemp& tmp = *info.tmp;
    const BoneMap& boneMap = *info.boneMap;

    if (nBoneAssign < 0) {
        nBoneAssign = boneMap.defaultRootNode;
    } else {
        int remapped = (nBoneAssign < static_cast<int>(boneMap.dmeModelToMdl.size()))
                           ? boneMap.dmeModelToMdl[nBoneAssign] : -1;
        nBoneAssign = (remapped < 0) ? boneMap.defaultRootNode : remapped;
    }

    pm::matrix3x4 normalMat = pm::MatrixInverseTranspose(mat);

    const std::vector<dmx::Vector3>* positions = PickV3Array(bindState, "position$0", "positions");
    const std::vector<dmx::Vector2>* texcoords = bindState->GetVector2Array("texcoord$0");
    if (!texcoords) texcoords = bindState->GetVector2Array("textureCoordinates");
    if (!positions) return;

    int nCount = static_cast<int>(positions->size());

    // skinning
    const std::vector<float>* jw = PickFloatArray(bindState, "jointWeights", "blendweights$0");
    const std::vector<int32_t>* ji = PickIntArray(bindState, "jointIndices", "blendindices$0");
    int nJointCount = bindState->GetInt("jointCount", 0);
    if (!(jw && ji) || nJointCount <= 0) nJointCount = 0;
    if (nJointCount > lim::kMaxSrcBoneWeights) {
        std::fprintf(stderr, "warning: mesh \"%s\" has %d bone influences per vertex (max %d)\n",
                     dag->name.c_str(), nJointCount, lim::kMaxSrcBoneWeights);
    }

    if (nJointCount <= 0 && nBoneAssign == boneMap.defaultRootNode && dag) {
        // use the bone created for the DmeDag node of the DmeMesh (name lookup,
        // case-sensitive like the reference CUtlDict)
        for (size_t i = 0; i < info.source->localBone.size(); ++i) {
            if (info.source->localBone[i].name == dag->name) {
                nBoneAssign = static_cast<int>(i);
                break;
            }
        }
    }

    std::vector<float> weightBuf(nJointCount > 0 ? nJointCount : 1);
    std::vector<int> indexBuf(nJointCount > 0 ? nJointCount : 1);

    for (int i = 0; i < nCount; ++i) {
        // position transformed into bind space, then scaled
        pm::Vector3 p = pm::VectorTransform(ToV3((*positions)[i]), mat);
        p.x *= info.flScale;
        p.y *= info.flScale;
        p.z *= info.flScale;
        tmp.vertex.push_back(p);

        SrcBoneWeight w;
        if (nJointCount == 0) {
            w.numbones = 1;
            w.bone[0] = nBoneAssign;
            w.weight[0] = 1.0f;
        } else {
            for (int k = 0; k < nJointCount; ++k) {
                size_t idx = static_cast<size_t>(i) * nJointCount + k;
                weightBuf[k] = (idx < jw->size()) ? (*jw)[idx] : 0.0f;
                indexBuf[k] = (idx < ji->size()) ? (*ji)[idx] : 0;
            }
            int nBoneCount = SortAndBalanceBones(nJointCount, lim::kMaxSrcBoneWeights,
                                                 indexBuf.data(), weightBuf.data());
            w.numbones = nBoneCount;
            for (int j = 0; j < nBoneCount; ++j) {
                int nBoneIndex = (indexBuf[j] >= 0 &&
                                  indexBuf[j] < static_cast<int>(boneMap.dmeModelToMdl.size()))
                                     ? boneMap.dmeModelToMdl[indexBuf[j]] : -1;
                w.bone[j] = (nBoneIndex < 0) ? nBoneAssign : nBoneIndex;
                w.weight[j] = weightBuf[j];
            }
        }
        tmp.bone.push_back(w);
    }

    // normals: normalize (already done by collapse pass) then rotate
    for (const dmx::Vector3& nsrc : normals) {
        pm::Vector3 v = ToV3(nsrc);
        pm::VectorNormalize(v); // reference normalizes again inside LoadVertices
        tmp.normal.push_back(pm::VectorRotate(v, normalMat));
    }

    // texcoords with optional v flip
    bool bFlipV = bindState->GetBool("flipVCoordinates", false);
    if (texcoords) {
        for (const dmx::Vector2& t : *texcoords) {
            pm::Vector2 uv = ToV2(t);
            if (bFlipV) uv.y = 1.0f - uv.y;
            tmp.texcoord.push_back(uv);
        }
    }
}

// find the vertex-data bind state of a mesh
const dmx::Element* MeshBindState(const dmx::Element* mesh) {
    if (const dmx::Element* bs = mesh->GetElement("bindState")) return bs;
    if (auto states = mesh->GetElementArray("baseStates")) {
        for (const dmx::Element* s : *states)
            if (s && s->name == "bind") return s;
        if (!states->empty()) return (*states)[0];
    }
    return mesh->GetElement("currentState");
}

// Valve Vector::Length (float accumulate + sqrtf)
inline float DeltaLength(const dmx::Vector3& v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

// reference CDmeVertexDeltaData::GenerateWrinkleDelta
// with bOverwrite = false, bUseNormalForSign = false: wrinkle value per unique
// bind TEXCOORD data index = |posDelta| * scale / maxDeflection. Only called
// when the delta has no authored wrinkle field.
void GenerateWrinkleDelta(const std::vector<dmx::Vector3>& deltaPositions,
                          const std::vector<int32_t>& deltaPositionIndices,
                          float flScale,
                          const std::vector<std::vector<int>>& posInverse,
                          const std::vector<int32_t>& bindTexcoordIndices,
                          int nTexcoordDataCount,
                          std::vector<float>& outWrinkle,
                          std::vector<int32_t>& outWrinkleIndices) {
    outWrinkle.clear();
    outWrinkleIndices.clear();
    if (flScale == 0.0f || nTexcoordDataCount <= 0)
        return;

    float flMaxDeflection = 0.0f;
    for (const dmx::Vector3& p : deltaPositions) {
        float d = DeltaLength(p);
        if (d > flMaxDeflection)
            flMaxDeflection = d;
    }
    if (flMaxDeflection == 0.0f)
        return;

    const double scaledInverseMaxDeflection =
        static_cast<double>(flScale) / static_cast<double>(flMaxDeflection);

    std::vector<bool> used(nTexcoordDataCount, false);
    for (size_t i = 0; i < deltaPositions.size(); ++i) {
        float flWrinkleDelta = static_cast<float>(
            static_cast<double>(DeltaLength(deltaPositions[i])) * scaledInverseMaxDeflection);

        int32_t posIdx = deltaPositionIndices[i];
        if (posIdx < 0 || posIdx >= static_cast<int32_t>(posInverse.size()))
            continue;
        for (int corner : posInverse[posIdx]) {
            if (corner >= static_cast<int>(bindTexcoordIndices.size()))
                continue;
            int32_t t = bindTexcoordIndices[corner];
            if (t < 0 || t >= nTexcoordDataCount || used[t])
                continue;
            used[t] = true;
            outWrinkle.push_back(flWrinkleDelta);
            outWrinkleIndices.push_back(t);
        }
    }
}

// reference LoadDeltaState: move one delta state's
// pos/normal/wrinkle entries into the source-wide DeltaTempState, keyed by
// unique vertex index. Deltas are direction vectors: VectorRotate (not
// Transform), positions additionally scaled.
void LoadDeltaState(FlexTemp& flex, const std::string& deltaName,
                    const std::vector<dmx::Vector3>& positions,
                    const std::vector<int32_t>& positionIndices,
                    const std::vector<dmx::Vector3>& normals,
                    const std::vector<int32_t>& normalIndices,
                    const std::vector<float>& wrinkle,
                    const std::vector<int32_t>& wrinkleIndices,
                    const pm::matrix3x4& mat, const pm::matrix3x4& normalMat, float flScale,
                    const std::vector<std::vector<int>>& posInverse,
                    const std::vector<std::vector<int>>& normalInverse,
                    const std::vector<std::vector<int>>& texcoordInverse,
                    int nStartingUniqueVertexMap) {
    DeltaTempState* ds = FindOrAddDeltaState(flex, deltaName, flex.uniqueVertices.size());

    auto forEachUnique = [&](const std::vector<std::vector<int>>& inverse, int32_t dataIdx,
                             auto&& fn) {
        if (dataIdx < 0 || dataIdx >= static_cast<int32_t>(inverse.size()))
            return;
        for (int corner : inverse[dataIdx]) {
            size_t mapIdx = static_cast<size_t>(nStartingUniqueVertexMap) + corner;
            if (mapIdx >= flex.uniqueVerticesMap.size())
                continue;
            fn(flex.uniqueVerticesMap[mapIdx]);
        }
    };

    // position deltas
    for (size_t i = 0; i < positions.size() && i < positionIndices.size(); ++i) {
        pm::Vector3 vecDelta = pm::VectorRotate(ToV3(positions[i]), mat);
        vecDelta.x *= flScale;
        vecDelta.y *= flScale;
        vecDelta.z *= flScale;
        int nPositionIndex = static_cast<int>(ds->posDeltas.size());
        ds->posDeltas.push_back(vecDelta);
        forEachUnique(posInverse, positionIndices[i], [&](int unique) {
            AddToDeltaList(*ds, unique);
            ds->indices[unique].pos = nPositionIndex;
        });
    }

    // normal deltas (post-collapse arrays + post-collapse bind inverse)
    for (size_t i = 0; i < normals.size() && i < normalIndices.size(); ++i) {
        pm::Vector3 vecDelta = pm::VectorRotate(ToV3(normals[i]), normalMat);
        int nNormalIndex = static_cast<int>(ds->normalDeltas.size());
        ds->normalDeltas.push_back(vecDelta);
        forEachUnique(normalInverse, normalIndices[i], [&](int unique) {
            AddToDeltaList(*ds, unique);
            ds->indices[unique].normal = nNormalIndex;
        });
    }

    // wrinkle (indices are bind TEXCOORD data indices - FIELD_WRINKLE
    // redirects to FIELD_TEXCOORD in the bind state)
    for (size_t i = 0; i < wrinkle.size() && i < wrinkleIndices.size(); ++i) {
        int nWrinkleIndex = static_cast<int>(ds->wrinkleDeltas.size());
        ds->wrinkleDeltas.push_back(wrinkle[i]);
        forEachUnique(texcoordInverse, wrinkleIndices[i], [&](int unique) {
            AddToDeltaList(*ds, unique);
            ds->indices[unique].wrinkle = nWrinkleIndex;
        });
    }
}

bool LoadMesh(const LoadMeshInfo& info, const dmx::Element* dag, const dmx::Element* mesh,
              const dmx::Element* bindState, const pm::matrix3x4& mat, int nBoneAssign) {
    MeshTemp& tmp = *info.tmp;

    int nStartingVertex = static_cast<int>(tmp.vertex.size());
    int nStartingNormal = static_cast<int>(tmp.normal.size());
    int nStartingTexCoord = static_cast<int>(tmp.texcoord.size());

    const std::vector<dmx::Vector3>* positions = PickV3Array(bindState, "position$0", "positions");
    const std::vector<int32_t>* positionIndices =
        PickIntArray(bindState, "position$0Indices", "positionsIndices");
    const std::vector<int32_t>* texcoordIndices =
        PickIntArray(bindState, "texcoord$0Indices", "textureCoordinatesIndices");

    // CollapseRedundantNormals mutates the (copied) normal stream + indices.
    // Meshes with delta states take the per-position variant that yields a
    // normalMap for remapping the delta states' normal entries.
    std::vector<dmx::Vector3> normals;
    std::vector<int32_t> normalIndices;
    {
    if (auto n = PickV3Array(bindState, "normal$0", "normals"))
        normals = *n;
    if (auto ni = PickIntArray(bindState, "normal$0Indices", "normalsIndices"))
        normalIndices = *ni;
    }


    const std::vector<dmx::ElementPtr>* deltaStates = mesh->GetElementArray("deltaStates");
    bool hasDeltas = deltaStates && !deltaStates->empty();

    const float flNormalBlend = static_cast<float>(cos(2.0f * pm::kDeg2Rad));
    { PULSE_PERF("mesh", "NormalizeDmxNormals"); NormalizeDmxNormals(normals); }

    // position-data inverse map (corner lists per position data index)
    // only the delta-state paths below read this, and it is one heap vector
    // per position - do not pay for it on a mesh without morphs
    std::vector<std::vector<int>> posInverse;
    if (hasDeltas && positions && positionIndices)
        posInverse = BuildInverseMap(*positionIndices, static_cast<int>(positions->size()));

    std::vector<int> normalMap; // old normal data index -> new (delta collapse)
    bool bNormalsCollapsed = false;
    if (!hasDeltas) {
        PULSE_PERF("mesh", "CollapseBaseNormals");
        CollapseBaseNormalsAggressive(normals, normalIndices, flNormalBlend);
    } else {
        bNormalsCollapsed =
            CollapseBaseNormalsWithMap(posInverse, normals, normalIndices, flNormalBlend, normalMap);
    }

    // flex bookkeeping (rendermesh loads only)
    FlexTemp* flex = info.flex;
    int nStartingUniqueVertexMap = flex ? static_cast<int>(flex->uniqueVerticesMap.size()) : 0;
    if (flex && positionIndices) {
        const std::vector<int32_t>* balanceIndices =
            PickIntArray(bindState, "balance$0Indices", "balanceIndices");
        const std::vector<int32_t>* speedIndices =
            PickIntArray(bindState, "speed$0Indices", "speedIndices");
        PULSE_PERF("mesh", "DefineUniqueVertices");
        DefineUniqueVertices(*flex, *positionIndices, normalIndices, texcoordIndices,
                             balanceIndices, speedIndices, nStartingVertex, nStartingNormal,
                             nStartingTexCoord);
    }

    { PULSE_PERF("mesh", "LoadVertices");
      LoadVertices(info, dag, bindState, mat, nBoneAssign, normals); }

    // balance/speed data follows the mesh's vertices (reference LoadVertices
    // tail: whole array, or a single 1.0f so the 0-index default hits it)
    if (flex) {
        const std::vector<float>* balances = PickFloatArray(bindState, "balance$0", "balance");
        const std::vector<float>* speeds = PickFloatArray(bindState, "speed$0", "speed");
        if (balances && !balances->empty())
            flex->balance.insert(flex->balance.end(), balances->begin(), balances->end());
        else
            flex->balance.push_back(1.0f);
        if (speeds && !speeds->empty())
            flex->speed.insert(flex->speed.end(), speeds->begin(), speeds->end());
        else
            flex->speed.push_back(1.0f);
    }

    // load the delta states (reference LoadMesh tail)
    if (flex && hasDeltas && positions && positionIndices) {
        pm::matrix3x4 normalMat = pm::MatrixInverseTranspose(mat);

        std::vector<std::vector<int>> normalInverse =
            BuildInverseMap(normalIndices, static_cast<int>(normals.size()));
        int nTexcoordDataCount = 0;
        {
            const std::vector<dmx::Vector2>* tex = bindState->GetVector2Array("texcoord$0");
            if (!tex) tex = bindState->GetVector2Array("textureCoordinates");
            if (tex) nTexcoordDataCount = static_cast<int>(tex->size());
        }
        std::vector<std::vector<int>> texcoordInverse;
        if (texcoordIndices)
            texcoordInverse = BuildInverseMap(*texcoordIndices, nTexcoordDataCount);

        for (const dmx::Element* delta : *deltaStates) {
            if (!delta) continue;

            std::vector<dmx::Vector3> dPos;
            if (auto p = PickV3Array(delta, "position$0", "positions"))
                dPos = *p;
            std::vector<int32_t> dPosIdx;
            if (auto pi = PickIntArray(delta, "position$0Indices", "positionsIndices"))
                dPosIdx = *pi;

            std::vector<dmx::Vector3> dNormals;
            if (auto n = PickV3Array(delta, "normal$0", "normals"))
                dNormals = *n;
            std::vector<int32_t> dNormalIdx;
            if (auto ni = PickIntArray(delta, "normal$0Indices", "normalsIndices"))
                dNormalIdx = *ni;
            if (bNormalsCollapsed)
                CollapseDeltaNormals(normalMap, dNormals, dNormalIdx);

            // wrinkle: authored field wins (GenerateWrinkleDeltas is called
            // with bOverwrite = false); otherwise generate from the raw
            // control's wrinkleScale when nonzero
            std::vector<float> dWrinkle;
            std::vector<int32_t> dWrinkleIdx;
            const FlexTemp::WrinkleScale* ws = nullptr;
            for (const FlexTemp::WrinkleScale& w : flex->wrinkleScales) {
                if (_stricmp(w.name.c_str(), delta->name.c_str()) == 0) {
                    ws = &w;
                    break;
                }
            }
            const std::vector<float>* aw = PickFloatArray(delta, "wrinkle$0", "wrinkle");
            if (aw) {
                dWrinkle = *aw;
                if (auto wi = PickIntArray(delta, "wrinkle$0Indices", "wrinkleIndices"))
                    dWrinkleIdx = *wi;
            } else if (ws) {
                GenerateWrinkleDelta(dPos, dPosIdx, ws->scale, posInverse,
                                     texcoordIndices ? *texcoordIndices : std::vector<int32_t>{},
                                     nTexcoordDataCount, dWrinkle, dWrinkleIdx);
            }

            LoadDeltaState(*flex, delta->name, dPos, dPosIdx, dNormals, dNormalIdx, dWrinkle,
                           dWrinkleIdx, mat, normalMat, info.flScale, posInverse, normalInverse,
                           texcoordInverse, nStartingUniqueVertexMap);
        }
    }

    if (!positions || !positionIndices) return true;

    auto faceSets = mesh->GetElementArray("faceSets");
    if (!faceSets) return true;
    PULSE_PERF("mesh", "faceSets");

    for (const dmx::Element* faceSet : *faceSets) {
        if (!faceSet) continue;
        const std::vector<int32_t>* faces = faceSet->GetIntArray("faces");
        if (!faces) continue;

        // material name
        std::string textureName;
        if (const dmx::Element* material = faceSet->GetElement("material")) {
            if (auto s = material->GetString("mtlName"))
                textureName = *s;
            else
                textureName = material->name;
        }

        // skip faces with the null texture
        {
            std::string noExt = textureName;
            size_t dot = noExt.find_last_of('.');
            size_t slash = noExt.find_last_of("/\\");
            if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
                noExt.resize(dot);
            if (_stricmp(noExt.c_str(), "null") == 0)
                continue;
        }

        int texture = info.mats->LookupTexture(textureName.c_str());
        int material = info.mats->UseTextureAsMaterial(texture);

        // walk polygons (index list with -1 terminators)
        int nFirstIndex = 0;
        int nIndexCount = static_cast<int>(faces->size());
        std::vector<int> triangulated;
        while (nFirstIndex < nIndexCount) {
            // GetNextPolygonVertexCount
            int nVertexCount = 0;
            while (nFirstIndex + nVertexCount < nIndexCount &&
                   (*faces)[nFirstIndex + nVertexCount] != -1)
                ++nVertexCount;

            if (nVertexCount >= 3) {
                ComputeTriangulatedIndices(*positions, *positionIndices, *faces, nFirstIndex,
                                           nVertexCount, triangulated);
                for (size_t ii = 0; ii + 2 < triangulated.size() + 1 && ii < triangulated.size();
                     ii += 3) {
                    // reference winding: (v1, v3, v2)
                    int corners[3] = {triangulated[ii], triangulated[ii + 2], triangulated[ii + 1]};
                    TmpFace f{};
                    f.material = material;
                    uint32_t* fv[3] = {&f.a, &f.b, &f.c};
                    uint32_t* fn[3] = {&f.na, &f.nb, &f.nc};
                    uint32_t* ft[3] = {&f.ta, &f.tb, &f.tc};
                    for (int cix = 0; cix < 3; ++cix) {
                        int corner = corners[cix];
                        int p = (corner < static_cast<int>(positionIndices->size()))
                                    ? (*positionIndices)[corner] : -1;
                        int n = (corner < static_cast<int>(normalIndices.size()))
                                    ? normalIndices[corner] : -1;
                        int t = (texcoordIndices && corner < static_cast<int>(texcoordIndices->size()))
                                    ? (*texcoordIndices)[corner] : -1;
                        *fv[cix] = (p >= 0) ? static_cast<uint32_t>(nStartingVertex + p) : 0;
                        *fn[cix] = (n >= 0) ? static_cast<uint32_t>(nStartingNormal + n) : 0;
                        *ft[cix] = (t >= 0) ? static_cast<uint32_t>(nStartingTexCoord + t) : 0;
                    }
                    tmp.face.push_back(f);
                }
            }
            nFirstIndex += nVertexCount + 1;
        }
    }

    return true;
}

// reference LoadMeshes recursion: aggregate dag-to-bind-pose transform
bool LoadMeshesRecursive(const LoadMeshInfo& info, const dmx::Element* dag,
                         const pm::matrix3x4& parentToBindPose, int nBoneAssign) {
    pm::matrix3x4 dagToBindPose;
    int nFoundIndex = GetJointIndex(info.jointList, dag);
    if (nFoundIndex >= 0)
        nBoneAssign = nFoundIndex;

    if (nFoundIndex >= 0 && nFoundIndex < static_cast<int>(info.bindPose.size())) {
        dagToBindPose = pm::ConcatTransforms(parentToBindPose, info.bindPose[nFoundIndex]);
    } else {
        // current pose fallback (no bind transform for this dag)
        const dmx::Element* t = dag->GetElement("transform");
        dagToBindPose = pm::ConcatTransforms(parentToBindPose, TransformMatrix(t));
    }

    const dmx::Element* shape = dag->GetElement("shape");
    // an $exceptionlist rejection skips the shape only - the dag still becomes a
    // localBone, exactly as it would for a dag carrying no mesh at all
    if (shape && shape->className == "DmeMesh" &&
        (!info.filter || info.filter->Keep(shape->name, dag->name))) {
        const dmx::Element* bindState = MeshBindState(shape);
        if (!bindState)
            return false;
        if (!LoadMesh(info, dag, shape, bindState, dagToBindPose, nBoneAssign))
            return false;
    }

    if (auto kids = dag->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c && !LoadMeshesRecursive(info, c, dagToBindPose, nBoneAssign))
                return false;
    return true;
}

bool LoadMeshes(const dmx::Element* model, float flScale, const BoneMap& boneMap,
                MaterialTable& mats, MeshTemp& tmp, FlexTemp* flex, MeshFilter* filter,
                Source& out) {
    LoadMeshInfo info;
    info.filter = filter;
    info.source = &out;
    info.model = model;
    info.jointList = JointArray(model);
    info.flScale = flScale;
    info.boneMap = &boneMap;
    info.tmp = &tmp;
    info.mats = &mats;
    info.flex = flex;

    // bind pose transforms per jointList entry ("bind" base state, else joint transforms)
    const dmx::Element* bindPose = FindBaseState(model, "bind");
    const std::vector<dmx::ElementPtr>* bindTransforms =
        bindPose ? bindPose->GetElementArray("transforms") : nullptr;
    int nCount = bindTransforms ? static_cast<int>(bindTransforms->size())
                                : (info.jointList ? static_cast<int>(info.jointList->size()) : 0);
    info.bindPose.resize(nCount);
    for (int i = 0; i < nCount; ++i) {
        const dmx::Element* t = nullptr;
        if (bindTransforms) {
            t = (*bindTransforms)[i];
        } else if (info.jointList) {
            t = JointTransform((*info.jointList)[i]);
        }
        info.bindPose[i] = t ? TransformMatrix(t) : pm::matrix3x4{};
    }

    pm::matrix3x4 identity;
    if (auto kids = model->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c && !LoadMeshesRecursive(info, c, identity, -1))
                return false;
    return true;
}

// ---------------------------------------------------------------------------
// UnifyIndices + BuildIndividualMeshes
// ---------------------------------------------------------------------------
struct VUnify {
    int refcount = 0;
    int lastref = -1;
    int v = 0, m = 0, n = 0;
    uint32_t t = 0;
    int next = -1; // chained per position
};

struct UnifyState {
    std::vector<VUnify> listdata;
    std::vector<int> vlist; // per position: head of chain, -1 none
};

int AddToVlist(UnifyState& st, int v, int m, int n, uint32_t t) {
    int prev = -1;
    int cur = st.vlist[v];
    while (cur != -1) {
        VUnify& u = st.listdata[cur];
        if (u.m == m && u.n == n && u.t == t) {
            u.refcount++;
            return cur;
        }
        prev = cur;
        cur = u.next;
    }
    int idx = static_cast<int>(st.listdata.size());
    VUnify u;
    u.refcount = 1;
    u.v = v;
    u.m = m;
    u.n = n;
    u.t = t;
    st.listdata.push_back(u);
    if (prev != -1)
        st.listdata[prev].next = idx;
    else
        st.vlist[v] = idx;
    return idx;
}

// qsort comparators (C qsort on purpose - reference uses libc qsort and the
// sort is unstable; matching the implementation matches the order)
const std::vector<VUnify>* g_sortListData = nullptr;
const std::vector<TmpFace>* g_sortFaces = nullptr;

int VlistCompare(const void* elem1, const void* elem2) {
    const VUnify& u1 = (*g_sortListData)[*static_cast<const int*>(elem1)];
    const VUnify& u2 = (*g_sortListData)[*static_cast<const int*>(elem2)];
    if (u1.m < u2.m) return -1;
    if (u1.m > u2.m) return 1;
    if (u1.lastref < u2.lastref) return -1;
    if (u1.lastref > u2.lastref) return 1;
    return 0;
}

int FaceCompare(const void* elem1, const void* elem2) {
    int i1 = *static_cast<const int*>(elem1);
    int i2 = *static_cast<const int*>(elem2);
    if ((*g_sortFaces)[i1].material < (*g_sortFaces)[i2].material) return -1;
    if ((*g_sortFaces)[i1].material > (*g_sortFaces)[i2].material) return 1;
    if (i1 < i2) return -1;
    if (i1 > i2) return 1;
    return 0;
}

// reference CalcTriangleTangentSpace (mathlib)
void CalcTriangleTangentSpace(const pm::Vector3& p0, const pm::Vector3& p1, const pm::Vector3& p2,
                              const pm::Vector2& t0, const pm::Vector2& t1, const pm::Vector2& t2,
                              pm::Vector3& sVect, pm::Vector3& tVect) {
    const double SMALL_FLOAT = 1e-12;

    sVect = {0.0f, 0.0f, 0.0f};
    tVect = {0.0f, 0.0f, 0.0f};

    auto cross = [](const pm::Vector3& a, const pm::Vector3& b) {
        return pm::Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };

    // x, s, t
    pm::Vector3 edge01{p1.x - p0.x, t1.x - t0.x, t1.y - t0.y};
    pm::Vector3 edge02{p2.x - p0.x, t2.x - t0.x, t2.y - t0.y};
    pm::Vector3 c = cross(edge01, edge02);
    if (fabs(c.x) > SMALL_FLOAT) {
        sVect.x += -c.y / c.x;
        tVect.x += -c.z / c.x;
    }
    // y, s, t
    edge01 = {p1.y - p0.y, t1.x - t0.x, t1.y - t0.y};
    edge02 = {p2.y - p0.y, t2.x - t0.x, t2.y - t0.y};
    c = cross(edge01, edge02);
    if (fabs(c.x) > SMALL_FLOAT) {
        sVect.y += -c.y / c.x;
        tVect.y += -c.z / c.x;
    }
    // z, s, t
    edge01 = {p1.z - p0.z, t1.x - t0.x, t1.y - t0.y};
    edge02 = {p2.z - p0.z, t2.x - t0.x, t2.y - t0.y};
    c = cross(edge01, edge02);
    if (fabs(c.x) > SMALL_FLOAT) {
        sVect.z += -c.y / c.x;
        tVect.z += -c.z / c.x;
    }

    // normalize sVect and tVect (reference does this per face)
    pm::VectorNormalize(sVect);
    pm::VectorNormalize(tVect);
}

} // namespace (anon) - CalcModelTangentSpaces is exposed (source.h) for the
  // SMD loader; it still uses the anon-namespace CalcTriangleTangentSpace above.

// reference CalcModelTangentSpaces
void CalcModelTangentSpaces(Source& src) {
    for (int meshID = 0; meshID < src.nummeshes; meshID++) {
        SrcMesh* pMesh = &src.mesh[src.meshindex[meshID]];
        // vert -> face lists, CSR (counts, prefix sum, fill) rather than a
        // vector per vertex: same ascending-faceID order, one allocation.
        // REFERENCE QUIRK (parity): s_face_t.d is 0 for triangles (calloc),
        // but the quad check is `d != 0xFFFFFFFF` - so EVERY triangle face is
        // also appended to vertex 0's map, skewing its tangent.
        std::vector<int> mapStart(static_cast<size_t>(pMesh->numvertices) + 1, 0);
        std::vector<int> mapFaces;
        {
            for (int faceID = 0; faceID < pMesh->numfaces; faceID++) {
                const SrcFace* pFace = &src.face[faceID + pMesh->faceoffset];
                mapStart[pFace->a + 1]++;
                mapStart[pFace->b + 1]++;
                mapStart[pFace->c + 1]++;
                mapStart[1]++;
            }
            for (int i = 0; i < pMesh->numvertices; i++)
                mapStart[i + 1] += mapStart[i];
            mapFaces.resize(mapStart[pMesh->numvertices]);
            std::vector<int> cursor(mapStart.begin(), mapStart.end() - 1);
            for (int faceID = 0; faceID < pMesh->numfaces; faceID++) {
                const SrcFace* pFace = &src.face[faceID + pMesh->faceoffset];
                mapFaces[cursor[pFace->a]++] = faceID;
                mapFaces[cursor[pFace->b]++] = faceID;
                mapFaces[cursor[pFace->c]++] = faceID;
                mapFaces[cursor[0]++] = faceID;
            }
        }

        std::vector<pm::Vector3> faceSVect(pMesh->numfaces);
        std::vector<pm::Vector3> faceTVect(pMesh->numfaces);
        for (int faceID = 0; faceID < pMesh->numfaces; faceID++) {
            SrcFace* pFace = &src.face[faceID + pMesh->faceoffset];
            const SrcVertex& v1 = src.vertex[pMesh->vertexoffset + pFace->a];
            const SrcVertex& v2 = src.vertex[pMesh->vertexoffset + pFace->b];
            const SrcVertex& v3 = src.vertex[pMesh->vertexoffset + pFace->c];
            CalcTriangleTangentSpace(v1.position, v2.position, v3.position, v1.texcoord,
                                     v2.texcoord, v3.texcoord, faceSVect[faceID],
                                     faceTVect[faceID]);
        }

        for (int vertID = 0; vertID < pMesh->numvertices; vertID++) {
            SrcVertex& vert = src.vertex[vertID + pMesh->vertexoffset];
            const pm::Vector3& normal = vert.normal;
            pm::Vector3 sVect{0, 0, 0}, tVect{0, 0, 0};
            for (int fi = mapStart[vertID]; fi < mapStart[vertID + 1]; fi++) {
                const int f = mapFaces[fi];
                sVect.x += faceSVect[f].x; sVect.y += faceSVect[f].y; sVect.z += faceSVect[f].z;
                tVect.x += faceTVect[f].x; tVect.y += faceTVect[f].y; tVect.z += faceTVect[f].z;
            }

            auto cross = [](const pm::Vector3& a, const pm::Vector3& b) {
                return pm::Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                                   a.x * b.y - a.y * b.x};
            };
            pm::Vector3 tmpVect = cross(sVect, tVect);
            bool leftHanded = (tmpVect.x * normal.x + tmpVect.y * normal.y +
                               tmpVect.z * normal.z) < 0.0f;
            if (!leftHanded) {
                tVect = cross(normal, sVect);
                sVect = cross(tVect, normal);
                pm::VectorNormalize(sVect);
                pm::VectorNormalize(tVect);
                vert.tangentS = {sVect.x, sVect.y, sVect.z, 1.0f};
            } else {
                tVect = cross(sVect, normal);
                sVect = cross(normal, tVect);
                pm::VectorNormalize(sVect);
                pm::VectorNormalize(tVect);
                vert.tangentS = {sVect.x, sVect.y, sVect.z, -1.0f};
            }
        }
    }
}

namespace { // reopen anon namespace for the rest of the loader internals

// reference BuildVertexAnimations: each delta state
// becomes one 1-frame morph anim. Runs AFTER UnifyIndices (needs the vlist)
// and BEFORE the material sort - vanim.vertex is the CREATION-ORDER listdata
// index, remapped through v_ilistsort by BuildIndividualMeshes like faces are.
void BuildVertexAnimations(const FlexTemp& flex, const UnifyState& st, Source& out) {
    if (flex.deltaStates.empty())
        return;

    for (const DeltaTempState& state : flex.deltaStates) {
        SrcMorphAnim morph;
        morph.name = state.name;

        // traverse the linked list of unique vertices that have a delta
        // (prepend order - matches the reference's written vanim order)
        for (int j = state.firstDelta; j >= 0; j = state.indices[j].next) {
            const DeltaIndexEntry& delta = state.indices[j];
            const UniqueVert& uniqueVert = flex.uniqueVertices[j];

            if (uniqueVert.v < 0 || uniqueVert.v >= static_cast<int>(st.vlist.size()))
                continue;
            // every unified vertex built on this position that matches the
            // unique vert's (normal, texcoord) gets the delta - materials
            // duplicate verts, so one delta can fan out to several
            for (int cur = st.vlist[uniqueVert.v]; cur != -1; cur = st.listdata[cur].next) {
                const VUnify& u = st.listdata[cur];
                if (u.n != uniqueVert.n || static_cast<int>(u.t) != uniqueVert.t)
                    continue;

                SrcVertAnim va;
                va.vertex = cur;
                va.speed = flex.speed[uniqueVert.speed];
                va.side = flex.balance[uniqueVert.balance];
                if (delta.pos >= 0)
                    va.pos = state.posDeltas[delta.pos];
                if (delta.normal >= 0)
                    va.normal = state.normalDeltas[delta.normal];
                if (delta.wrinkle >= 0)
                    va.wrinkle = state.wrinkleDeltas[delta.wrinkle];
                morph.vanims.push_back(va);
            }
        }

        out.morphs.push_back(std::move(morph));
    }
}

void BuildIndividualMeshes(const MeshTemp& tmp, FlexTemp* flex, Source& out) {
    // UnifyIndices
    UnifyState st;
    st.vlist.assign(tmp.vertex.size(), -1);
    struct UFace { int a, b, c; };
    std::vector<UFace> ufaces(tmp.face.size());
    { PULSE_PERF("build", "UnifyIndices");
    for (size_t i = 0; i < tmp.face.size(); ++i) {
        const TmpFace& f = tmp.face[i];
        ufaces[i].a = AddToVlist(st, f.a, f.material, f.na, f.ta);
        ufaces[i].b = AddToVlist(st, f.b, f.material, f.nb, f.tb);
        ufaces[i].c = AddToVlist(st, f.c, f.material, f.nc, f.tc);
    } }

    // reference order: UnifyIndices -> BuildVertexAnimations -> sort/remap
    if (flex)
        BuildVertexAnimations(*flex, st, out);

    int numvlist = static_cast<int>(st.listdata.size());
    int numfaces = static_cast<int>(tmp.face.size());

    // SortVerticesByMaterial / SortFacesByMaterial (C qsort)
    std::vector<int> v_listsort(numvlist);
    std::vector<int> v_ilistsort(numvlist);
    std::vector<int> facesort(numfaces);
    for (int i = 0; i < numvlist; i++) v_listsort[i] = i;
    for (int i = 0; i < numfaces; i++) facesort[i] = i;
    g_sortListData = &st.listdata;
    g_sortFaces = &tmp.face;
    { PULSE_PERF("build", "sort verts/faces");
    if (numvlist > 0) qsort(v_listsort.data(), numvlist, sizeof(int), VlistCompare);
    if (numfaces > 0) qsort(facesort.data(), numfaces, sizeof(int), FaceCompare); }
    g_sortListData = nullptr;
    g_sortFaces = nullptr;
    for (int i = 0; i < numvlist; i++) v_ilistsort[v_listsort[i]] = i;

    // BuildUniqueVertexList
    out.vertex.resize(numvlist);
    for (int i = 0; i < numvlist; i++) {
        const VUnify& u = st.listdata[v_listsort[i]];
        SrcVertex& vertex = out.vertex[i];
        vertex.position = tmp.vertex[u.v];
        vertex.normal = (u.n >= 0 && u.n < static_cast<int>(tmp.normal.size()))
                            ? tmp.normal[u.n] : pm::Vector3{};
        vertex.boneweight = tmp.bone[u.v];
        vertex.texcoord = (u.t < tmp.texcoord.size()) ? tmp.texcoord[u.t] : pm::Vector2{};
        vertex.material = u.m;
    }

    // PointMeshesToVertexAndFaceData
    out.mesh.assign(lim::kMaxSkins, SrcMesh{});
    out.meshindex.assign(lim::kMaxSkins, 0);
    for (int m = 0; m < lim::kMaxSkins; m++) {
        out.mesh[m].vertexoffset = numvlist;
        out.mesh[m].faceoffset = numfaces;
    }
    for (int i = 0; i < numvlist; i++) {
        int m = out.vertex[i].material;
        out.mesh[m].numvertices++;
        if (out.mesh[m].vertexoffset > i)
            out.mesh[m].vertexoffset = i;
    }
    for (int i = 0; i < numfaces; i++) {
        int m = tmp.face[facesort[i]].material;
        out.mesh[m].numfaces++;
        if (out.mesh[m].faceoffset > i)
            out.mesh[m].faceoffset = i;
    }

    // BuildFaceList
    out.face.resize(numfaces);
    out.nummeshes = 0;
    for (int m = 0; m < lim::kMaxSkins; m++) {
        if (!out.mesh[m].numfaces)
            continue;
        out.meshindex[out.nummeshes++] = m;
        for (int i = out.mesh[m].faceoffset; i < out.mesh[m].numfaces + out.mesh[m].faceoffset;
             i++) {
            int j = facesort[i];
            out.face[i].a = static_cast<uint32_t>(v_ilistsort[ufaces[j].a] - out.mesh[m].vertexoffset);
            out.face[i].b = static_cast<uint32_t>(v_ilistsort[ufaces[j].b] - out.mesh[m].vertexoffset);
            out.face[i].c = static_cast<uint32_t>(v_ilistsort[ufaces[j].c] - out.mesh[m].vertexoffset);
        }
    }

    // RemapVertexAnimations: vanims are MODEL relative -
    // translate creation-order listdata indices to the sorted vertex order
    for (SrcMorphAnim& morph : out.morphs) {
        for (SrcVertAnim& va : morph.vanims) {
            if (va.vertex >= 0 && va.vertex < numvlist)
                va.vertex = v_ilistsort[va.vertex];
        }
    }

    { PULSE_PERF("build", "CalcModelTangentSpaces"); CalcModelTangentSpaces(out); }
}

// ---------------------------------------------------------------------------
// Combination operator (reference CDmeCombinationOperator + AddCombination /
// BuildCombinationSourceData). The KV2 layout:
//   root."combinationOperator" -> DmeCombinationOperator
//     "controls"   element_array of DmeCombinationInputControl:
//                  "rawControlNames" str[], "stereo" bool, "eyelid" bool,
//                  "flexgroup" str, "wrinkleScales" float[] (padded to raw
//                  count with 0), optional "flexMin"/"flexMax" floats,
//                  optional "eyesUpDownFlex" str (eyelid)
//     "dominators" element_array of DmeCombinationDominationRule:
//                  "dominators" str[], "suppressed" str[]
//     "targets"    element_array: DmeMesh(es) + DmeFlexRules
// The operation list is COMPUTED from delta state names: "A_B" combines raw
// controls A and B (reference ParseDeltaName).
// ---------------------------------------------------------------------------
struct ComboControlTemp {
    std::string name;
    std::vector<std::string> rawControls;
    bool stereo = false;
    bool eyelid = false;
    std::string flexgroup;
    std::string eyesUpDownFlexName;
    bool hasMinMax = false;
    float flexMin = 0.0f;
    float flexMax = 1.0f;
};

struct ComboTemp {
    std::vector<ComboControlTemp> controls;
    struct RawControl {
        std::string name;
        int inputControl = 0;
        float wrinkleScale = 0.0f;
    };
    std::vector<RawControl> raws; // flattened, control order
    struct DomInfo {
        std::vector<int> dominant;   // raw control indices
        std::vector<int> suppressed; // raw control indices
    };
    std::vector<DomInfo> dominators;
};

int FindRawControlIndex(const ComboTemp& combo, const char* name) {
    for (size_t i = 0; i < combo.raws.size(); ++i)
        if (_stricmp(combo.raws[i].name.c_str(), name) == 0)
            return static_cast<int>(i);
    return -1;
}

// reference ParseDeltaName: split on '_', ALL parts must resolve or return 0
int ParseDeltaName(const ComboTemp& combo, const std::string& deltaName,
                   std::vector<int>& outControls) {
    outControls.clear();
    size_t start = 0;
    while (start <= deltaName.size()) {
        size_t end = deltaName.find('_', start);
        std::string part = (end == std::string::npos) ? deltaName.substr(start)
                                                      : deltaName.substr(start, end - start);
        int idx = FindRawControlIndex(combo, part.c_str());
        if (idx < 0) {
            outControls.clear();
            return 0;
        }
        outControls.push_back(idx);
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return static_cast<int>(outControls.size());
}

// reference IsDeltaStateStereo
bool IsDeltaStateStereo(const ComboTemp& combo, const std::string& deltaName) {
    std::vector<int> parts;
    int n = ParseDeltaName(combo, deltaName, parts);
    for (int i = 0; i < n; ++i)
        if (combo.controls[combo.raws[parts[i]].inputControl].stereo)
            return true;
    return false;
}

// phase A - controls + dominator infos (also feeds the wrinkle scales the
// delta loader needs, so this runs BEFORE LoadMeshes)
void ParseComboControls(const dmx::Element* comboOp, ComboTemp& combo, FlexTemp& flex) {
    if (auto controls = comboOp->GetElementArray("controls")) {
        for (const dmx::Element* c : *controls) {
            if (!c) continue;
            ComboControlTemp ct;
            ct.name = c->name;
            if (auto raws = c->GetStringArray("rawControlNames"))
                ct.rawControls = *raws;
            ct.stereo = c->GetBool("stereo", false);
            ct.eyelid = c->GetBool("eyelid", false);
            if (auto fg = c->GetString("flexgroup"))
                ct.flexgroup = *fg;
            if (auto ud = c->GetString("eyesUpDownFlex"))
                ct.eyesUpDownFlexName = *ud;
            ct.hasMinMax = (c->Get("flexMin") != nullptr) || (c->Get("flexMax") != nullptr);
            ct.flexMin = c->GetFloat("flexMin", 0.0f);
            ct.flexMax = c->GetFloat("flexMax", 1.0f);

            // wrinkleScales parallel rawControlNames; short arrays pad 0.0f
            // (CDmeCombinationInputControl::OnElementUnserialized)
            const std::vector<float>* scales = c->GetFloatArray("wrinkleScales");
            int nControlIndex = static_cast<int>(combo.controls.size());
            for (size_t j = 0; j < ct.rawControls.size(); ++j) {
                ComboTemp::RawControl rc;
                rc.name = ct.rawControls[j];
                rc.inputControl = nControlIndex;
                rc.wrinkleScale = (scales && j < scales->size()) ? (*scales)[j] : 0.0f;
                combo.raws.push_back(rc);

                if (rc.wrinkleScale != 0.0f) {
                    FlexTemp::WrinkleScale ws;
                    ws.name = rc.name;
                    ws.scale = rc.wrinkleScale;
                    flex.wrinkleScales.push_back(ws);
                }
            }
            combo.controls.push_back(std::move(ct));
        }
    }

    // RebuildDominatorInfo: skip a rule when
    // either list is empty or any name doesn't resolve
    if (auto doms = comboOp->GetElementArray("dominators")) {
        for (const dmx::Element* rule : *doms) {
            if (!rule) continue;
            auto dnames = rule->GetStringArray("dominators");
            auto snames = rule->GetStringArray("suppressed");
            if (!dnames || !snames || dnames->empty() || snames->empty())
                continue;

            ComboTemp::DomInfo info;
            bool bRuleOk = true;
            for (const std::string& n : *dnames) {
                int idx = FindRawControlIndex(combo, n.c_str());
                if (idx < 0) { bRuleOk = false; break; }
                info.dominant.push_back(idx);
            }
            if (bRuleOk) {
                for (const std::string& n : *snames) {
                    int idx = FindRawControlIndex(combo, n.c_str());
                    if (idx < 0) { bRuleOk = false; break; }
                    info.suppressed.push_back(idx);
                }
            }
            if (bRuleOk)
                combo.dominators.push_back(std::move(info));
        }
    }
}

int FindSourceFlexKey(const Source& src, const char* name) {
    for (size_t i = 0; i < src.flexkeys.size(); ++i)
        if (_stricmp(src.flexkeys[i].name.c_str(), name) == 0)
            return static_cast<int>(i);
    return -1;
}

// phase B for a render-mesh load - after the meshes are built. AddFlexKeys:
// one flexkey per delta state, dedup by name; flexpair
// signal = IsDeltaStateStereo. A render mesh stops here: the rig itself
// (controllers, correctives, dominators, rules) comes from $datamodelflexes -
// which re-derives `stereo` from its own controls (flexreg.cpp
// ApplyDataModelFlex), so this is the no-import fallback.
void AddFlexKeys(const ComboTemp& combo, const FlexTemp& flex, Source& out) {
    for (const SrcMorphAnim& morph : out.morphs) {
        if (FindSourceFlexKey(out, morph.name.c_str()) >= 0)
            continue;
        SrcFlexKey key;
        key.name = morph.name;
        key.stereo = IsDeltaStateStereo(combo, morph.name);
        out.flexkeys.push_back(std::move(key));
    }

    // reference bHasRealBalance - the whole mesh's
    // FIELD_BALANCE, not just the morphed verts. flexreg pairs it with the
    // stereo signal, which an imported rig can still change.
    for (float b : flex.balance)
        if (b != 1.0f) { out.hasBalanceData = true; break; }
}

// $datamodelflexes: BuildCombinationSourceData + CaptureDmeFlexRules, captured
// detached from any geometry - correctives name their delta state instead of
// indexing a source's flexkeys, so the rig can be stamped onto a mesh that came
// out of a different file.
void CaptureFlexRig(const dmx::Element* comboOp, const ComboTemp& combo, FlexRig& out) {
    // raw control name list (s_combinationcontrol_t)
    for (const ComboTemp::RawControl& rc : combo.raws) {
        CombinationControl cc;
        cc.name = rc.name;
        out.controls.push_back(std::move(cc));
    }

    // combination rules per operation target (BuildCombinationSourceData).
    // Only targets with deltaStates AND a valid deltaStateWeights array of at
    // least the same size produce operations (ComputeCombinationInfo).
    auto targets = comboOp->GetElementArray("targets");
    if (targets) {
        std::vector<int> controlIndices;
        for (const dmx::Element* target : *targets) {
            if (!target) continue;
            auto deltas = target->GetElementArray("deltaStates");
            if (!deltas) continue;
            auto weights = target->GetVector2Array("deltaStateWeights");
            if (!weights || static_cast<size_t>(deltas->size()) > weights->size())
                continue;

            for (const dmx::Element* delta : *deltas) {
                if (!delta) continue;
                if (ParseDeltaName(combo, delta->name, controlIndices) == 0)
                    continue;

                FlexRig::Corrective rule;
                rule.delta = delta->name;
                rule.combination = controlIndices;
                // FindDominators: rule i
                // applies when ALL its suppressed controls are in the combo
                for (const ComboTemp::DomInfo& info : combo.dominators) {
                    bool bAll = true;
                    for (int s : info.suppressed) {
                        bool bFound = false;
                        for (int c : rule.combination)
                            if (c == s) { bFound = true; break; }
                        if (!bFound) { bAll = false; break; }
                    }
                    if (bAll)
                        rule.dominators.push_back(info.dominant);
                }
                out.correctives.push_back(std::move(rule));
            }
        }
    }

    // controller remaps per input control (BuildCombinationSourceData)
    for (const ComboControlTemp& ct : combo.controls) {
        ControllerRemap remap;
        remap.name = ct.name;
        remap.flexgroup = ct.flexgroup;
        remap.stereo = ct.stereo;
        remap.eyelid = ct.eyelid;
        remap.hasMinMax = ct.hasMinMax;
        remap.min = ct.flexMin;
        remap.max = ct.flexMax;
        remap.rawControls = ct.rawControls;
        if (ct.eyelid) {
            remap.type = RemapType::Eyelid;
            remap.eyesUpDownFlexName =
                ct.eyesUpDownFlexName.empty() ? "eyes_updown" : ct.eyesUpDownFlexName;
        } else {
            switch (ct.rawControls.size()) {
                case 0:
                case 1: remap.type = RemapType::PassThru; break;
                case 2: remap.type = RemapType::TwoWay; break;
                default: remap.type = RemapType::NWay; break;
            }
        }
        out.remaps.push_back(std::move(remap));
    }

    // DmeFlexRules capture (CaptureDmeFlexRules / BuildFlexRuleMemoryScript).
    // The rules array of a DmeFlexRules element is also named "deltaStates".
    if (targets) {
        for (const dmx::Element* target : *targets) {
            if (!target || target->className != "DmeFlexRules")
                continue;
            out.hasRules = true;
            auto rules = target->GetElementArray("deltaStates");
            if (!rules) continue;
            for (const dmx::Element* rule : *rules) {
                if (!rule) continue;
                SrcFlexRule fr;
                fr.name = rule->name;
                if (rule->className == "DmeFlexRulePassThrough") {
                    fr.expr = rule->name;
                } else if (rule->className == "DmeFlexRuleExpression") {
                    if (auto e = rule->GetString("expr"))
                        fr.expr = *e;
                    // $var$ expansion inside a DMX flex rule is not supported
                    if (fr.expr.find('$') != std::string::npos) {
                        std::fprintf(stderr,
                                     "warning: DMX flex rule '%s' references $variables$ - not "
                                     "supported yet, rule dropped\n",
                                     fr.name.c_str());
                        continue;
                    }
                } else if (rule->className == "DmeFlexRuleLocalVar") {
                    fr.isLocalVar = true;
                } else {
                    std::fprintf(stderr, "warning: unknown DmeDeltaRule: %s of type %s\n",
                                 rule->name.c_str(), rule->className.c_str());
                    continue;
                }
                out.rules.push_back(std::move(fr));
            }
        }
    }
}

} // namespace

// ---------------------------------------------------------------------------

int MaterialTable::LookupTexture(const char* name, bool relative) {
    // reference LookupTexture: names are stored extension-stripped (path kept).
    // Q_StripExtension: drop the extension, keep any directory.
    std::string noExt = name;
    {
        size_t dot = noExt.find_last_of('.');
        size_t slash = noExt.find_last_of("/\\");
        if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
            noExt.resize(dot);
    }
    // Q_FileBase: basename with no directory and no extension.
    auto fileBase = [](const std::string& s) {
        size_t slash = s.find_last_of("/\\");
        std::string b = slash == std::string::npos ? s : s.substr(slash + 1);
        size_t dot = b.find_last_of('.');
        if (dot != std::string::npos) b.resize(dot);
        return b;
    };
    const std::string base = fileBase(name);

    for (size_t i = 0; i < textures.size(); ++i) {
        const Texture& tex = textures[i];
        if (tex.relative == relative) {
            if (_stricmp(tex.name.c_str(), noExt.c_str()) == 0)
                return static_cast<int>(i);
            continue;
        }
        // comparing across flag classes: match on base name only
        if (relative) {
            if (_stricmp(base.c_str(), tex.name.c_str()) == 0)
                return static_cast<int>(i);
        } else {
            if (_stricmp(noExt.c_str(), fileBase(tex.name).c_str()) == 0)
                return static_cast<int>(i);
        }
    }
    Texture t;
    t.name = noExt;
    t.relative = relative;
    textures.push_back(t);
    return static_cast<int>(textures.size()) - 1;
}

int MaterialTable::UseTextureAsMaterial(int textureIdx) {
    if (textures[textureIdx].material == -1) {
        textures[textureIdx].material = static_cast<int>(materialToTexture.size());
        materialToTexture.push_back(textureIdx);
    }
    return textures[textureIdx].material;
}

SourceAnim* FindSourceAnim(Source& src, const char* name) {
    for (auto& a : src.anims)
        if (_stricmp(a.name.c_str(), name) == 0)
            return &a;
    return nullptr;
}

bool DmxUpAxisY() { return s_bUpAxisY; }
void ResetDmxUpAxis() { s_bUpAxisY = false; s_bUpAxisChecked = false; }

bool LoadDmxSource(const dmx::Datamodel& dm, Source& out, MaterialTable& mats, float scale,
                   std::string* err, bool morphSource, MeshFilter* filter, bool animOnly) {
    const dmx::Element* root = dm.root;
    if (!root) {
        if (err) *err = "DMX has no root element";
        return false;
    }

    const dmx::Element* model = root->GetElement("model");
    const dmx::Element* skeleton = root->GetElement("skeleton");
    if (!skeleton) skeleton = model;
    if (!skeleton) {
        if (err) *err = "no 'model'/'skeleton' element on root";
        return false;
    }
    if (!model) model = skeleton;

    // upAxis (set-once): reference switches g_defaultrotation for Y-up DMX
    if (!s_bUpAxisChecked) {
        if (auto up = model->GetString("upAxis")) {
            if (!up->empty() && ((*up)[0] == 'Y' || (*up)[0] == 'y'))
                s_bUpAxisY = true;
        }
        s_bUpAxisChecked = true;
    }

    // Morphs are model data: only parse them for render-mesh loads so a DMX
    // referenced solely as an animation source contributes none (reference
    // LoadingModelBody gate).
    //
    // The combination operator is OPTIONAL - it only supplies wrinkle scales and
    // the per-delta stereo signal. Without one the delta states still load and
    // the rig is hand-authored ($flexcontroller / $datamodelflexes).
    FlexTemp flexTemp;
    ComboTemp comboTemp;
    if (morphSource && !animOnly) {
        flexTemp.enabled = true;
        // controls first - GenerateWrinkleDeltas runs before the meshes load
        if (const dmx::Element* comboOp = root->GetElement("combinationOperator"))
            ParseComboControls(comboOp, comboTemp, flexTemp);
    }

    BoneMap boneMap;
    out.numbones = LoadSkeleton(skeleton, model, out, boneMap);

    LoadBindPose(model, scale, boneMap, out);

    // geometry (if any mesh dags exist under the model). Skipped whole for an
    // animation reference: geometry is model data and reading it would register
    // the file's materials in the compile-wide table, putting a texture the
    // model never draws in the .mdl.
    MeshTemp tmp;
    if (!animOnly) {
        PULSE_PERF("load", "LoadMeshes");
        if (!LoadMeshes(model, scale, boneMap, mats, tmp, flexTemp.enabled ? &flexTemp : nullptr,
                        filter, out)) {
            if (err) *err = "failed to load meshes";
            return false;
        }
        if (!tmp.face.empty())
            { PULSE_PERF("load", "BuildIndividualMeshes"); BuildIndividualMeshes(tmp, flexTemp.enabled ? &flexTemp : nullptr, out); }
    }

    // one flexkey per delta state (reference AddFlexKeys). With no combination
    // operator the ComboTemp is empty, so every key is mono.
    if (flexTemp.enabled)
        AddFlexKeys(comboTemp, flexTemp, out);

    // animations
    if (const dmx::Element* animationList = root->GetElement("animationList"))
        LoadAnimations(animationList, scale, boneMap, out);

    return true;
}

bool LoadDmxFlexRig(const dmx::Datamodel& dm, FlexRig& out, std::string* err) {
    const dmx::Element* root = dm.root;
    if (!root) {
        if (err) *err = "DMX has no root element";
        return false;
    }
    const dmx::Element* comboOp = root->GetElement("combinationOperator");
    if (!comboOp)
        return true; // no flex authored in this file - an empty rig, not a fault

    ComboTemp comboTemp;
    FlexTemp unused; // only here for the wrinkle scales, which need a mesh
    ParseComboControls(comboOp, comboTemp, unused);
    CaptureFlexRig(comboOp, comboTemp, out);
    return true;
}

} // namespace pulse::source
