// skeletonedit.cpp - $skeletonedit operations. See skeletonedit.h.

#include "strcompat.h"
#include "diagnostic.h"

#include "skeletonedit.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>

#include "math/math.h"

namespace pulse::source {

namespace pm = pulse::math;

namespace {

int FindBone(const Source& src, const std::string& name) {
    for (int i = 0; i < src.numbones; ++i)
        if (_stricmp(src.localBone[i].name.c_str(), name.c_str()) == 0) return i;
    return -1;
}

bool ContainsText(std::string hay, std::string needle) {
    std::transform(hay.begin(), hay.end(), hay.begin(), ::tolower);
    std::transform(needle.begin(), needle.end(), needle.begin(), ::tolower);
    return hay.find(needle) != std::string::npos;
}

// localBone is a DFS, so one forward sweep marks the whole subtree.
std::vector<char> SubtreeOf(const Source& src, int bone) {
    std::vector<char> sub(src.numbones, 0);
    sub[bone] = 1;
    for (int i = bone + 1; i < src.numbones; ++i)
        sub[i] = src.localBone[i].parent >= 0 && sub[src.localBone[i].parent];
    return sub;
}

bool HasPose(const Source& src) {
    return src.boneToPose.size() == static_cast<size_t>(src.numbones);
}

pm::matrix3x4 LocalMatrix(const SrcBonePose& pose) {
    pm::matrix3x4 m;
    pm::AngleMatrix(pose.rot, pose.pos, m);
    return m;
}

void SetTranslation(pm::matrix3x4& m, const Vector3& t) {
    m.m[0][3] = t.x;
    m.m[1][3] = t.y;
    m.m[2][3] = t.z;
}

Vector3 Translation(const pm::matrix3x4& m) { return {m.m[0][3], m.m[1][3], m.m[2][3]}; }

Vector3 Lerp(const Vector3& a, const Vector3& b, float t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

Vector3 Unit(Vector3 v) {
    const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return l > 0.0f ? Vector3{v.x / l, v.y / l, v.z / l} : v;
}

// How a vertex (and its morph deltas) is carried by a bone edit.
struct VertexMap {
    std::function<Vector3(const Vector3&)> point, normal, tangent, deltaPos, deltaNormal;
};

// Each vertex moves by the share of its weight that sits on `inSet`.
void DeformBySet(Source& src, const std::vector<char>& inSet, const VertexMap& f) {
    std::vector<float> share(src.vertex.size(), 0.0f);

    for (size_t i = 0; i < src.vertex.size(); ++i) {
        SrcVertex& v = src.vertex[i];
        const SrcBoneWeight& bw = v.boneweight;
        for (int k = 0; k < bw.numbones; ++k)
            if (bw.bone[k] >= 0 && bw.bone[k] < src.numbones && inSet[bw.bone[k]])
                share[i] += bw.weight[k];
        if (share[i] <= 0.0f)
            continue;

        const Vector3 tan{v.tangentS.x, v.tangentS.y, v.tangentS.z};
        const Vector3 t = Unit(Lerp(tan, Unit(f.tangent(tan)), share[i]));
        v.position = Lerp(v.position, f.point(v.position), share[i]);
        v.normal = Unit(Lerp(v.normal, Unit(f.normal(v.normal)), share[i]));
        v.tangentS = {t.x, t.y, t.z, v.tangentS.w};
    }

    for (SrcMorphAnim& morph : src.morphs)
        for (SrcVertAnim& va : morph.vanims) {
            if (va.vertex < 0 || static_cast<size_t>(va.vertex) >= share.size() ||
                share[va.vertex] <= 0.0f)
                continue;
            va.pos = Lerp(va.pos, f.deltaPos(va.pos), share[va.vertex]);
            va.normal = Lerp(va.normal, f.deltaNormal(va.normal), share[va.vertex]);
        }
}

// Replaces the bind pose with `bnew`. A moved bone keeps its skinning transform, so every
// animation frame is re-expressed against the new rest frames; unmoved bones keep their floats.
void RebindBones(Source& src, const std::vector<pm::matrix3x4>& bnew, const std::vector<char>& moved) {
    const int n = src.numbones;

    std::vector<pm::matrix3x4> delta(n);
    for (int i = 0; i < n; ++i)
        if (moved[i])
            delta[i] = pm::ConcatTransforms(pm::MatrixInvert(src.boneToPose[i]), bnew[i]);

    std::vector<pm::matrix3x4> oldGlobal(n), newGlobal(n);
    for (SourceAnim& anim : src.anims)
        for (std::vector<SrcBonePose>& frame : anim.frames) {
            frame.resize(n);

            for (int i = 0; i < n; ++i) {
                const pm::matrix3x4 m = LocalMatrix(frame[i]);
                const int p = src.localBone[i].parent;
                oldGlobal[i] = p < 0 ? m : pm::ConcatTransforms(oldGlobal[p], m);
                newGlobal[i] = moved[i] ? pm::ConcatTransforms(oldGlobal[i], delta[i]) : oldGlobal[i];
            }

            for (int i = 0; i < n; ++i) {
                const int p = src.localBone[i].parent;
                if (!moved[i] && (p < 0 || !moved[p]))
                    continue;
                const pm::matrix3x4 local =
                    p < 0 ? newGlobal[i]
                          : pm::ConcatTransforms(pm::MatrixInvert(newGlobal[p]), newGlobal[i]);
                pm::MatrixAngles(local, frame[i].rot, frame[i].pos);
            }
        }

    src.boneToPose = bnew;
}

// Drops every bone with keep == 0. Influences move to the nearest kept ancestor (merging
// duplicates) and children are re-parented there, keeping their world pose in every frame.
// Returns false, naming the bone, when a weighted bone has no kept ancestor.
bool RemoveBones(Source& src, const std::vector<char>& keep, std::string* failedBone) {
    const int n = src.numbones;

    std::vector<int> remap(n, -1), target(n, -1);
    int nKept = 0;
    for (int i = 0; i < n; ++i)
        if (keep[i])
            remap[i] = nKept++;

    // nearest kept ancestor-or-self, in OLD indices
    for (int i = 0; i < n; ++i) {
        int b = i;
        while (b >= 0 && !keep[b])
            b = src.localBone[b].parent;
        target[i] = b;
    }

    for (const SrcVertex& v : src.vertex)
        for (int k = 0; k < v.boneweight.numbones; ++k) {
            const int b = v.boneweight.bone[k];
            if (b >= 0 && b < n && target[b] < 0 && v.boneweight.weight[k] > 0.0f) {
                *failedBone = src.localBone[b].name;
                return false;
            }
        }

    std::vector<pm::matrix3x4> global(n);
    std::vector<SrcBonePose> rebuilt(nKept);
    for (SourceAnim& anim : src.anims)
        for (std::vector<SrcBonePose>& frame : anim.frames) {
            frame.resize(n);

            for (int i = 0; i < n; ++i) {
                const pm::matrix3x4 m = LocalMatrix(frame[i]);
                const int p = src.localBone[i].parent;
                global[i] = p < 0 ? m : pm::ConcatTransforms(global[p], m);
            }

            for (int i = 0; i < n; ++i) {
                if (!keep[i])
                    continue;
                SrcBonePose& out = rebuilt[remap[i]];
                const int oldParent = src.localBone[i].parent;
                if (oldParent < 0 || keep[oldParent]) {
                    out = frame[i];
                    continue;
                }

                const int anc = target[oldParent];
                const pm::matrix3x4 local =
                    anc >= 0 ? pm::ConcatTransforms(pm::MatrixInvert(global[anc]), global[i]) : global[i];
                pm::MatrixAngles(local, out.rot, out.pos);
            }
            frame = rebuilt;
        }

    const bool havePose = HasPose(src);
    std::vector<LocalBone> bones(nKept);
    std::vector<pm::matrix3x4> pose(havePose ? nKept : 0);
    for (int i = 0; i < n; ++i) {
        if (!keep[i])
            continue;
        LocalBone& b = bones[remap[i]];
        b = src.localBone[i];
        const int oldParent = src.localBone[i].parent;
        b.parent = oldParent >= 0 && target[oldParent] >= 0 ? remap[target[oldParent]] : -1;
        if (havePose)
            pose[remap[i]] = src.boneToPose[i];
    }
    src.localBone.swap(bones);
    src.boneToPose.swap(pose);
    src.numbones = nKept;

    for (SrcVertex& v : src.vertex) {
        SrcBoneWeight merged;
        for (int k = 0; k < v.boneweight.numbones; ++k) {
            const int b = v.boneweight.bone[k];
            const int dst = b >= 0 && b < n && target[b] >= 0 ? remap[target[b]] : -1;
            if (dst < 0)
                continue;

            int slot = 0;
            while (slot < merged.numbones && merged.bone[slot] != dst)
                ++slot;
            if (slot == merged.numbones) {
                merged.bone[slot] = dst;
                merged.weight[slot] = 0.0f;
                ++merged.numbones;
            }
            merged.weight[slot] += v.boneweight.weight[k];
        }
        v.boneweight = merged;
    }
    return true;
}

std::vector<char> SkinnedBones(const Source& src) {
    std::vector<char> skinned(src.numbones, 0);
    for (const SrcVertex& v : src.vertex)
        for (int k = 0; k < v.boneweight.numbones; ++k) {
            const int b = v.boneweight.bone[k];
            if (b >= 0 && b < src.numbones)
                skinned[b] = 1;
        }
    return skinned;
}

// Rigid move of `bone` (and its subtree with affectChildren) to a new global pose.
void RealignOne(Source& src, int bone, const EditSkeletonOperation& op) {
    const pm::matrix3x4 base = src.boneToPose[bone];
    const Vector3 origin = Translation(base);
    pm::matrix3x4 next = base;

    if (op.hasAngles) {
        pm::matrix3x4 rotation;
        pm::AngleMatrixDeg(op.angles, rotation);
        next = op.absolute ? rotation
                           : (op.worldAxes ? pm::ConcatTransforms(rotation, base)
                                           : pm::ConcatTransforms(base, rotation));
        SetTranslation(next, origin);
    }
    if (op.hasOrigin) {
        const Vector3 offset = op.worldAxes || op.absolute ? op.origin : pm::VectorRotate(op.origin, base);
        SetTranslation(next, op.absolute ? offset
                                         : Vector3{origin.x + offset.x, origin.y + offset.y, origin.z + offset.z});
    }

    const pm::matrix3x4 carry = pm::ConcatTransforms(next, pm::MatrixInvert(base));
    std::vector<char> inSet = op.affectChildren ? SubtreeOf(src, bone) : std::vector<char>(src.numbones, 0);
    inSet[bone] = 1;

    std::vector<pm::matrix3x4> pose = src.boneToPose;
    pose[bone] = next;
    for (int i = bone + 1; i < src.numbones; ++i)
        if (inSet[i])
            pose[i] = pm::ConcatTransforms(carry, src.boneToPose[i]);

    RebindBones(src, pose, inSet);

    if (op.affectMeshes) {
        VertexMap f;
        f.point = [&](const Vector3& v) { return pm::VectorTransform(v, carry); };
        f.normal = f.tangent = f.deltaPos = f.deltaNormal =
            [&](const Vector3& v) { return pm::VectorRotate(v, carry); };
        DeformBySet(src, inSet, f);
    }
}

// Scale about the bone's origin along its own axes; normals take the inverse scale.
void ScaleOne(Source& src, int bone, const Vector3& scale, bool affectChildren) {
    const pm::matrix3x4 B = src.boneToPose[bone];
    const Vector3 inv{1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};

    auto mul = [](const Vector3& v, const Vector3& s) { return Vector3{v.x * s.x, v.y * s.y, v.z * s.z}; };
    auto point = [&](const Vector3& v) { return pm::VectorTransform(mul(pm::VectorITransform(v, B), scale), B); };
    auto dir = [&](const Vector3& v, const Vector3& s) { return pm::VectorRotate(mul(pm::VectorIRotate(v, B), s), B); };

    std::vector<char> inSet = affectChildren ? SubtreeOf(src, bone) : std::vector<char>(src.numbones, 0);
    inSet[bone] = 1;

    VertexMap f;
    f.point = point;
    f.normal = [&](const Vector3& v) { return dir(v, inv); };
    f.tangent = [&](const Vector3& v) { return dir(v, scale); };
    f.deltaPos = f.tangent;
    f.deltaNormal = f.normal;
    DeformBySet(src, inSet, f);

    if (!affectChildren)
        return;

    for (int i = bone + 1; i < src.numbones; ++i) {
        if (!inSet[i])
            continue;
        pm::matrix3x4& m = src.boneToPose[i];
        SetTranslation(m, point(Translation(m)));
    }
    for (SourceAnim& anim : src.anims)
        for (std::vector<SrcBonePose>& frame : anim.frames)
            for (int i = bone + 1; i < src.numbones && i < static_cast<int>(frame.size()); ++i) {
                const int p = src.localBone[i].parent;
                if (!inSet[i] || p < 0)
                    continue;
                const pm::matrix3x4& P = src.boneToPose[p];
                const Vector3 d = pm::VectorRotate(frame[i].pos, P);
                frame[i].pos = pm::VectorIRotate(dir(d, scale), P);
            }
}

} // namespace

bool ApplyEditSkeleton(Source& src, const std::vector<EditSkeletonOperation>& operations,
                       std::string* err, int* errorLine) {
    std::vector<char> selection(src.numbones, 0);

    for (const EditSkeletonOperation& op : operations) {
        auto fail = [&](const std::string& message) {
            *err = message;
            *errorLine = op.line;
            return false;
        };
        auto combine = [&](const std::vector<char>& matches) {
            for (int i = 0; i < src.numbones; ++i) {
                if (op.combine == SelectionCombine::Add) selection[i] = selection[i] || matches[i];
                else if (op.combine == SelectionCombine::Subtract) selection[i] = selection[i] && !matches[i];
                else selection[i] = selection[i] && matches[i];
            }
        };
        auto warnEmpty = [&](const std::vector<char>& matches, const char* selector) {
            if (op.ignoreSelectWarn || std::find(matches.begin(), matches.end(), 1) != matches.end())
                return;
            pulse::diag::Location at = pulse::diag::Current();
            at.line = op.line;
            pulse::diag::WarnAtf(at, "$skeletonedit: %s: no bones matched", selector);
        };

        using Kind = SkeletonOperation;
        if (op.kind == Kind::InvertSelection) {
            for (char& s : selection) s = !s;
        } else if (op.kind == Kind::SelectBone || op.kind == Kind::SelectBoneTree) {
            std::vector<char> matches(src.numbones, 0);
            const int bone = FindBone(src, op.name);
            if (bone >= 0) {
                if (op.kind == Kind::SelectBoneTree) matches = SubtreeOf(src, bone);
                else matches[bone] = 1;
            }
            warnEmpty(matches, op.kind == Kind::SelectBoneTree ? "$selectbonetree" : "$selectbone");
            combine(matches);
        } else if (op.kind == Kind::SelectBoneSubstring) {
            std::vector<char> matches(src.numbones, 0);
            for (int i = 0; i < src.numbones; ++i)
                matches[i] = ContainsText(src.localBone[i].name, op.name);
            warnEmpty(matches, "$selectbonesubstring");
            combine(matches);
        } else if (op.kind == Kind::SelectUnskinned) {
            std::vector<char> needed = SkinnedBones(src);
            if (op.keepParents)
                for (int i = src.numbones - 1; i >= 0; --i)
                    if (needed[i] && src.localBone[i].parent >= 0)
                        needed[src.localBone[i].parent] = 1;

            std::vector<char> matches(src.numbones, 0);
            for (int i = 0; i < src.numbones; ++i)
                matches[i] = !needed[i];
            warnEmpty(matches, "$selectunskinnedbones");
            combine(matches);
        } else if (op.kind == Kind::RealignBone) {
            if (!HasPose(src))
                return fail("$realignbone: the source has no bind pose");
            for (int i = 0; i < src.numbones; ++i)
                if (selection[i])
                    RealignOne(src, i, op);
        } else if (op.kind == Kind::CollapseBone) {
            std::vector<char> keep(src.numbones, 0);
            int removed = 0;
            for (int i = 0; i < src.numbones; ++i) {
                keep[i] = !selection[i];
                removed += selection[i] ? 1 : 0;
            }
            if (removed == 0)
                continue;
            if (removed == src.numbones)
                return fail("$collapsebone: cannot collapse every bone");

            std::string failedBone;
            if (!RemoveBones(src, keep, &failedBone))
                return fail("$collapsebone: bone \"" + failedBone +
                            "\" is skinned and has no parent to take its weights");
            selection.assign(src.numbones, 0);
        } else if (op.kind == Kind::CreateBone) {
            if (!HasPose(src))
                return fail("$createbone: the source has no bind pose");
            if (FindBone(src, op.name) >= 0)
                return fail("$createbone: bone \"" + op.name + "\" already exists");

            int parent = -1;
            if (!op.parent.empty()) {
                parent = FindBone(src, op.parent);
                if (parent < 0)
                    return fail("$createbone: no parent bone \"" + op.parent + "\"");
            }

            pm::matrix3x4 local;
            pm::AngleMatrixDeg(op.angles, local);
            SetTranslation(local, op.origin);

            SrcBonePose pose;
            pm::MatrixAngles(local, pose.rot, pose.pos);

            LocalBone bone;
            bone.name = op.name;
            bone.parent = parent;
            src.localBone.push_back(bone);
            src.boneToPose.push_back(parent < 0 ? local : pm::ConcatTransforms(src.boneToPose[parent], local));
            ++src.numbones;
            for (SourceAnim& anim : src.anims)
                for (std::vector<SrcBonePose>& frame : anim.frames) {
                    frame.resize(src.numbones - 1);
                    frame.push_back(pose);
                }
            selection.push_back(op.selectNew ? 1 : 0);
        } else if (op.kind == Kind::ScaleBone) {
            if (!HasPose(src))
                return fail("$scalebone: the source has no bind pose");
            const int bone = FindBone(src, op.name);
            if (bone < 0)
                return fail("$scalebone: no bone \"" + op.name + "\"");
            ScaleOne(src, bone, op.scale, op.affectChildren);
        }
    }

    return true;
}

} // namespace pulse::source
