// meshedit.cpp - $rendermesh edit options. See meshedit.h.

#include "meshedit.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "math/math.h"

namespace pulse::source {

namespace pm = pulse::math;
namespace lim = pulse::limits;

bool MeshFilter::Keep(const std::string& meshName, const std::string& dagName) {
    if (names.empty())
        return true;
    bool hit = false;
    for (Entry& e : names) {
        if (_stricmp(e.name.c_str(), meshName.c_str()) == 0 ||
            _stricmp(e.name.c_str(), dagName.c_str()) == 0) {
            e.matched = true;
            hit = true;
        }
    }
    return exclusive ? !hit : hit;
}

void ApplyWrinkleScales(Source& src, std::vector<WrinkleScaleOption>& opts) {
    for (WrinkleScaleOption& w : opts) {
        SrcMorphAnim* morph = nullptr;
        for (SrcMorphAnim& m : src.morphs)
            if (_stricmp(m.name.c_str(), w.shape.c_str()) == 0) { morph = &m; break; }
        if (!morph)
            continue;
        w.matched = true;

        // Valve Vector::Length (float accumulate + sqrtf), as the DMX path uses
        float maxDeflection = 0.0f;
        for (const SrcVertAnim& va : morph->vanims) {
            const float d = sqrtf(va.pos.x * va.pos.x + va.pos.y * va.pos.y + va.pos.z * va.pos.z);
            if (d > maxDeflection)
                maxDeflection = d;
        }
        if (w.scale == 0.0f || maxDeflection == 0.0f) {
            for (SrcVertAnim& va : morph->vanims)
                va.wrinkle = 0.0f;
            continue;
        }
        const double invMax = static_cast<double>(w.scale) / static_cast<double>(maxDeflection);
        for (SrcVertAnim& va : morph->vanims) {
            const float d = sqrtf(va.pos.x * va.pos.x + va.pos.y * va.pos.y + va.pos.z * va.pos.z);
            va.wrinkle = static_cast<float>(static_cast<double>(d) * invMax);
        }
    }
}

const std::string* MeshFilter::Unmatched() const {
    for (const Entry& e : names)
        if (!e.matched)
            return &e.name;
    return nullptr;
}

void CullUnskinnedBones(Source& src, SkinnedBoneCull mode) {
    if (mode == SkinnedBoneCull::None || src.numbones <= 0)
        return;

    // "skinned" = named by a vertex influence. localBone is a DFS, so a parent
    // always precedes its children and one backwards sweep propagates the keep.
    std::vector<char> keep(src.numbones, 0);
    for (const SrcVertex& v : src.vertex)
        for (int i = 0; i < v.boneweight.numbones; ++i) {
            const int b = v.boneweight.bone[i];
            if (b >= 0 && b < src.numbones)
                keep[b] = 1;
        }
    bool anySkinned = false;
    for (char k : keep)
        if (k) { anySkinned = true; break; }
    if (!anySkinned)
        return; // no geometry left to skin anything - culling to nothing helps nobody

    if (mode == SkinnedBoneCull::Tree) {
        for (int i = src.numbones - 1; i >= 0; --i) {
            const int p = src.localBone[i].parent;
            if (keep[i] && p >= 0)
                keep[p] = 1;
        }
    }

    std::vector<int> remap(src.numbones, -1);
    int nKept = 0;
    for (int i = 0; i < src.numbones; ++i)
        if (keep[i])
            remap[i] = nKept++;
    if (nKept == src.numbones)
        return;

    // nearest surviving ancestor, kept in OLD indices too - the aggressive mode
    // needs its global pose to rebuild the local one below it
    std::vector<int> ancestor(src.numbones, -1);
    for (int i = 0; i < src.numbones; ++i) {
        if (!keep[i])
            continue;
        int p = src.localBone[i].parent;
        while (p >= 0 && !keep[p])
            p = src.localBone[p].parent;
        ancestor[i] = p;
    }

    // Local poses are parent-relative, so a bone that outlived its parent needs
    // new numbers: global pose under the ORIGINAL hierarchy, then re-localized
    // against the ancestor that survived. A bone whose parent survived keeps its
    // floats bit-for-bit, which is why a tree cull cannot perturb geometry.
    std::vector<pm::matrix3x4> global(src.numbones);
    std::vector<SrcBonePose> rebuilt(nKept);
    for (SourceAnim& anim : src.anims) {
        for (std::vector<SrcBonePose>& frame : anim.frames) {
            frame.resize(src.numbones);
            for (int i = 0; i < src.numbones; ++i) {
                pm::matrix3x4 m;
                pm::AngleMatrix(frame[i].rot, frame[i].pos, m);
                const int p = src.localBone[i].parent;
                global[i] = (p < 0) ? m : pm::ConcatTransforms(global[p], m);
            }
            for (int i = 0; i < src.numbones; ++i) {
                if (!keep[i])
                    continue;
                SrcBonePose& out = rebuilt[remap[i]];
                const int oldParent = src.localBone[i].parent;
                if (oldParent < 0 || keep[oldParent]) {
                    out = frame[i];
                    continue;
                }
                pm::matrix3x4 local = global[i];
                if (ancestor[i] >= 0)
                    local = pm::ConcatTransforms(pm::MatrixInvert(global[ancestor[i]]), local);
                pm::MatrixAngles(local, out.rot, out.pos);
            }
            frame = rebuilt;
        }
    }

    // skeleton (boneToPose is global, so the survivors' entries carry over as-is)
    const bool havePose = src.boneToPose.size() == static_cast<size_t>(src.numbones);
    std::vector<LocalBone> bones(nKept);
    std::vector<pm::matrix3x4> pose(havePose ? nKept : 0);
    for (int i = 0; i < src.numbones; ++i) {
        if (!keep[i])
            continue;
        LocalBone& b = bones[remap[i]];
        b = src.localBone[i];
        b.parent = ancestor[i] >= 0 ? remap[ancestor[i]] : -1;
        if (havePose)
            pose[remap[i]] = src.boneToPose[i];
    }
    src.localBone.swap(bones);
    src.boneToPose.swap(pose);
    src.numbones = nKept;

    for (SrcVertex& v : src.vertex)
        for (int i = 0; i < v.boneweight.numbones; ++i)
            v.boneweight.bone[i] = remap[v.boneweight.bone[i]];
}

// ---------------------------------------------------------------------------
// MergeSources - see meshedit.h.
// ---------------------------------------------------------------------------

bool MergeSources(const std::vector<Source*>& parts, Source& out, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err) *err = m;
        return false;
    };
    if (parts.empty())
        return fail("nothing to merge");

    // ---- skeleton: union by name. localBone is parent-before-child in every
    // part and a new bone is appended only after its parent has been, so the
    // merged list keeps that ordering - which is all the compile stage needs.
    std::vector<std::vector<int>> boneMap(parts.size());
    std::vector<SrcBonePose> bind; // merged BindPose frame 0
    for (size_t p = 0; p < parts.size(); ++p) {
        Source& s = *parts[p];
        const SourceAnim* sb = FindSourceAnim(s, "BindPose");
        if (!sb && !s.anims.empty())
            sb = &s.anims[0];
        const bool haveBind = sb && !sb->frames.empty() &&
                              sb->frames[0].size() == static_cast<size_t>(s.numbones);
        boneMap[p].assign(s.numbones, -1);
        for (int i = 0; i < s.numbones; ++i) {
            int found = -1;
            for (size_t j = 0; j < out.localBone.size(); ++j)
                if (_stricmp(out.localBone[j].name.c_str(), s.localBone[i].name.c_str()) == 0) {
                    found = static_cast<int>(j);
                    break;
                }
            if (found >= 0) {
                boneMap[p][i] = found;
                continue;
            }
            if (out.localBone.size() >= static_cast<size_t>(lim::kMaxSrcBones))
                return fail("merged mesh needs more than " + std::to_string(lim::kMaxSrcBones) +
                            " bones");
            LocalBone b = s.localBone[i];
            const int par = s.localBone[i].parent;
            b.parent = par >= 0 ? boneMap[p][par] : -1;
            boneMap[p][i] = static_cast<int>(out.localBone.size());
            out.localBone.push_back(std::move(b));
            bind.push_back(haveBind ? sb->frames[0][i] : SrcBonePose{});
        }
    }
    out.numbones = static_cast<int>(out.localBone.size());

    SourceAnim anim;
    anim.name = "BindPose";
    anim.numframes = 1;
    anim.frames.push_back(std::move(bind));
    out.anims.push_back(std::move(anim));

    // Build_Reference, exactly as the loaders run it after LoadBindPose
    const std::vector<SrcBonePose>& bp = out.anims[0].frames[0];
    out.boneToPose.resize(out.numbones);
    for (int i = 0; i < out.numbones; ++i) {
        pm::matrix3x4 m;
        pm::AngleMatrix(bp[i].rot, bp[i].pos, m);
        const int par = out.localBone[i].parent;
        out.boneToPose[i] = par < 0 ? m : pm::ConcatTransforms(out.boneToPose[par], m);
    }

    // ---- vertices: concatenate, then group by material. Every part is already
    // material-sorted, so one stable sort interleaves them material by material
    // and leaves each part's own order intact inside a material.
    size_t total = 0;
    for (const Source* s : parts)
        total += s->vertex.size();
    if (total > static_cast<size_t>(lim::kMaxVerts))
        return fail("merged mesh has " + std::to_string(total) + " vertices (max " +
                    std::to_string(lim::kMaxVerts) + ")");

    std::vector<std::pair<int, int>> order; // (part, part-local vertex)
    order.reserve(total);
    for (size_t p = 0; p < parts.size(); ++p)
        for (size_t i = 0; i < parts[p]->vertex.size(); ++i)
            order.emplace_back(static_cast<int>(p), static_cast<int>(i));
    std::stable_sort(order.begin(), order.end(),
                     [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                         return parts[a.first]->vertex[a.second].material <
                                parts[b.first]->vertex[b.second].material;
                     });

    std::vector<std::vector<int>> vertMap(parts.size());
    for (size_t p = 0; p < parts.size(); ++p)
        vertMap[p].assign(parts[p]->vertex.size(), -1);
    out.vertex.resize(order.size());
    out.mergedParts.assign(parts.begin(), parts.end());
    out.vertexPart.assign(order.size(), 0);
    for (size_t i = 0; i < order.size(); ++i) {
        const int p = order[i].first;
        out.vertexPart[i] = p;
        SrcVertex& v = out.vertex[i];
        v = parts[p]->vertex[order[i].second];
        for (int b = 0; b < v.boneweight.numbones; ++b) {
            const int lb = v.boneweight.bone[b];
            v.boneweight.bone[b] =
                (lb >= 0 && lb < static_cast<int>(boneMap[p].size())) ? boneMap[p][lb] : 0;
        }
        vertMap[p][order[i].second] = static_cast<int>(i);
    }

    // ---- faces: part face indices are MESH-relative, so they go through the
    // part's own vertexoffset before the merged remap.
    struct MFace {
        int material;
        uint32_t a, b, c;
    };
    std::vector<MFace> faces;
    for (size_t p = 0; p < parts.size(); ++p) {
        const Source& s = *parts[p];
        for (int mi = 0; mi < s.nummeshes; ++mi) {
            const int m = s.meshindex[mi];
            const SrcMesh& sm = s.mesh[m];
            for (int f = 0; f < sm.numfaces; ++f) {
                const SrcFace& sf = s.face[sm.faceoffset + f];
                faces.push_back({m,
                                 static_cast<uint32_t>(vertMap[p][sm.vertexoffset + sf.a]),
                                 static_cast<uint32_t>(vertMap[p][sm.vertexoffset + sf.b]),
                                 static_cast<uint32_t>(vertMap[p][sm.vertexoffset + sf.c])});
            }
        }
    }
    if (faces.size() > static_cast<size_t>(lim::kMaxTriangles))
        return fail("merged mesh has " + std::to_string(faces.size()) + " triangles (max " +
                    std::to_string(lim::kMaxTriangles) + ")");
    std::stable_sort(faces.begin(), faces.end(),
                     [](const MFace& a, const MFace& b) { return a.material < b.material; });

    // PointMeshesToVertexAndFaceData / BuildFaceList, same shape as the loader's
    const int numverts = static_cast<int>(out.vertex.size());
    const int numfaces = static_cast<int>(faces.size());
    out.mesh.assign(lim::kMaxSkins, SrcMesh{});
    out.meshindex.assign(lim::kMaxSkins, 0);
    for (int m = 0; m < lim::kMaxSkins; ++m) {
        out.mesh[m].vertexoffset = numverts;
        out.mesh[m].faceoffset = numfaces;
    }
    for (int i = 0; i < numverts; ++i) {
        const int m = out.vertex[i].material;
        out.mesh[m].numvertices++;
        if (out.mesh[m].vertexoffset > i)
            out.mesh[m].vertexoffset = i;
    }
    for (int i = 0; i < numfaces; ++i) {
        const int m = faces[i].material;
        out.mesh[m].numfaces++;
        if (out.mesh[m].faceoffset > i)
            out.mesh[m].faceoffset = i;
    }
    out.face.resize(numfaces);
    out.nummeshes = 0;
    for (int m = 0; m < lim::kMaxSkins; ++m) {
        if (!out.mesh[m].numfaces)
            continue;
        out.meshindex[out.nummeshes++] = m;
        const int base = out.mesh[m].vertexoffset;
        for (int i = out.mesh[m].faceoffset; i < out.mesh[m].faceoffset + out.mesh[m].numfaces;
             ++i) {
            out.face[i].a = faces[i].a - base;
            out.face[i].b = faces[i].b - base;
            out.face[i].c = faces[i].c - base;
        }
    }

    // ---- delta shapes: one morph per NAME across the parts, so a flex authored
    // on both halves moves both. Vertex indices are model-relative, so they ride
    // the same remap the faces did.
    for (size_t p = 0; p < parts.size(); ++p) {
        for (const SrcMorphAnim& morph : parts[p]->morphs) {
            SrcMorphAnim* dst = nullptr;
            for (SrcMorphAnim& existing : out.morphs)
                if (_stricmp(existing.name.c_str(), morph.name.c_str()) == 0) {
                    dst = &existing;
                    break;
                }
            if (!dst) {
                out.morphs.push_back(SrcMorphAnim{});
                out.morphs.back().name = morph.name;
                dst = &out.morphs.back();
            }
            for (const SrcVertAnim& va : morph.vanims) {
                if (va.vertex < 0 || va.vertex >= static_cast<int>(vertMap[p].size()))
                    continue;
                SrcVertAnim d = va;
                d.vertex = vertMap[p][va.vertex];
                dst->vanims.push_back(d);
            }
        }
        if (parts[p]->hasBalanceData)
            out.hasBalanceData = true;
    }

    // AddFlexKeys: one key per delta state, already deduped by the loop above.
    // A part that split the delta stereo splits it for the merged mesh too - the
    // rig $datamodelflexes stamps on can still overrule it (flexreg.cpp).
    for (const SrcMorphAnim& morph : out.morphs) {
        SrcFlexKey key;
        key.name = morph.name;
        for (const Source* s : parts)
            for (const SrcFlexKey& k : s->flexkeys)
                if (_stricmp(k.name.c_str(), morph.name.c_str()) == 0) {
                    if (k.stereo)
                        key.stereo = true;
                    key.target1 = k.target1;
                }
        out.flexkeys.push_back(std::move(key));
    }

    out.kind = LoadKind::Model;
    return true;
}

} // namespace pulse::source
