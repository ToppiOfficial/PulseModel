// meshedit.cpp - $rendermesh edit options. See meshedit.h.

#include "strcompat.h"

#include "meshedit.h"

#include <algorithm>
#include <filesystem>
#include <map>
#include <tuple>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstring>

#include "math/math.h"
#include "meshoptimizer.h"
#include "pulselimits.h"

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

int MeshFilter::Tag(const std::string& meshName, const std::string& dagName) {
    auto find = [&](std::vector<TagEntry>& list) {
        for (TagEntry& e : list)
            if (_stricmp(e.name.c_str(), meshName.c_str()) == 0 ||
                _stricmp(e.name.c_str(), dagName.c_str()) == 0) {
                e.matched = true;
                return e.tag;
            }
        return 0;
    };
    return find(tags) | (find(inflateTags) << 8);
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

bool MeshFilter::MaterialRemoved(const std::string& materialName) const {
    if (removeWords.empty() && removeNames.empty())
        return false;
    std::string hay = materialName;
    std::transform(hay.begin(), hay.end(), hay.begin(), ::tolower);
    std::string base = std::filesystem::path(hay).stem().string();
    bool hit = false;
    for (const RemoveEntry& e : removeNames) {
        std::string n = e.name;
        std::transform(n.begin(), n.end(), n.begin(), ::tolower);
        if (n == hay || n == base)
            hit = e.matched = true;
    }
    if (hit)
        return true;
    for (const std::string& w : removeWords) {
        std::string needle = w;
        std::transform(needle.begin(), needle.end(), needle.begin(), ::tolower);
        if (!needle.empty() && hay.find(needle) != std::string::npos)
            return true;
    }
    return false;
}

namespace {

// Exact float compare on the bit pattern - a weld only ever joins vertices the
// exporter emitted from one authored point.
int CmpBits(const void* a, const void* b, size_t n) { return std::memcmp(a, b, n); }

int WeldCompare(const SrcVertex& a, const SrcVertex& b, WeldMode mode) {
    if (a.material != b.material)
        return a.material < b.material ? -1 : 1;
    if (int c = CmpBits(&a.position, &b.position, sizeof(a.position)))
        return c;
    if (mode == WeldMode::KeepSeams)
        if (int c = CmpBits(&a.texcoord, &b.texcoord, sizeof(a.texcoord)))
            return c;
    if (a.boneweight.numbones != b.boneweight.numbones)
        return a.boneweight.numbones < b.boneweight.numbones ? -1 : 1;
    for (int i = 0; i < a.boneweight.numbones; ++i) {
        if (a.boneweight.bone[i] != b.boneweight.bone[i])
            return a.boneweight.bone[i] < b.boneweight.bone[i] ? -1 : 1;
        if (int c = CmpBits(&a.boneweight.weight[i], &b.boneweight.weight[i], sizeof(float)))
            return c;
    }
    return 0;
}

// One face in model-absolute vertex indices, before FillMeshes groups it.
struct MFace {
    int material;
    uint32_t a, b, c;
    uint16_t tag = 0;
};

// PointMeshesToVertexAndFaceData / BuildFaceList, same shape as the loader's.
// Vertices and faces must already be material-sorted.
void FillMeshes(Source& out, const std::vector<MFace>& faces) {
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
    const bool tagged = std::any_of(faces.begin(), faces.end(), [](const MFace& f) { return f.tag != 0; });
    out.faceTag.assign(tagged ? numfaces : 0, 0);
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
            if (tagged)
                out.faceTag[i] = faces[i].tag;
        }
    }
}

} // namespace

void WeldVertices(Source& src, WeldMode mode, bool sharp, float sharpAngle) {
    const int n = static_cast<int>(src.vertex.size());
    if (mode == WeldMode::None || n == 0)
        return;

    // group the equal vertices; the lowest index in a group is its survivor, so
    // the surviving order (and with it the material grouping) is unchanged
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i)
        order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        const int c = WeldCompare(src.vertex[a], src.vertex[b], mode);
        return c ? c < 0 : a < b;
    });

    // no sharp pass: one cluster per position, every normal averages together
    const float sharpCos =
        !sharp ? -2.0f
               : (sharpAngle > 0.0f ? std::cos(sharpAngle * pm::kPiF / 180.0f) : 1.0f - 1e-6f);

    std::vector<int> rep(n);
    std::vector<int> run, clusterHead;
    for (int i = 0; i < n;) {
        int j = i + 1;
        while (j < n && WeldCompare(src.vertex[order[i]], src.vertex[order[j]], mode) == 0)
            ++j;
        run.assign(order.begin() + i, order.begin() + j);
        std::sort(run.begin(), run.end());
        // one cluster per normal direction: a vertex joins the first cluster it
        // is smooth with, so a sharp corner keeps its own split normal
        clusterHead.clear();
        for (int v : run) {
            int head = -1;
            for (int h : clusterHead) {
                const Vector3& a = src.vertex[h].normal;
                const Vector3& b = src.vertex[v].normal;
                if (a.x * b.x + a.y * b.y + a.z * b.z >= sharpCos) {
                    head = h;
                    break;
                }
            }
            if (head < 0) {
                clusterHead.push_back(v);
                head = v;
            }
            rep[v] = head;
        }
        i = j;
    }

    std::vector<int> remap(n, -1);
    std::vector<SrcVertex> kept;
    kept.reserve(n);
    std::vector<Vector3> normalSum;
    for (int i = 0; i < n; ++i) {
        if (rep[i] == i) {
            remap[i] = static_cast<int>(kept.size());
            kept.push_back(src.vertex[i]);
            normalSum.push_back(src.vertex[i].normal);
        }
    }
    if (static_cast<int>(kept.size()) == n)
        return;
    for (int i = 0; i < n; ++i) {
        remap[i] = remap[rep[i]];
        if (rep[i] != i)
        {
            Vector3& sum = normalSum[remap[i]];
            sum.x += src.vertex[i].normal.x;
            sum.y += src.vertex[i].normal.y;
            sum.z += src.vertex[i].normal.z;
        }
    }
    for (size_t i = 0; i < kept.size(); ++i) {
        Vector3 nrm = normalSum[i];
        if (pm::VectorNormalize(nrm) > 0.0f)
            kept[i].normal = nrm;
    }

    // faces are mesh-relative, so go through absolute indices; face order and
    // per-material face ranges are untouched, only the vertex ranges move
    std::vector<SrcFace> faces = src.face;
    for (int m = 0; m < static_cast<int>(src.mesh.size()); ++m) {
        const SrcMesh& mesh = src.mesh[m];
        for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
            faces[f].a = static_cast<uint32_t>(remap[src.face[f].a + mesh.vertexoffset]);
            faces[f].b = static_cast<uint32_t>(remap[src.face[f].b + mesh.vertexoffset]);
            faces[f].c = static_cast<uint32_t>(remap[src.face[f].c + mesh.vertexoffset]);
        }
    }

    src.vertex.swap(kept);
    const int newCount = static_cast<int>(src.vertex.size());
    for (int m = 0; m < static_cast<int>(src.mesh.size()); ++m) {
        SrcMesh& mesh = src.mesh[m];
        if (!mesh.numvertices)
            continue;
        mesh.numvertices = 0;
        mesh.vertexoffset = newCount;
    }
    for (int i = 0; i < newCount; ++i) {
        SrcMesh& mesh = src.mesh[src.vertex[i].material];
        mesh.numvertices++;
        if (mesh.vertexoffset > i)
            mesh.vertexoffset = i;
    }
    for (int m = 0; m < static_cast<int>(src.mesh.size()); ++m) {
        const SrcMesh& mesh = src.mesh[m];
        for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
            src.face[f].a = faces[f].a - mesh.vertexoffset;
            src.face[f].b = faces[f].b - mesh.vertexoffset;
            src.face[f].c = faces[f].c - mesh.vertexoffset;
        }
    }

    // a morph delta on a welded-away vertex lands on its survivor; the first
    // one wins, the rest would fight over the same vertex
    for (SrcMorphAnim& morph : src.morphs) {
        std::vector<char> seen(newCount, 0);
        std::vector<SrcVertAnim> out;
        out.reserve(morph.vanims.size());
        for (SrcVertAnim va : morph.vanims) {
            if (va.vertex < 0 || va.vertex >= n)
                continue;
            va.vertex = remap[va.vertex];
            if (seen[va.vertex])
                continue;
            seen[va.vertex] = 1;
            out.push_back(va);
        }
        morph.vanims.swap(out);
    }

    CalcModelTangentSpaces(src);
}

bool InflateVertices(Source& src, float amount, int meshTag,
                     const std::vector<std::string>& materials, const MaterialTable* mats) {
    std::vector<char> pick(src.vertex.size(), 1);
    if (meshTag > 0) {
        std::fill(pick.begin(), pick.end(), 0);
        for (int m = 0; m < src.nummeshes; ++m) {
            const SrcMesh& mesh = src.mesh[src.meshindex[m]];
            for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
                if (f >= static_cast<int>(src.faceTag.size()) || (src.faceTag[f] >> 8) != meshTag)
                    continue;
                const SrcFace& face = src.face[f];
                pick[mesh.vertexoffset + face.a] = pick[mesh.vertexoffset + face.b] =
                    pick[mesh.vertexoffset + face.c] = 1;
            }
        }
    }
    if (!materials.empty() && mats) {
        for (size_t i = 0; i < src.vertex.size(); ++i) {
            const int mat = src.vertex[i].material;
            if (mat < 0 || mat >= static_cast<int>(mats->materialToTexture.size())) {
                pick[i] = 0;
                continue;
            }
            std::string hay = mats->textures[mats->materialToTexture[mat]].name;
            std::transform(hay.begin(), hay.end(), hay.begin(), ::tolower);
            const std::string base = std::filesystem::path(hay).stem().string();
            bool hit = false;
            for (std::string n : materials) {
                std::transform(n.begin(), n.end(), n.begin(), ::tolower);
                hit = hit || n == hay || n == base;
            }
            pick[i] = pick[i] && hit;
        }
    }
    if (std::find(pick.begin(), pick.end(), 1) == pick.end())
        return src.vertex.empty();
    if (amount == 0.0f)
        return true;
    for (SrcMorphAnim& morph : src.morphs) {
        for (SrcVertAnim& va : morph.vanims) {
            if (va.vertex < 0 || static_cast<size_t>(va.vertex) >= src.vertex.size() ||
                !pick[va.vertex])
                continue;
            Vector3 base = src.vertex[va.vertex].normal;
            Vector3 target{base.x + va.normal.x, base.y + va.normal.y, base.z + va.normal.z};
            pm::VectorNormalize(base);
            pm::VectorNormalize(target);
            va.pos.x += amount * (target.x - base.x);
            va.pos.y += amount * (target.y - base.y);
            va.pos.z += amount * (target.z - base.z);
        }
    }
    for (size_t i = 0; i < src.vertex.size(); ++i) {
        if (!pick[i])
            continue;
        SrcVertex& v = src.vertex[i];
        Vector3 normal = v.normal;
        pm::VectorNormalize(normal);
        v.position.x += amount * normal.x;
        v.position.y += amount * normal.y;
        v.position.z += amount * normal.z;
    }
    CalcModelTangentSpaces(src);
    return true;
}

void FlipNormals(Source& src) {
    for (SrcVertex& v : src.vertex) {
        v.normal = {-v.normal.x, -v.normal.y, -v.normal.z};
        v.tangentS.w = -v.tangentS.w;
    }
    for (SrcMorphAnim& morph : src.morphs)
        for (SrcVertAnim& va : morph.vanims)
            va.normal = {-va.normal.x, -va.normal.y, -va.normal.z};
    for (SrcFace& face : src.face)
        std::swap(face.b, face.c);
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

void SimplifyFaces(Source& dst, const Source& src, float factor, bool lockBorder,
                   const std::vector<bool>* skipMaterial, int onlyTag, bool rigAware) {
    // globalVertices is empty until RemapVerticesToGlobalBones, so the
    // simplifier normally sees bind-space positions - the same input the
    // reference LOD path feeds it.
    const bool haveGlobal = src.globalVertices.size() >= src.vertex.size();
    const SrcVertex* pVertBase =
        haveGlobal ? src.globalVertices.data() : src.vertex.data();
    const size_t nAvailVerts =
        haveGlobal ? src.globalVertices.size() : src.vertex.size();

    const unsigned int options = lockBorder ? meshopt_SimplifyLockBorder : 0u;
    rigAware = rigAware && src.numbones > 1 &&
               src.boneToPose.size() >= static_cast<size_t>(src.numbones);

    // heap, not stack - kMaxSkins vectors is far past a thread's stack
    std::vector<std::vector<SrcFace>> meshFaces(lim::kMaxSkins);
    std::vector<std::vector<uint16_t>> meshTags(lim::kMaxSkins);
    const bool tagged = !src.faceTag.empty();
    float resultError = 0.0f;

    for (int mi = 0; mi < src.nummeshes; mi++) {
        const int matID = src.meshindex[mi];
        const SrcMesh& srcMesh = src.mesh[matID];
        if (srcMesh.numfaces == 0 || srcMesh.numvertices == 0 || nAvailVerts == 0)
            continue;
        if (skipMaterial && matID < static_cast<int>(skipMaterial->size()) &&
            (*skipMaterial)[matID])
            continue;

        // each tag group simplifies on its own so the tags stay exact
        std::vector<std::vector<unsigned int>> groups(tagged ? 256 : 1);
        for (int fi = 0; fi < srcMesh.numfaces; fi++) {
            const int f = srcMesh.faceoffset + fi;
            const SrcFace& face = src.face[f];
            std::vector<unsigned int>& grp = groups[tagged ? (src.faceTag[f] & 0xFF) : 0];
            grp.insert(grp.end(), {face.a, face.b, face.c});
        }

        // face indices are mesh-local, so hand over the mesh's own vertex slice.
        // SrcVertex leads with position, so &position + sizeof(SrcVertex) stride
        // walks the array correctly.
        const float* pPositions =
            reinterpret_cast<const float*>(&pVertBase[srcMesh.vertexoffset].position);

        // skin centroid (weighted bind-pose bone origins) as a quadric attribute,
        // so collapses across bones cost more and small limbs like fingers survive
        std::vector<float> skinAttr;
        if (rigAware) {
            const SrcVertex* mv = &pVertBase[srcMesh.vertexoffset];
            float lo[3] = {FLT_MAX, FLT_MAX, FLT_MAX}, hi[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
            for (int vi = 0; vi < srcMesh.numvertices; vi++) {
                const float p[3] = {mv[vi].position.x, mv[vi].position.y, mv[vi].position.z};
                for (int k = 0; k < 3; k++) {
                    lo[k] = std::min(lo[k], p[k]);
                    hi[k] = std::max(hi[k], p[k]);
                }
            }
            const float extent = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
            const float inv = extent > 0.0f ? 1.0f / extent : 0.0f;
            skinAttr.assign(static_cast<size_t>(srcMesh.numvertices) * 3, 0.0f);
            for (int vi = 0; vi < srcMesh.numvertices; vi++) {
                const auto& bw = mv[vi].boneweight;
                float* a = &skinAttr[static_cast<size_t>(vi) * 3];
                for (int i = 0; i < bw.numbones; i++) {
                    const int b = bw.bone[i];
                    if (b < 0 || b >= src.numbones)
                        continue;
                    const auto& m = src.boneToPose[b].m;
                    for (int k = 0; k < 3; k++)
                        a[k] += bw.weight[i] * m[k][3] * inv;
                }
            }
        }
        static const float kSkinWeights[3] = {1.0f, 1.0f, 1.0f};

        for (size_t g = 0; g < groups.size(); g++) {
            std::vector<unsigned int>& srcIdx = groups[g];
            if (srcIdx.empty())
                continue;
            std::vector<unsigned int> dstIdx(srcIdx.size());
            size_t newIdxCount = srcIdx.size();
            if (onlyTag >= 0 && static_cast<int>(g) != onlyTag) {
                dstIdx = srcIdx;
            } else {
                size_t targetIdx = static_cast<size_t>(static_cast<float>(srcIdx.size()) * factor);
                targetIdx = (targetIdx / 3) * 3;
                if (targetIdx < 3)
                    targetIdx = 3;
                if (rigAware)
                    newIdxCount = meshopt_simplifyWithAttributes(
                        dstIdx.data(), srcIdx.data(), srcIdx.size(), pPositions,
                        static_cast<size_t>(srcMesh.numvertices), sizeof(SrcVertex),
                        skinAttr.data(), 3 * sizeof(float), kSkinWeights, 3, nullptr, targetIdx,
                        1.0f, options | meshopt_SimplifyRegularize, &resultError);
                else
                    newIdxCount = meshopt_simplify(
                        dstIdx.data(), srcIdx.data(), srcIdx.size(), pPositions,
                        static_cast<size_t>(srcMesh.numvertices), sizeof(SrcVertex), targetIdx,
                        1.0f, options, &resultError);
            }
            for (size_t fi = 0; fi < newIdxCount / 3; fi++) {
                meshFaces[matID].push_back({dstIdx[fi * 3 + 0], dstIdx[fi * 3 + 1], dstIdx[fi * 3 + 2]});
                meshTags[matID].push_back(static_cast<uint16_t>(g));
            }
        }
    }

    // flatten back into one face array, meshes in the source's own order
    dst.face.clear();
    dst.faceTag.clear();
    for (int mi = 0; mi < src.nummeshes; mi++) {
        const int matID = src.meshindex[mi];
        SrcMesh& dstMesh = dst.mesh[matID];
        dstMesh.faceoffset = static_cast<int>(dst.face.size());
        dstMesh.numfaces = static_cast<int>(meshFaces[matID].size());
        dst.face.insert(dst.face.end(), meshFaces[matID].begin(), meshFaces[matID].end());
        if (tagged)
            dst.faceTag.insert(dst.faceTag.end(), meshTags[matID].begin(), meshTags[matID].end());
    }
}

void RemoveSmallMeshes(Source& src, float limit) {
    std::vector<SrcFace> faces;
    std::vector<uint16_t> tags;
    const bool tagged = !src.faceTag.empty();

    for (int mi = 0; mi < src.nummeshes; ++mi) {
        SrcMesh& mesh = src.mesh[src.meshindex[mi]];
        const int firstFace = mesh.faceoffset;
        std::map<std::tuple<float, float, float>, std::vector<int>> positionFaces;
        for (int fi = 0; fi < mesh.numfaces; ++fi) {
            const SrcFace& face = src.face[firstFace + fi];
            for (uint32_t vi : {face.a, face.b, face.c}) {
                const Vector3& p = src.vertex[mesh.vertexoffset + vi].position;
                positionFaces[{p.x, p.y, p.z}].push_back(fi);
            }
        }

        std::vector<char> seen(mesh.numfaces);
        mesh.faceoffset = static_cast<int>(faces.size());
        mesh.numfaces = 0;
        for (int start = 0; start < static_cast<int>(seen.size()); ++start) {
            if (seen[start])
                continue;
            std::vector<int> component{start};
            seen[start] = 1;
            Vector3 lo = src.vertex[mesh.vertexoffset + src.face[firstFace + start].a].position;
            Vector3 hi = lo;
            for (size_t i = 0; i < component.size(); ++i) {
                const SrcFace& face = src.face[firstFace + component[i]];
                for (uint32_t vi : {face.a, face.b, face.c}) {
                    const Vector3& p = src.vertex[mesh.vertexoffset + vi].position;
                    lo.x = std::min(lo.x, p.x);
                    lo.y = std::min(lo.y, p.y);
                    lo.z = std::min(lo.z, p.z);
                    hi.x = std::max(hi.x, p.x);
                    hi.y = std::max(hi.y, p.y);
                    hi.z = std::max(hi.z, p.z);
                    for (int next : positionFaces[{p.x, p.y, p.z}])
                        if (!seen[next]) {
                            seen[next] = 1;
                            component.push_back(next);
                        }
                }
            }
            if (std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z}) < limit)
                continue;
            for (int fi : component) {
                faces.push_back(src.face[firstFace + fi]);
                if (tagged)
                    tags.push_back(src.faceTag[firstFace + fi]);
                ++mesh.numfaces;
            }
        }
    }
    src.face.swap(faces);
    if (tagged)
        src.faceTag.swap(tags);
}

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

    FillMeshes(out, faces);

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

bool AddToonOutline(Source& src, MaterialTable& mats, const ToonOutlineOption& o, std::string* err) {
    auto fail = [&](const std::string& m) {
        if (err) *err = m;
        return false;
    };
    const int n = static_cast<int>(src.vertex.size());
    for (SrcVertex& v : src.vertex)
        if (v.outline < 0.0f)
            v.outline = o.forceUse ? 1.0f : 0.0f;

    std::string cd = o.cdmaterial;
    if (!cd.empty() && cd.back() != '/' && cd.back() != '\\')
        cd += '/';
    std::map<int, int> outMat;
    auto outlineMaterial = [&](int m) {
        auto it = outMat.find(m);
        if (it != outMat.end())
            return it->second;
        std::string name = cd + o.material;
        if (o.perMaterial) {
            const std::string& base = mats.textures[mats.materialToTexture[m]].name;
            const size_t slash = base.find_last_of("/\\");
            const std::string dir = !cd.empty() ? cd : slash == std::string::npos ? "" : base.substr(0, slash + 1);
            name = dir + std::filesystem::path(base).stem().string() + "_toonoutline";
        }
        return outMat[m] = mats.UseTextureAsMaterial(mats.LookupTexture(name.c_str()));
    };

    // weld: one push direction per position, so split normals cannot crack the hull
    std::vector<Vector3> push(n);
    std::map<std::tuple<float, float, float>, Vector3> byPos;
    for (int i = 0; i < n; ++i) {
        push[i] = src.vertex[i].normal;
        pm::VectorNormalize(push[i]);
        if (o.weld) {
            const Vector3& p = src.vertex[i].position;
            Vector3& s = byPos[{p.x, p.y, p.z}];
            s.x += push[i].x;
            s.y += push[i].y;
            s.z += push[i].z;
        }
    }
    if (o.weld)
        for (int i = 0; i < n; ++i) {
            const Vector3& p = src.vertex[i].position;
            Vector3 s = byPos[{p.x, p.y, p.z}];
            if (pm::VectorNormalize(s) > 0.0f)
                push[i] = s;
        }

    std::vector<SrcVertex> verts = src.vertex;
    std::vector<int> origin; // per outline vertex, the source vertex it copies
    std::vector<int> noWeldOf(n, -1);
    auto less = [](const SrcVertex& a, const SrcVertex& b) { return WeldCompare(a, b, WeldMode::All) < 0; };
    std::map<SrcVertex, int, decltype(less)> welded(less);
    auto outlineVertex = [&](int v, int om) {
        if (!o.weld && noWeldOf[v] >= 0)
            return noWeldOf[v];
        SrcVertex ov = src.vertex[v];
        const float d = o.thickness * ov.outline;
        ov.material = om;
        ov.position = {ov.position.x + push[v].x * d, ov.position.y + push[v].y * d,
                       ov.position.z + push[v].z * d};
        ov.normal = {-push[v].x, -push[v].y, -push[v].z};
        if (o.weld) {
            auto it = welded.find(ov);
            if (it != welded.end())
                return it->second;
        }
        const int idx = static_cast<int>(verts.size());
        verts.push_back(ov);
        origin.push_back(v);
        if (o.weld)
            welded.emplace(ov, idx);
        else
            noWeldOf[v] = idx;
        return idx;
    };

    std::vector<MFace> faces;
    const bool tagged = !src.faceTag.empty();
    for (int mi = 0; mi < src.nummeshes; ++mi) {
        const int m = src.meshindex[mi];
        const SrcMesh& mesh = src.mesh[m];
        for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f)
            faces.push_back({m, mesh.vertexoffset + src.face[f].a, mesh.vertexoffset + src.face[f].b,
                             mesh.vertexoffset + src.face[f].c, tagged ? src.faceTag[f] : uint16_t(0)});
    }
    const size_t numOrig = faces.size();
    auto inRange = [&](uint32_t v) {
        const float w = src.vertex[v].outline;
        return w >= o.minWeight && w <= o.maxWeight;
    };
    for (size_t f = 0; f < numOrig; ++f) {
        const MFace face = faces[f];
        if (!inRange(face.a) || !inRange(face.b) || !inRange(face.c))
            continue;
        const int om = outlineMaterial(face.material);
        if (om >= lim::kMaxSkins)
            return fail("too many materials (max " + std::to_string(lim::kMaxSkins) + ")");
        // inverse hull: reversed winding so only the back faces draw
        faces.push_back({om, static_cast<uint32_t>(outlineVertex(face.a, om)),
                         static_cast<uint32_t>(outlineVertex(face.c, om)),
                         static_cast<uint32_t>(outlineVertex(face.b, om))});
    }
    if (faces.size() == numOrig)
        return fail("min/max weight culls every face");
    if (verts.size() > static_cast<size_t>(lim::kMaxVerts))
        return fail("outlined mesh has " + std::to_string(verts.size()) + " vertices (max " +
                    std::to_string(lim::kMaxVerts) + ")");
    if (faces.size() > static_cast<size_t>(lim::kMaxTriangles))
        return fail("outlined mesh has " + std::to_string(faces.size()) + " triangles (max " +
                    std::to_string(lim::kMaxTriangles) + ")");

    // the hull follows every flex its source vertex takes part in, re-aiming its
    // offset along the flexed normal the way $inflate does
    for (SrcMorphAnim& morph : src.morphs) {
        std::vector<int> at(n, -1);
        for (size_t i = 0; i < morph.vanims.size(); ++i)
            if (morph.vanims[i].vertex >= 0 && morph.vanims[i].vertex < n)
                at[morph.vanims[i].vertex] = static_cast<int>(i);
        for (size_t k = 0; k < origin.size(); ++k) {
            const int v = origin[k];
            if (at[v] < 0)
                continue;
            SrcVertAnim va = morph.vanims[at[v]];
            const float d = o.thickness * src.vertex[v].outline;
            Vector3 flexed{push[v].x + va.normal.x, push[v].y + va.normal.y, push[v].z + va.normal.z};
            if (pm::VectorNormalize(flexed) > 0.0f) {
                va.pos.x += d * (flexed.x - push[v].x);
                va.pos.y += d * (flexed.y - push[v].y);
                va.pos.z += d * (flexed.z - push[v].z);
            }
            va.vertex = n + static_cast<int>(k);
            va.normal = {-va.normal.x, -va.normal.y, -va.normal.z};
            morph.vanims.push_back(va);
        }
    }

    // regroup by material; the source's own vertices are already sorted
    std::vector<int> order(verts.size());
    for (size_t i = 0; i < order.size(); ++i)
        order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return verts[a].material < verts[b].material; });
    std::vector<int> remap(verts.size());
    src.vertex.resize(verts.size());
    for (size_t i = 0; i < order.size(); ++i) {
        remap[order[i]] = static_cast<int>(i);
        src.vertex[i] = verts[order[i]];
    }
    for (MFace& f : faces) {
        f.a = remap[f.a];
        f.b = remap[f.b];
        f.c = remap[f.c];
    }
    std::stable_sort(faces.begin(), faces.end(),
                     [](const MFace& a, const MFace& b) { return a.material < b.material; });
    for (SrcMorphAnim& morph : src.morphs)
        for (SrcVertAnim& va : morph.vanims)
            if (va.vertex >= 0 && va.vertex < static_cast<int>(remap.size()))
                va.vertex = remap[va.vertex];

    FillMeshes(src, faces);
    CalcModelTangentSpaces(src);
    return true;
}


bool ScaleBone(Source& src, const std::string& bone, const Vector3& scale) {
    int b = -1;
    for (int i = 0; i < src.numbones && b < 0; ++i)
        if (_stricmp(src.localBone[i].name.c_str(), bone.c_str()) == 0)
            b = i;
    if (b < 0 || src.boneToPose.size() != static_cast<size_t>(src.numbones))
        return false;

    // localBone is a DFS, so a parent is marked before its children
    std::vector<char> sub(src.numbones, 0);
    sub[b] = 1;
    for (int i = b + 1; i < src.numbones; ++i)
        sub[i] = src.localBone[i].parent >= 0 && sub[src.localBone[i].parent];

    // Scale about the bone's origin along its own axes; normals take the inverse scale.
    const pm::matrix3x4& B = src.boneToPose[b];
    auto mul = [](const Vector3& v, const Vector3& s) { return Vector3{v.x * s.x, v.y * s.y, v.z * s.z}; };
    const Vector3 inv{1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};
    auto point = [&](const Vector3& v) { return pm::VectorTransform(mul(pm::VectorITransform(v, B), scale), B); };
    auto dir = [&](const Vector3& v, const Vector3& s) { return pm::VectorRotate(mul(pm::VectorIRotate(v, B), s), B); };
    auto lerp = [](const Vector3& a, const Vector3& b, float t) {
        return Vector3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    };
    auto unit = [](Vector3 v) {
        const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return l > 0.0f ? Vector3{v.x / l, v.y / l, v.z / l} : v;
    };

    std::vector<float> w(src.vertex.size(), 0.0f); // weight share inside the scaled subtree
    for (size_t i = 0; i < src.vertex.size(); ++i) {
        const SrcBoneWeight& bw = src.vertex[i].boneweight;
        for (int k = 0; k < bw.numbones; ++k)
            if (bw.bone[k] >= 0 && bw.bone[k] < src.numbones && sub[bw.bone[k]])
                w[i] += bw.weight[k];
        if (w[i] <= 0.0f)
            continue;
        SrcVertex& v = src.vertex[i];
        v.position = lerp(v.position, point(v.position), w[i]);
        v.normal = unit(lerp(v.normal, unit(dir(v.normal, inv)), w[i]));
        const Vector3 t = unit(lerp({v.tangentS.x, v.tangentS.y, v.tangentS.z},
                                    unit(dir({v.tangentS.x, v.tangentS.y, v.tangentS.z}, scale)), w[i]));
        v.tangentS = {t.x, t.y, t.z, v.tangentS.w};
    }
    for (SrcMorphAnim& morph : src.morphs)
        for (SrcVertAnim& va : morph.vanims) {
            if (va.vertex < 0 || static_cast<size_t>(va.vertex) >= w.size() || w[va.vertex] <= 0.0f)
                continue;
            va.pos = lerp(va.pos, dir(va.pos, scale), w[va.vertex]);
            va.normal = lerp(va.normal, dir(va.normal, inv), w[va.vertex]);
        }

    // Descendant bones ride the scale: move their bind origins, then re-localize positions.
    for (int i = b + 1; i < src.numbones; ++i) {
        if (!sub[i])
            continue;
        pm::matrix3x4& m = src.boneToPose[i];
        const Vector3 o = point({m.m[0][3], m.m[1][3], m.m[2][3]});
        m.m[0][3] = o.x; m.m[1][3] = o.y; m.m[2][3] = o.z;
    }
    for (SourceAnim& anim : src.anims)
        for (std::vector<SrcBonePose>& frame : anim.frames)
            for (int i = b + 1; i < src.numbones && i < static_cast<int>(frame.size()); ++i) {
                const int p = src.localBone[i].parent;
                if (!sub[i] || p < 0)
                    continue;
                const pm::matrix3x4& P = src.boneToPose[p];
                const Vector3 d = pm::VectorRotate(frame[i].pos, P); // world offset in bind orientation
                frame[i].pos = pm::VectorIRotate(dir(d, scale), P);
            }
    return true;
}

} // namespace pulse::source
