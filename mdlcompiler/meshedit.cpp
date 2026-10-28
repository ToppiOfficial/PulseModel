// meshedit.cpp - $rendermesh edit options. See meshedit.h.

#include "strcompat.h"
#include "diagnostic.h"

#include "meshedit.h"

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
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

int FindWeightBone(const Source& src, const std::string& name) {
    for (int i = 0; i < src.numbones; ++i)
        if (_stricmp(src.localBone[i].name.c_str(), name.c_str()) == 0) return i;
    return -1;
}

int FindWeightSlot(const SrcBoneWeight& weights, int bone) {
    for (int i = 0; i < weights.numbones; ++i)
        if (weights.bone[i] == bone) return i;
    return -1;
}

bool ValidBoneWeights(const Source& src, const SrcBoneWeight& weights) {
    if (weights.numbones <= 0 || weights.numbones > lim::kMaxSrcBoneWeights) return false;
    for (int i = 0; i < weights.numbones; ++i)
        if (weights.bone[i] < 0 || weights.bone[i] >= src.numbones ||
            !std::isfinite(weights.weight[i]) || weights.weight[i] < 0.0f)
            return false;
    return true;
}

enum class WeightEditResult { Applied, NoRecipient, Full };

WeightEditResult SetBoneWeight(SrcBoneWeight& weights, int bone, float value) {
    if (value == 1.0f) {
        weights.numbones = 1;
        weights.bone[0] = bone;
        weights.weight[0] = 1.0f;
        return WeightEditResult::Applied;
    }

    const int slot = FindWeightSlot(weights, bone);
    if (slot < 0 && value == 0.0f) return WeightEditResult::Applied;

    double otherWeight = 0.0;
    int otherCount = 0;
    for (int i = 0; i < weights.numbones; ++i) {
        if (i == slot || weights.weight[i] <= 0.0f) continue;
        otherWeight += weights.weight[i];
        ++otherCount;
    }
    if (otherWeight == 0.0) return WeightEditResult::NoRecipient;
    if (value > 0.0f && otherCount >= lim::kMaxSrcBoneWeights) return WeightEditResult::Full;

    SrcBoneWeight updated;
    for (int i = 0; i < weights.numbones; ++i) {
        const float weight = i == slot ? value :
            static_cast<float>(weights.weight[i] * ((1.0 - value) / otherWeight));
        if (weight <= 0.0f) continue;
        const int next = updated.numbones++;
        updated.bone[next] = weights.bone[i];
        updated.weight[next] = weight;
    }
    if (slot < 0 && value > 0.0f) {
        const int next = updated.numbones++;
        updated.bone[next] = bone;
        updated.weight[next] = value;
    }
    weights = updated;
    return WeightEditResult::Applied;
}

WeightEditResult TransferBoneWeight(SrcBoneWeight& weights, int slot, int residual, float moved) {
    const float kept = weights.weight[slot] - moved;
    if (residual < 0) return SetBoneWeight(weights, weights.bone[slot], kept);

    const int residualSlot = FindWeightSlot(weights, residual);
    int count = 0;
    for (int i = 0; i < weights.numbones; ++i)
        if (weights.weight[i] > 0.0f) ++count;
    if (residualSlot < 0 && kept > 0.0f && count >= lim::kMaxSrcBoneWeights)
        return WeightEditResult::Full;

    SrcBoneWeight updated;
    for (int i = 0; i < weights.numbones; ++i) {
        float weight = i == slot ? kept : weights.weight[i];
        if (i == residualSlot) weight += moved;
        if (weight <= 0.0f) continue;
        const int next = updated.numbones++;
        updated.bone[next] = weights.bone[i];
        updated.weight[next] = weight;
    }
    if (residualSlot < 0) {
        const int next = updated.numbones++;
        updated.bone[next] = residual;
        updated.weight[next] = moved;
    }
    weights = updated;
    return WeightEditResult::Applied;
}

bool MaterialMatches(const std::string& material, const std::vector<std::string>& names) {
    const std::string base = std::filesystem::path(material).stem().string();
    for (const auto& name : names)
        if (_stricmp(name.c_str(), material.c_str()) == 0 || _stricmp(name.c_str(), base.c_str()) == 0)
            return true;
    return false;
}

bool WeldAttributesMatch(const SrcVertex& a, const SrcVertex& b) {
    if (a.material != b.material || a.boneweight.numbones != b.boneweight.numbones)
        return false;
    for (int i = 0; i < a.boneweight.numbones; ++i) {
        if (a.boneweight.bone[i] != b.boneweight.bone[i] ||
            std::memcmp(&a.boneweight.weight[i], &b.boneweight.weight[i], sizeof(float)) != 0)
            return false;
    }
    return true;
}

bool DistanceWeldTargets(Source& src, const std::vector<char>& selection,
                         float distance, bool sharpEdges,
                         std::vector<int>& targets) {
    std::vector<int> order;
    for (size_t i = 0; i < src.vertex.size(); ++i)
        if (selection[i]) order.push_back(static_cast<int>(i));

    std::vector<Vector3> positions;
    positions.reserve(src.vertex.size());
    for (const auto& vertex : src.vertex) positions.push_back(vertex.position);
    auto coordinate = [&](int index, int axis) {
        const auto& p = positions[index];
        return axis == 0 ? p.x : axis == 1 ? p.y : p.z;
    };
    auto build = [&](auto&& self, int begin, int end, int axis) -> void {
        if (begin >= end) return;
        const int middle = begin + (end - begin) / 2;
        std::nth_element(order.begin() + begin, order.begin() + middle, order.begin() + end,
                         [&](int a, int b) {
                             const float ca = coordinate(a, axis), cb = coordinate(b, axis);
                             return ca < cb || (ca == cb && a < b);
                         });
        self(self, begin, middle, (axis + 1) % 3);
        self(self, middle + 1, end, (axis + 1) % 3);
    };
    build(build, 0, static_cast<int>(order.size()), 0);

    const float radiusSquared = distance * distance;
    auto squaredDistance = [](const Vector3& a, const Vector3& b) {
        const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
        return x * x + y * y + z * z;
    };
    targets.assign(src.vertex.size(), -1);
    std::vector<int> cluster, normalHeads;
    bool changed = false;
    for (size_t seed = 0; seed < src.vertex.size(); ++seed) {
        if (!selection[seed] || targets[seed] >= 0) continue;
        cluster.clear();
        auto gather = [&](auto&& self, int begin, int end, int axis) -> void {
            if (begin >= end) return;
            const int middle = begin + (end - begin) / 2, candidate = order[middle];
            const float delta = coordinate(static_cast<int>(seed), axis) - coordinate(candidate, axis);
            if (candidate != static_cast<int>(seed) && targets[candidate] < 0 &&
                WeldAttributesMatch(src.vertex[seed], src.vertex[candidate]) &&
                squaredDistance(positions[seed], positions[candidate]) <= radiusSquared)
                cluster.push_back(candidate);
            if (delta <= 0.0f || delta * delta <= radiusSquared)
                self(self, begin, middle, (axis + 1) % 3);
            if (delta >= 0.0f || delta * delta <= radiusSquared)
                self(self, middle + 1, end, (axis + 1) % 3);
        };
        gather(gather, 0, static_cast<int>(order.size()), 0);
        if (cluster.empty()) {
            targets[seed] = static_cast<int>(seed);
            continue;
        }
        changed = true;
        std::sort(cluster.begin(), cluster.end());
        cluster.push_back(static_cast<int>(seed));

        Vector3 center{};
        for (int index : cluster) {
            center.x += positions[index].x;
            center.y += positions[index].y;
            center.z += positions[index].z;
        }
        const float count = static_cast<float>(cluster.size());
        center = {center.x / count, center.y / count, center.z / count};
        int survivor = static_cast<int>(seed);
        for (int index : cluster) {
            const float candidateDistance = squaredDistance(center, positions[index]);
            const float bestDistance = squaredDistance(center, positions[survivor]);
            if (candidateDistance < bestDistance || (candidateDistance == bestDistance && index < survivor))
                survivor = index;
        }
        if (cluster.size() == 2) survivor = std::min(cluster[0], cluster[1]);

        normalHeads.clear();
        normalHeads.push_back(survivor);
        for (int index : cluster) {
            int head = survivor;
            if (sharpEdges) {
                head = -1;
                for (int normalHead : normalHeads) {
                    const auto& a = src.vertex[index].normal;
                    const auto& b = src.vertex[normalHead].normal;
                    if (a.x == b.x && a.y == b.y && a.z == b.z) {
                        head = normalHead;
                        break;
                    }
                }
                if (head < 0) {
                    head = index;
                    normalHeads.push_back(head);
                }
            }
            targets[index] = head;
            src.vertex[index].position = center;
        }
    }
    for (size_t i = 0; i < targets.size(); ++i)
        if (targets[i] < 0) targets[i] = static_cast<int>(i);
    return changed;
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

void InflationVectors(const Source& src, float thickness, float thicknessClamp,
                      const std::vector<char>& selection, std::vector<Vector3>& push,
                      std::vector<float>& clampScale) {
    const int n = static_cast<int>(src.vertex.size());

    // Push along Blender solidify's high quality normal, built from face geometry
    // per position so authored normals cannot bend the hull: each edge takes the
    // sum of its (at most two) face normals, each position the sum of its edges.
    using PosKey = std::tuple<float, float, float>;
    auto posKey = [&](uint32_t v) {
        const Vector3& p = src.vertex[v].position;
        return std::make_tuple(p.x, p.y, p.z);
    };
    struct EdgeFaces {
        Vector3 sum{0.0f, 0.0f, 0.0f};
        int count = 0;
    };
    std::map<std::pair<PosKey, PosKey>, EdgeFaces> edges;
    for (int mi = 0; mi < src.nummeshes; ++mi) {
        const SrcMesh& mesh = src.mesh[src.meshindex[mi]];
        for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
            const uint32_t c[3] = {mesh.vertexoffset + src.face[f].a, mesh.vertexoffset + src.face[f].b,
                                   mesh.vertexoffset + src.face[f].c};
            if (!selection[c[0]] && !selection[c[1]] && !selection[c[2]])
                continue;
            const Vector3 &p0 = src.vertex[c[0]].position, &p1 = src.vertex[c[1]].position,
                          &p2 = src.vertex[c[2]].position;
            const Vector3 e1{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z}, e2{p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
            // e2 x e1: faces are stored in the reference's (v1, v3, v2) winding
            Vector3 fn{e2.y * e1.z - e2.z * e1.y, e2.z * e1.x - e2.x * e1.z, e2.x * e1.y - e2.y * e1.x};
            if (pm::VectorNormalize(fn) <= 0.0f)
                continue;
            for (int k = 0; k < 3; ++k) {
                PosKey a = posKey(c[k]), b = posKey(c[(k + 1) % 3]);
                if (a == b)
                    continue;
                if (b < a)
                    std::swap(a, b);
                EdgeFaces& e = edges[{a, b}];
                e.sum = {e.sum.x + fn.x, e.sum.y + fn.y, e.sum.z + fn.z};
                ++e.count;
            }
        }
    }
    std::map<PosKey, Vector3> byPos;
    for (auto& [ends, e] : edges) {
        if (e.count > 2 || pm::VectorNormalize(e.sum) <= 0.0f)
            continue; // non-manifold edges add nothing
        for (const PosKey& p : {ends.first, ends.second}) {
            Vector3& s = byPos[p];
            s = {s.x + e.sum.x, s.y + e.sum.y, s.z + e.sum.z};
        }
    }
    push.assign(n, Vector3{});
    for (int i = 0; i < n; ++i) {
        auto it = byPos.find(posKey(i));
        Vector3 s = it != byPos.end() ? it->second : Vector3{0.0f, 0.0f, 0.0f};
        if (pm::VectorNormalize(s) <= 0.0f) {
            s = src.vertex[i].normal;
            pm::VectorNormalize(s);
        }
        push[i] = s;
    }

    // thicknessclamp: Blender solidify's Clamp. A vertex whose shortest edge is
    // under |thickness| * clamp scales by edge / that. Edges are measured
    // between positions, not split verts.
    clampScale.assign(n, 1.0f);
    const float clampOffset = std::fabs(thickness) * thicknessClamp;
    if (clampOffset > FLT_EPSILON) {
        std::map<PosKey, float> lenSq;
        auto key = posKey;
        for (int mi = 0; mi < src.nummeshes; ++mi) {
            const SrcMesh& mesh = src.mesh[src.meshindex[mi]];
            for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
                const uint32_t c[3] = {mesh.vertexoffset + src.face[f].a, mesh.vertexoffset + src.face[f].b,
                                  mesh.vertexoffset + src.face[f].c};
                if (!selection[c[0]] && !selection[c[1]] && !selection[c[2]])
                    continue;
                for (int k = 0; k < 3; ++k) {
                    const Vector3 &a = src.vertex[c[k]].position, &b = src.vertex[c[(k + 1) % 3]].position;
                    const float l = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
                    if (l == 0.0f)
                        continue;
                    for (uint32_t v : {c[k], c[(k + 1) % 3]}) {
                        auto it = lenSq.try_emplace(key(v), FLT_MAX).first;
                        it->second = std::min(it->second, l);
                    }
                }
            }
        }
        for (int i = 0; i < n; ++i) {
            auto it = lenSq.find(key(i));
            if (it != lenSq.end() && it->second < clampOffset * clampOffset)
                clampScale[i] = std::sqrt(it->second) / clampOffset;
        }
    }
}

void ApplyInflation(Source& src, const std::vector<char>& selection,
                    const std::vector<Vector3>& push, const std::vector<float>& distance,
                    bool hull = false) {
    for (SrcMorphAnim& morph : src.morphs) {
        for (SrcVertAnim& va : morph.vanims) {
            if (va.vertex < 0 || static_cast<size_t>(va.vertex) >= src.vertex.size() ||
                !selection[va.vertex]) continue;
            const Vector3& base = push[va.vertex];
            const Vector3& normal = hull ? base : src.vertex[va.vertex].normal;
            Vector3 target{normal.x + va.normal.x, normal.y + va.normal.y, normal.z + va.normal.z};
            pm::VectorNormalize(target);
            va.pos.x += distance[va.vertex] * (target.x - base.x);
            va.pos.y += distance[va.vertex] * (target.y - base.y);
            va.pos.z += distance[va.vertex] * (target.z - base.z);
        }
    }

    for (size_t i = 0; i < src.vertex.size(); ++i) {
        if (!selection[i]) continue;
        Vector3& position = src.vertex[i].position;
        position.x += distance[i] * push[i].x;
        position.y += distance[i] * push[i].y;
        position.z += distance[i] * push[i].z;
    }
    CalcModelTangentSpaces(src);
}

void ReindexEditVertices(Source& src, const std::vector<int>& origins, std::vector<char>& selection) {
    std::vector<int> head(src.vertex.size(), -1), next(origins.size(), -1);
    std::vector<SrcVertex> vertices;
    std::vector<char> picked;
    vertices.reserve(origins.size());
    picked.reserve(origins.size());
    for (int v : origins) {
        vertices.push_back(src.vertex[v]);
        picked.push_back(selection[v]);
    }
    for (size_t i = origins.size(); i-- > 0;) {
        next[i] = head[origins[i]];
        head[origins[i]] = static_cast<int>(i);
    }

    if (!src.vertexEditMesh.empty()) {
        std::vector<int> meshes;
        meshes.reserve(origins.size());
        for (int v : origins) meshes.push_back(src.vertexEditMesh[v]);
        std::vector<std::pair<int, int>> extras;
        for (const auto& [vertex, mesh] : src.extraEditMeshes)
            for (int v = head[vertex]; v >= 0; v = next[v]) extras.emplace_back(v, mesh);
        src.vertexEditMesh.swap(meshes);
        src.extraEditMeshes.swap(extras);
    }

    for (auto& stream : src.editStreams) {
        std::vector<float> values;
        values.reserve(origins.size());
        for (int v : origins) values.push_back(stream.values[v]);
        stream.values.swap(values);
    }
    for (auto& morph : src.morphs) {
        std::vector<SrcVertAnim> deltas;
        for (const auto& va : morph.vanims) {
            if (va.vertex < 0 || static_cast<size_t>(va.vertex) >= head.size()) continue;
            for (int v = head[va.vertex]; v >= 0; v = next[v]) {
                SrcVertAnim copy = va;
                copy.vertex = v;
                deltas.push_back(copy);
            }
        }
        morph.vanims.swap(deltas);
    }
    src.vertex.swap(vertices);
    selection.swap(picked);
}

std::vector<MFace> AbsoluteFaces(const Source& src) {
    std::vector<MFace> faces;
    faces.reserve(src.face.size());
    for (int mi = 0; mi < src.nummeshes; ++mi) {
        const int material = src.meshindex[mi];
        const auto& mesh = src.mesh[material];
        for (int f = mesh.faceoffset; f < mesh.faceoffset + mesh.numfaces; ++f) {
            const auto& face = src.face[f];
            faces.push_back({material, mesh.vertexoffset + face.a, mesh.vertexoffset + face.b,
                             mesh.vertexoffset + face.c, src.faceTag.empty() ? uint16_t(0) : src.faceTag[f]});
        }
    }
    return faces;
}

void RebuildEditMeshes(Source& src, std::vector<MFace>& faces, std::vector<char>& selection) {
    std::vector<int> origins(src.vertex.size()), remap(src.vertex.size());
    for (size_t i = 0; i < origins.size(); ++i) origins[i] = static_cast<int>(i);
    std::stable_sort(origins.begin(), origins.end(),
                     [&](int a, int b) { return src.vertex[a].material < src.vertex[b].material; });
    for (size_t i = 0; i < origins.size(); ++i) remap[origins[i]] = static_cast<int>(i);
    ReindexEditVertices(src, origins, selection);
    for (auto& face : faces) {
        face.a = remap[face.a];
        face.b = remap[face.b];
        face.c = remap[face.c];
    }
    std::stable_sort(faces.begin(), faces.end(),
                     [](const MFace& a, const MFace& b) { return a.material < b.material; });
    FillMeshes(src, faces);
}

void PruneEditVertices(Source& src, std::vector<MFace>& faces, std::vector<char>& selection) {
    std::vector<int> remap(src.vertex.size(), -1), origins;
    for (const auto& face : faces) remap[face.a] = remap[face.b] = remap[face.c] = 0;
    for (size_t i = 0; i < remap.size(); ++i) {
        if (remap[i] < 0) continue;
        remap[i] = static_cast<int>(origins.size());
        origins.push_back(static_cast<int>(i));
    }
    for (auto& face : faces) {
        face.a = remap[face.a];
        face.b = remap[face.b];
        face.c = remap[face.c];
    }
    ReindexEditVertices(src, origins, selection);
}

} // namespace

void WeldVertices(Source& src, std::vector<char>& selection, float distance, bool sharpEdges) {
    const int n = static_cast<int>(src.vertex.size());
    if (n == 0)
        return;

    std::vector<int> rep;
    if (!DistanceWeldTargets(src, selection, distance, sharpEdges, rep))
        return;

    std::vector<int> remap(n, -1);
    std::vector<SrcVertex> kept;
    kept.reserve(n);
    std::vector<Vector3> normalSum;
    std::vector<int> survivors;
    for (int i = 0; i < n; ++i) {
        if (rep[i] == i) {
            remap[i] = static_cast<int>(kept.size());
            kept.push_back(src.vertex[i]);
            normalSum.push_back(src.vertex[i].normal);
            survivors.push_back(i);
        }
    }
    for (int i = 0; i < n; ++i) {
        remap[i] = remap[rep[i]];
        if (rep[i] != i) {
            Vector3& sum = normalSum[remap[i]];
            sum.x += src.vertex[i].normal.x;
            sum.y += src.vertex[i].normal.y;
            sum.z += src.vertex[i].normal.z;
        }
    }
    for (size_t i = 0; i < kept.size(); ++i) {
        if (sharpEdges || !selection[survivors[i]]) continue;
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
    if (!src.vertexEditMesh.empty()) {
        std::vector<int> meshes;
        meshes.reserve(survivors.size());
        for (int v : survivors) meshes.push_back(src.vertexEditMesh[v]);

        std::vector<std::pair<int, int>> extras;
        for (int v = 0; v < n; ++v)
            if (src.vertexEditMesh[v] != meshes[remap[v]])
                extras.emplace_back(remap[v], src.vertexEditMesh[v]);
        for (const auto& [vertex, mesh] : src.extraEditMeshes)
            if (mesh != meshes[remap[vertex]]) extras.emplace_back(remap[vertex], mesh);
        std::sort(extras.begin(), extras.end());
        extras.erase(std::unique(extras.begin(), extras.end()), extras.end());
        src.vertexEditMesh.swap(meshes);
        src.extraEditMeshes.swap(extras);
    }
    for (auto& stream : src.editStreams) {
        std::vector<float> values;
        values.reserve(survivors.size());
        for (int v : survivors) values.push_back(stream.values[v]);
        stream.values.swap(values);
    }
    std::vector<char> keptSelection;
    keptSelection.reserve(survivors.size());
    for (int v : survivors) keptSelection.push_back(selection[v]);
    selection.swap(keptSelection);

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

    auto weldedFaces = AbsoluteFaces(src);
    std::set<std::tuple<int, uint32_t, uint32_t, uint32_t>> seenFaces;
    auto samePosition = [&](uint32_t a, uint32_t b) {
        const auto& p = src.vertex[a].position;
        const auto& q = src.vertex[b].position;
        return p.x == q.x && p.y == q.y && p.z == q.z;
    };
    weldedFaces.erase(std::remove_if(weldedFaces.begin(), weldedFaces.end(), [&](const MFace& face) {
        if (!selection[face.a] && !selection[face.b] && !selection[face.c])
            return false;
        if (samePosition(face.a, face.b) || samePosition(face.b, face.c) || samePosition(face.c, face.a))
            return true;
        uint32_t corners[] = {face.a, face.b, face.c};
        std::sort(corners, corners + 3);
        return !seenFaces.emplace(face.material, corners[0], corners[1], corners[2]).second;
    }), weldedFaces.end());

    PruneEditVertices(src, weldedFaces, selection);
    FillMeshes(src, weldedFaces);
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
            pick[i] = pick[i] && MaterialMatches(mats->textures[mats->materialToTexture[mat]].name, materials);
        }
    }
    if (std::find(pick.begin(), pick.end(), 1) == pick.end())
        return src.vertex.empty();
    if (amount == 0.0f)
        return true;

    std::vector<Vector3> push(src.vertex.size());
    for (size_t i = 0; i < push.size(); ++i) {
        push[i] = src.vertex[i].normal;
        pm::VectorNormalize(push[i]);
    }
    ApplyInflation(src, pick, push, std::vector<float>(push.size(), amount));
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

bool ApplyEditMesh(Source& src, MaterialTable& mats,
                   const std::vector<EditMeshOperation>& operations,
                   std::string* err, int* errorLine) {
    std::vector<char> selection(src.vertex.size(), 1);
    for (const auto& op : operations) {
        auto fail = [&](const std::string& message) {
            if (err) *err = message;
            if (errorLine) *errorLine = op.line;
            return false;
        };
        if (op.kind == MeshOperation::SelectAll || op.kind == MeshOperation::SelectNone) {
            std::fill(selection.begin(), selection.end(), op.kind == MeshOperation::SelectAll);
            continue;
        }
        if (op.kind == MeshOperation::InvertSelection) {
            for (char& selected : selection) selected = !selected;
            continue;
        }

        int weightBone = -1;
        if (op.kind == MeshOperation::SelectWeights || op.kind == MeshOperation::SetWeight ||
            op.kind == MeshOperation::ShiftWeights) {
            if (src.numbones < 0 || src.localBone.size() != static_cast<size_t>(src.numbones))
                return fail("invalid source bone table");
            weightBone = FindWeightBone(src, op.name);
            if (weightBone < 0) return fail("unknown source bone \"" + op.name + "\"");
            if (op.kind != MeshOperation::ShiftWeights &&
                (!std::isfinite(op.value) || op.value < 0.0f || op.value > 1.0f))
                return fail("weight or threshold must be between 0 and 1");
        }

        if (op.kind == MeshOperation::SelectStream || op.kind == MeshOperation::SelectMaterial ||
            op.kind == MeshOperation::SelectMesh || op.kind == MeshOperation::SelectMeshByStream ||
            op.kind == MeshOperation::SelectWeights) {
            std::vector<char> matches(selection.size(), 0);
            if (op.kind == MeshOperation::SelectWeights) {
                std::vector<char> bones(src.numbones, 0);
                bones[weightBone] = 1;
                if (op.selectChildren)
                    for (int i = weightBone + 1; i < src.numbones; ++i) {
                        const int parent = src.localBone[i].parent;
                        if (parent >= 0 && parent < i) bones[i] = bones[parent];
                    }

                for (size_t i = 0; i < matches.size(); ++i) {
                    const SrcBoneWeight& weights = src.vertex[i].boneweight;
                    if (!ValidBoneWeights(src, weights)) return fail("invalid vertex bone weights");
                    for (int j = 0; j < weights.numbones; ++j)
                        if (bones[weights.bone[j]] && weights.weight[j] > op.value)
                            matches[i] = 1;
                }
            } else if (op.kind == MeshOperation::SelectStream) {
                const Source::EditStream* stream = nullptr;
                for (const auto& candidate : src.editStreams)
                    if (_stricmp(candidate.name.c_str(), op.name.c_str()) == 0) stream = &candidate;
                if (!stream) return fail("no kept mesh has float stream \"" + op.name + "\"");
                for (size_t i = 0; i < matches.size(); ++i)
                    matches[i] = std::isfinite(stream->values[i]) && stream->values[i] >= op.value;
            } else if (op.kind == MeshOperation::SelectMaterial) {
                std::vector<char> matchedMaterials(mats.materialToTexture.size());
                for (size_t material = 0; material < matchedMaterials.size(); ++material)
                    matchedMaterials[material] = MaterialMatches(
                        mats.textures[mats.materialToTexture[material]].name, op.materials);
                for (size_t i = 0; i < matches.size(); ++i) {
                    const int material = src.vertex[i].material;
                    matches[i] = material >= 0 && static_cast<size_t>(material) < matchedMaterials.size() &&
                                 matchedMaterials[material];
                }
            } else {
                std::vector<char> matchedMeshes(src.editMeshes.size());
                for (size_t i = 0; i < src.editMeshes.size(); ++i) {
                    const auto& mesh = src.editMeshes[i];
                    matchedMeshes[i] = op.kind == MeshOperation::SelectMesh
                        ? _stricmp(mesh.name.c_str(), op.name.c_str()) == 0 ||
                          _stricmp(mesh.dagName.c_str(), op.name.c_str()) == 0
                        : std::any_of(mesh.streams.begin(), mesh.streams.end(),
                            [&](const std::string& name) { return _stricmp(name.c_str(), op.name.c_str()) == 0; });
                }
                for (size_t i = 0; i < src.vertexEditMesh.size(); ++i)
                    matches[i] = matchedMeshes[src.vertexEditMesh[i]];
                for (const auto& [vertex, mesh] : src.extraEditMeshes)
                    matches[vertex] = matches[vertex] || matchedMeshes[mesh];
            }

            for (size_t i = 0; i < selection.size(); ++i) {
                if (op.combine == SelectionCombine::Add) selection[i] = selection[i] || matches[i];
                else if (op.combine == SelectionCombine::Subtract) selection[i] = selection[i] && !matches[i];
                else selection[i] = selection[i] && matches[i];
            }
            continue;
        }

        if (op.kind == MeshOperation::SetWeight || op.kind == MeshOperation::ShiftWeights) {
            const bool shift = op.kind == MeshOperation::ShiftWeights;
            int residual = -1;
            Vector3 origin, axis;
            double lengthSquared = 0.0;
            if (shift) {
                if (!op.residualBone.empty()) {
                    residual = FindWeightBone(src, op.residualBone);
                    if (residual < 0) return fail("unknown residual bone \"" + op.residualBone + "\"");
                }
                if (src.boneToPose.size() != static_cast<size_t>(src.numbones))
                    return fail("missing source bind transforms");

                matrix3x4 rotation;
                pm::AngleMatrixDeg(op.angles, rotation);
                origin = pm::VectorTransform(Vector3{}, src.boneToPose[weightBone]);
                axis = pm::VectorRotate(pm::VectorRotate(op.offset, rotation),
                                        src.boneToPose[weightBone]);
                if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
                    !std::isfinite(axis.x) || !std::isfinite(axis.y) || !std::isfinite(axis.z))
                    return fail("shift produces a non-finite ramp");
                lengthSquared = static_cast<double>(axis.x) * axis.x +
                                static_cast<double>(axis.y) * axis.y +
                                static_cast<double>(axis.z) * axis.z;
                if (lengthSquared == 0.0 || residual == weightBone) continue;
            }

            size_t noRecipient = 0, full = 0;
            for (size_t i = 0; i < src.vertex.size(); ++i) {
                if (!shift && !selection[i]) continue;
                SrcVertex& vertex = src.vertex[i];
                SrcBoneWeight& weights = vertex.boneweight;
                if (!ValidBoneWeights(src, weights)) return fail("invalid vertex bone weights");

                WeightEditResult result;
                if (shift) {
                    const int slot = FindWeightSlot(weights, weightBone);
                    if (slot < 0 || weights.weight[slot] <= 0.0f) continue;
                    const Vector3& position = vertex.position;
                    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                        !std::isfinite(position.z))
                        return fail("shift requires finite vertex positions");
                    const double projection = (static_cast<double>(position.x) - origin.x) * axis.x +
                                              (static_cast<double>(position.y) - origin.y) * axis.y +
                                              (static_cast<double>(position.z) - origin.z) * axis.z;
                    const float blend = static_cast<float>(
                        std::clamp(1.0 - projection / lengthSquared, 0.0, 1.0));
                    const float moved = weights.weight[slot] * blend;
                    if (moved <= 0.0f) continue;
                    result = TransferBoneWeight(weights, slot, residual, moved);
                } else {
                    result = SetBoneWeight(weights, weightBone, op.value);
                }
                if (result == WeightEditResult::NoRecipient) ++noRecipient;
                if (result == WeightEditResult::Full) ++full;
            }
            if (noRecipient || full) {
                pulse::diag::Location at = pulse::diag::Current();
                at.line = op.line;
                pulse::diag::WarnAtf(at, "$meshedit: %s \"%s\": left vertices unchanged "
                                     "(%zu without another positive influence, %zu at the influence limit)",
                                     shift ? "$shiftweights" : "$setweight",
                                     op.name.c_str(), noRecipient, full);
            }
            continue;
        }
        if (std::find(selection.begin(), selection.end(), 1) == selection.end()) continue;

        if (op.kind == MeshOperation::Weld) {
            WeldVertices(src, selection, op.value, op.sharpEdges);
            continue;
        }
        if (op.kind == MeshOperation::Inflate) {
            if (op.value == 0.0f) continue;
            std::vector<Vector3> push;
            std::vector<float> distance;
            InflationVectors(src, op.value, op.thicknessClamp, selection, push, distance);
            for (size_t i = 0; i < push.size(); ++i) {
                if (!op.hull) {
                    push[i] = src.vertex[i].normal;
                    pm::VectorNormalize(push[i]);
                }
                distance[i] *= op.value;
                const auto& p = src.vertex[i].position;
                if (selection[i] && (!std::isfinite(p.x + push[i].x * distance[i]) ||
                                     !std::isfinite(p.y + push[i].y * distance[i]) ||
                                     !std::isfinite(p.z + push[i].z * distance[i])))
                    return fail("inflate produces a non-finite position");
            }
            ApplyInflation(src, selection, push, distance, op.hull);
            continue;
        }

        auto faces = AbsoluteFaces(src);
        auto fullySelected = [&](const MFace& face) {
            return selection[face.a] && selection[face.b] && selection[face.c];
        };
        if (op.kind == MeshOperation::DeleteFaces || op.kind == MeshOperation::DeleteVertices) {
            faces.erase(std::remove_if(faces.begin(), faces.end(), [&](const MFace& face) {
                return op.kind == MeshOperation::DeleteFaces ? fullySelected(face)
                    : selection[face.a] || selection[face.b] || selection[face.c];
            }), faces.end());
            PruneEditVertices(src, faces, selection);
            RebuildEditMeshes(src, faces, selection);
            CalcModelTangentSpaces(src);
            continue;
        }

        std::vector<char> pickedFaces(faces.size()), protectedVertices(src.vertex.size(), 0);
        size_t pickedCount = 0;
        for (size_t i = 0; i < faces.size(); ++i) {
            const auto& face = faces[i];
            pickedFaces[i] = fullySelected(face);
            if (pickedFaces[i]) ++pickedCount;
            else protectedVertices[face.a] = protectedVertices[face.b] = protectedVertices[face.c] = 1;
        }
        if (pickedCount == 0) {
            if (op.kind == MeshOperation::Duplicate && op.replaceSelection)
                std::fill(selection.begin(), selection.end(), 0);
            continue;
        }
        if (op.kind == MeshOperation::Duplicate &&
            pickedCount > static_cast<size_t>(lim::kMaxTriangles) - faces.size())
            return fail("duplicatemesh exceeds the triangle limit");

        int material = -1;
        if (op.kind == MeshOperation::ReplaceMaterial) {
            const int texture = mats.LookupTexture(op.name.c_str());
            if (mats.textures[texture].material < 0 && mats.materialToTexture.size() >= lim::kMaxSkins)
                return fail("replacematerial exceeds the material limit");
            material = mats.UseTextureAsMaterial(texture);
        }

        std::vector<int> origins(src.vertex.size()), copied(src.vertex.size(), -1);
        std::vector<char> affected(src.vertex.size(), 0);
        for (size_t i = 0; i < origins.size(); ++i) origins[i] = static_cast<int>(i);
        for (size_t i = 0; i < pickedFaces.size(); ++i) {
            if (!pickedFaces[i]) continue;
            MFace face = faces[i];
            for (uint32_t* index : {&face.a, &face.b, &face.c}) {
                const int original = static_cast<int>(*index);
                if (op.kind == MeshOperation::Duplicate || protectedVertices[original]) {
                    if (copied[original] < 0) {
                        if (origins.size() >= static_cast<size_t>(lim::kMaxVerts))
                            return fail("operation exceeds the vertex limit");
                        copied[original] = static_cast<int>(origins.size());
                        origins.push_back(original);
                        affected.push_back(0);
                    }
                    *index = copied[original];
                }
                affected[*index] = 1;
            }
            if (op.kind == MeshOperation::Duplicate) faces.push_back(face);
            else {
                if (op.kind == MeshOperation::ReplaceMaterial) face.material = material;
                if (op.kind == MeshOperation::FlipNormals) std::swap(face.b, face.c);
                faces[i] = face;
            }
        }
        ReindexEditVertices(src, origins, selection);

        if (op.kind == MeshOperation::Duplicate && op.replaceSelection)
            selection = affected;
        if (op.kind == MeshOperation::ReplaceMaterial)
            for (size_t i = 0; i < affected.size(); ++i)
                if (affected[i]) src.vertex[i].material = material;
        if (op.kind == MeshOperation::FlipNormals) {
            for (size_t i = 0; i < affected.size(); ++i) {
                if (!affected[i]) continue;
                auto& v = src.vertex[i];
                v.normal = {-v.normal.x, -v.normal.y, -v.normal.z};
            }
            for (auto& morph : src.morphs)
                for (auto& va : morph.vanims)
                    if (affected[va.vertex]) va.normal = {-va.normal.x, -va.normal.y, -va.normal.z};
        }
        RebuildEditMeshes(src, faces, selection);
        CalcModelTangentSpaces(src);
    }
    return true;
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
