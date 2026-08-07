// writevtx.cpp - the .vtx (OptimizedModel) builder, dx90 legacy (format 0)
// path: MergeLikeBoneIndicesWithinVerts, ProcessMesh's four strip-group
// passes, HW-skinned flood fill over the LRU hardware matrix state, meshopt
// vcache+overdraw ordering, PostProcessStripGroup, the offset-computed file
// layout, and MapGlobalBonesToHardwareBoneIDs.

#include "writer.h"

#include <cstring>
#include <map>
#include <utility>

#include "format/mdl.h"
#include "format/vvd.h"
#include "format/vtx.h"
#include "meshoptimizer.h"
#include "pulselimits.h"

namespace pulse::writer {

namespace fmt = pulse::format;
namespace cm = pulse::compile;
namespace lim = pulse::limits;

namespace {

constexpr int kMaxBonesPerVert = 3;

// VtxMesh flags
constexpr int kMeshIsTeeth = 0x01;
constexpr int kMeshIsEyes = 0x02;

// ---------------------------------------------------------------------------
// CHardwareMatrixState - LRU bone palette
// ---------------------------------------------------------------------------
struct HardwareMatrixState {
    struct MatrixState {
        bool allocated = false;
        int globalMatrixID = 0;
        int lastUsageID = 0;
    };
    std::vector<MatrixState> state;
    int lruCounter = 0;
    int allocatedCount = 0;

    void Init(int numMatrices) {
        state.assign(numMatrices, MatrixState{});
        lruCounter = 0;
        allocatedCount = 0;
    }
    bool IsMatrixAllocated(int globalID) const {
        for (const MatrixState& s : state)
            if (s.globalMatrixID == globalID && s.allocated)
                return true;
        return false;
    }
    bool AllocateMatrix(int globalID) {
        if (IsMatrixAllocated(globalID))
            return true;
        for (MatrixState& s : state) {
            if (!s.allocated) {
                s.globalMatrixID = globalID;
                s.allocated = true;
                s.lastUsageID = lruCounter++;
                ++allocatedCount;
                return true;
            }
        }
        return false;
    }
    // flush the whole palette (reference CHardwareMatrixState::DeallocateAll);
    // the LRU counter keeps running, matching the reference's allocation ids
    void DeallocateAll() {
        for (MatrixState& s : state)
            s.allocated = false;
        allocatedCount = 0;
    }
    void DeallocateLRU(int n) {
        for (int i = 0; i < n; i++) {
            int oldest = INT32_MAX, oldestID = 0;
            for (size_t j = 0; j < state.size(); j++) {
                if (!state[j].allocated)
                    continue;
                if (state[j].lastUsageID < oldest) {
                    oldest = state[j].lastUsageID;
                    oldestID = static_cast<int>(j);
                }
            }
            state[oldestID].allocated = false;
            --allocatedCount;
        }
    }
    int FreeMatrixCount() const { return static_cast<int>(state.size()) - allocatedCount; }
    int AllocatedMatrixCount() const { return allocatedCount; }
    int GetNthBoneGlobalID(int n) const {
        int m = 0;
        for (const MatrixState& s : state) {
            if (s.allocated) {
                if (n == m)
                    return s.globalMatrixID;
                m++;
            }
        }
        return 0;
    }
};

// ---------------------------------------------------------------------------
// working structures (reference Strip_t/StripGroup_t/Mesh_t/...)
// ---------------------------------------------------------------------------
struct Face {
    int vertID[4] = {-1, -1, -1, -1};
    int neighborID[4] = {-1, -1, -1, -1};
    int boneID[kMaxBonesPerVert * 4];
    int numBones = 0;
    bool touched = false;
};

struct BoneStateChange {
    int hardwareID = -1;
    int newBoneID = -1;
};

struct Strip {
    int numIndices = 0;
    std::vector<uint16_t> indices;
    std::vector<fmt::vtx::Vertex_t> verts;
    uint8_t flags = 0;
    int numBoneStateChanges = 0;
    BoneStateChange boneStateChanges[fmt::vtx::kMaxBonesPerStrip];
    // post-process results
    int stripGroupIndexOffset = 0;
    int stripGroupVertexOffset = 0;
    int numStripGroupIndices = 0;
    int numStripGroupVerts = 0;
    int numBones = 0;
};

struct StripGroup {
    uint8_t flags = 0;
    std::vector<Strip> strips;
    std::vector<fmt::vtx::Vertex_t> verts;
    std::vector<uint16_t> indices;
};

struct VtxMesh {
    uint8_t flags = 0;
    std::vector<StripGroup> stripGroups;
};

struct VtxModelLOD {
    float switchPoint = 0;
    std::vector<VtxMesh> meshes;
};

struct VtxModel {
    std::vector<VtxModelLOD> modelLODs;
};

struct Builder {
    cm::CompiledModel* m = nullptr;
    std::vector<uint8_t>* mdlBuf = nullptr;
    std::vector<uint8_t>* vvdBuf = nullptr;
    HardwareMatrixState hwState;
    int maxBonesPerVert = 3;
    int maxBonesPerFace = 9;
    int maxBonesPerStrip = 53;
    int numBones = 0;
    std::vector<VtxModel> models;

    fmt::studiohdr_t* hdr() { return reinterpret_cast<fmt::studiohdr_t*>(mdlBuf->data()); }
    fmt::mstudiobodyparts_t* bodypart(int i) {
        return reinterpret_cast<fmt::mstudiobodyparts_t*>(mdlBuf->data() + hdr()->bodypartindex) + i;
    }
    fmt::mstudiomodel_t* model(fmt::mstudiobodyparts_t* bp, int i) {
        return reinterpret_cast<fmt::mstudiomodel_t*>(
                   reinterpret_cast<uint8_t*>(bp) + bp->modelindex) + i;
    }
    fmt::mstudiomesh_t* mesh(fmt::mstudiomodel_t* mdl, int i) {
        return reinterpret_cast<fmt::mstudiomesh_t*>(
                   reinterpret_cast<uint8_t*>(mdl) + mdl->meshindex) + i;
    }
    // vvd accessors (cached copy the builder may mutate, like the reference)
    fmt::mstudiovertex_t* vvdVertex(fmt::mstudiomodel_t* mdl, int i) {
        fmt::vertexFileHeader_t* vh =
            reinterpret_cast<fmt::vertexFileHeader_t*>(vvdBuf->data());
        return reinterpret_cast<fmt::mstudiovertex_t*>(
                   vvdBuf->data() + vh->vertexDataStart + mdl->vertexindex) + i;
    }
};

// MergeLikeBoneIndicesWithinVert
void MergeLikeBoneIndicesWithinVert(fmt::mstudioboneweight_t* bw) {
    if (bw->numbones == 1)
        return;
    int realNumBones = bw->numbones;
    for (int i = 0; i < bw->numbones; i++) {
        for (int j = i + 1; j < bw->numbones; j++) {
            if ((bw->bone[i] == bw->bone[j]) && (bw->weight[i] != 0.0f)) {
                bw->weight[i] += bw->weight[j];
                bw->weight[j] = 0.0f;
                realNumBones--;
            }
        }
    }
    for (int j = bw->numbones; j > 1; j--) {
        for (int k = 0; k < j - 1; k++) {
            if ((bw->weight[k] == 0.0f) && (bw->weight[k + 1] != 0.0f)) {
                uint8_t tb = bw->bone[k];
                float tw = bw->weight[k];
                bw->bone[k] = bw->bone[k + 1];
                bw->weight[k] = bw->weight[k + 1];
                bw->bone[k + 1] = tb;
                bw->weight[k + 1] = tw;
            }
        }
    }
    bw->numbones = static_cast<uint8_t>(realNumBones);
}

// TryToReduceBoneInfluence
void TryToReduceBoneInfluence(fmt::vtx::Vertex_t& v, fmt::mstudioboneweight_t& bw, int maxBones) {
    int i;
    while (v.numBones > maxBones) {
        float minWeight = 2.0f;
        int minIndex = -1;
        for (i = 0; i < kMaxBonesPerVert; ++i) {
            if (v.boneID[i] != 255) {
                float weight = bw.weight[v.boneWeightIndex[i]];
                if (weight < minWeight) {
                    minWeight = weight;
                    minIndex = i;
                }
            }
        }
        if (minWeight >= 0.0001f)
            break;
        for (i = minIndex; i < kMaxBonesPerVert - 1; ++i) {
            v.boneID[i] = v.boneID[i + 1];
            v.boneWeightIndex[i] = v.boneWeightIndex[i + 1];
        }
        v.boneID[kMaxBonesPerVert - 1] = 255;
        v.boneWeightIndex[kMaxBonesPerVert - 1] = 0;
        --v.numBones;
    }

    float t = 0.0f;
    for (i = 0; i < kMaxBonesPerVert; ++i)
        if (v.boneID[i] != 255)
            t += bw.weight[v.boneWeightIndex[i]];
    if (t > 0.0f && fabsf(t - 1.0f) > 0.001f) {
        t = 1.0f / t;
        for (i = 0; i < kMaxBonesPerVert; ++i)
            if (v.boneID[i] != 255)
                bw.weight[v.boneWeightIndex[i]] *= t;
    }
}

// GenerateStripGroupVerticesFromFace
// IsVertexFlexed: is this mesh-relative vertex touched by
// any of the mesh's flexes? Called at VTX build time, BEFORE FixupMDLFile
// remaps the vanim indices, so pAnim->index is still mesh-relative here.
bool IsVertexFlexed(fmt::mstudiomesh_t* pStudioMesh, int vertID) {
    uint8_t* pMeshBase = reinterpret_cast<uint8_t*>(pStudioMesh);
    for (int i = 0; i < pStudioMesh->numflexes; i++) {
        auto* pflex =
            reinterpret_cast<fmt::mstudioflex_t*>(pMeshBase + pStudioMesh->flexindex) + i;
        uint8_t* pvanim = reinterpret_cast<uint8_t*>(pflex) + pflex->vertindex;
        size_t nVAnimSizeBytes = (pflex->vertanimtype == fmt::STUDIO_VERT_ANIM_WRINKLE)
                                     ? sizeof(fmt::mstudiovertanim_wrinkle_t)
                                     : sizeof(fmt::mstudiovertanim_t);
        for (int j = 0; j < pflex->numverts; j++, pvanim += nVAnimSizeBytes) {
            if (reinterpret_cast<fmt::mstudiovertanim_t*>(pvanim)->index == vertID)
                return true;
        }
    }
    return false;
}

// returns true when ANY vertex of the face is flexed (reference returns the
// same from GenerateStripGroupVerticesFromFace)
bool GenerateStripGroupVerticesFromFace(Builder& b, const source::SrcFace& face,
                                        fmt::mstudiomodel_t* pStudioModel,
                                        fmt::mstudiomesh_t* pStudioMesh, int maxPreferredBones,
                                        fmt::vtx::Vertex_t* out) {
    uint32_t vertIDs[3] = {face.a, face.b, face.c};
    bool bFaceIsFlexed = false;
    for (int fi = 0; fi < 3; ++fi) {
        int vertex = static_cast<int>(vertIDs[fi]);
        bFaceIsFlexed = bFaceIsFlexed || IsVertexFlexed(pStudioMesh, vertex);
        // face vertex ids are MESH-relative; vvdVertex indexes the model's
        // vertex block, so the mesh's vertexoffset has to be added (the
        // reference goes through pStudioMesh->GetVertexData(), which is
        // already mesh-relative). Only meshes after the first have a nonzero
        // offset, which is why single-mesh testcases never caught this.
        fmt::mstudioboneweight_t* bw =
            &b.vvdVertex(pStudioModel, pStudioMesh->vertexoffset + vertex)->m_BoneWeights;
        int bonesAffectingVertex = bw->numbones;

        out[fi].origMeshVertID = static_cast<uint16_t>(vertex);
        out[fi].numBones = static_cast<uint8_t>(bonesAffectingVertex);

        int boneID;
        for (boneID = 0; boneID < bonesAffectingVertex; boneID++) {
            out[fi].boneID[boneID] = bw->bone[boneID];
            out[fi].boneWeightIndex[boneID] = static_cast<uint8_t>(boneID);
        }
        for (; boneID < kMaxBonesPerVert; boneID++) {
            out[fi].boneID[boneID] = 255;
            out[fi].boneWeightIndex[boneID] = static_cast<uint8_t>(boneID);
        }

        if (maxPreferredBones > 0 && bonesAffectingVertex > maxPreferredBones)
            TryToReduceBoneInfluence(out[fi], *bw, maxPreferredBones);
    }
    return bFaceIsFlexed;
}

int FindOrCreateVertex(std::vector<fmt::vtx::Vertex_t>& list,
                       std::map<int, int>& lookup, const fmt::vtx::Vertex_t& vert) {
    auto it = lookup.find(vert.origMeshVertID);
    if (it != lookup.end())
        return it->second;
    int result = static_cast<int>(list.size());
    list.push_back(vert);
    lookup[vert.origMeshVertID] = result;
    return result;
}

// BuildFaceBoneData
void BuildFaceBoneData(Builder& b, std::vector<fmt::vtx::Vertex_t>& list, Face& face) {
    std::vector<bool> seen(b.numBones, false);
    face.numBones = 0;
    for (int j = 0; j < 3; j++) {
        fmt::vtx::Vertex_t& vert = list[face.vertID[j]];
        for (int k = 0; k < vert.numBones; ++k) {
            int bone = vert.boneID[k];
            if (!seen[bone]) {
                seen[bone] = true;
                face.boneID[face.numBones++] = bone;
            }
        }
    }
}

// BuildNeighborInfo: shared-edge adjacency
void BuildNeighborInfo(std::vector<Face>& faceList, int numVerts) {
    struct EdgeInfo {
        int connectedVertId;
        int edgeIndex;
        int faceId;
    };
    std::vector<std::vector<EdgeInfo>> vertexToEdges(numVerts);

    auto findMatchingEdge = [&](int faceId, int edgeIndex, int v0, int v1) {
        Face& face = faceList[faceId];
        int order = (v0 < v1) ? 0 : 1;
        int vertIndex = order == 0 ? v0 : v1;
        int connectedVertId = order == 0 ? v1 : v0;

        auto& edges = vertexToEdges[vertIndex];
        for (size_t e = 0; e < edges.size(); e++) {
            if (edges[e].connectedVertId != connectedVertId)
                continue;
            if (edges[e].faceId == faceId)
                continue;
            face.neighborID[edgeIndex] = edges[e].faceId;
            faceList[edges[e].faceId].neighborID[edges[e].edgeIndex] = faceId;
            edges.erase(edges.begin() + e);
            return;
        }
        // reference links new edges at the head of the per-vertex list
        edges.insert(edges.begin(), EdgeInfo{connectedVertId, edgeIndex, faceId});
    };

    for (int i = 0; i < static_cast<int>(faceList.size()); i++) {
        Face& face = faceList[i];
        findMatchingEdge(i, 0, face.vertID[0], face.vertID[1]);
        findMatchingEdge(i, 1, face.vertID[1], face.vertID[2]);
        findMatchingEdge(i, 2, face.vertID[2], face.vertID[0]);
    }
}

int ComputeNewBonesNeeded(Builder& b, const Face& face) {
    int numNewBones = 0;
    for (int i = 0; i < face.numBones; ++i)
        if (!b.hwState.IsMatrixAllocated(face.boneID[i]))
            ++numNewBones;
    return numNewBones;
}

Face* GetNextUntouched(std::vector<Face>& faces) {
    for (Face& f : faces)
        if (!f.touched)
            return &f;
    return nullptr;
}

Face* GetNextUntouchedWithoutBoneStateChange(Builder& b, std::vector<Face>& faces) {
    Face* bestFace = nullptr;
    int bestNumNewBones = kMaxBonesPerVert * 3 + 1;
    for (Face& f : faces) {
        if (!f.touched) {
            int numNewBones = ComputeNewBonesNeeded(b, f);
            if (numNewBones <= b.hwState.FreeMatrixCount() && numNewBones < bestNumNewBones) {
                bestNumNewBones = numNewBones;
                bestFace = &f;
                if (bestNumNewBones == 0)
                    break;
            }
        }
    }
    return bestFace;
}

Face* GetNextUntouchedWithLeastBoneStateChanges(Builder& b, std::vector<Face>& faces) {
    Face* bestFace = nullptr;
    int bestNumNewBones = kMaxBonesPerVert * 3 + 1;
    for (Face& f : faces) {
        if (!f.touched) {
            int numNewBones = ComputeNewBonesNeeded(b, f);
            if (numNewBones < bestNumNewBones) {
                bestNumNewBones = numNewBones;
                bestFace = &f;
            }
        }
    }
    if (!bestFace)
        return nullptr;
    // PARITY: the reference compiles the USE_FLUSH branch -
    // it flushes the WHOLE palette here, it does NOT evict just enough via
    // DeallocateLRU (that is the dead #else branch). The difference is not
    // just bookkeeping: an empty palette means the next
    // GetNextUntouchedWithoutBoneStateChange finds nothing, so face traversal
    // (and therefore strip/vertex order) changes. Found by byte-diffing
    // testcase10 - only the FIRST mesh matched with the LRU variant.
    b.hwState.DeallocateAll();
    return bestFace;
}

Face* GetNextFace(Builder& b, std::vector<Face>& faceList, bool allowNewStrip) {
    Face* face = GetNextUntouchedWithoutBoneStateChange(b, faceList);
    if (!face && allowNewStrip)
        face = GetNextUntouchedWithLeastBoneStateChanges(b, faceList);
    return face;
}

bool AllocateHardwareBonesForFace(Builder& b, Face* face) {
    for (int i = 0; i < face->numBones; ++i) {
        int bone = face->boneID[i];
        if (!b.hwState.IsMatrixAllocated(bone))
            if (!b.hwState.AllocateMatrix(bone))
                return false;
    }
    return true;
}

// BuildStripsRecursive (iterative flood fill)
void BuildStripsFloodFill(Builder& b, std::vector<uint16_t>& indices, std::vector<Face>& faceList,
                          Face* seed) {
    std::vector<Face*> stack;
    stack.reserve(faceList.size());
    stack.push_back(seed);
    while (!stack.empty()) {
        Face* f = stack.back();
        stack.pop_back();
        if (f->touched)
            continue;
        if (ComputeNewBonesNeeded(b, *f))
            continue;
        f->touched = true;
        indices.push_back(static_cast<uint16_t>(f->vertID[0]));
        indices.push_back(static_cast<uint16_t>(f->vertID[1]));
        indices.push_back(static_cast<uint16_t>(f->vertID[2]));
        if (f->neighborID[0] != -1) stack.push_back(&faceList[f->neighborID[0]]);
        if (f->neighborID[1] != -1) stack.push_back(&faceList[f->neighborID[1]]);
        if (f->neighborID[2] != -1) stack.push_back(&faceList[f->neighborID[2]]);
    }
}

// Stripify - trilist + meshopt vcache
void Stripify(const std::vector<uint16_t>& sourceIndices, Strip& strip) {
    if (sourceIndices.empty()) {
        strip.numIndices = 0;
        return;
    }
    size_t indexCount = sourceIndices.size();
    std::vector<unsigned int> indices32(indexCount);
    unsigned int vertex_count = 0;
    for (size_t i = 0; i < indexCount; i++) {
        indices32[i] = sourceIndices[i];
        if (indices32[i] >= vertex_count)
            vertex_count = indices32[i] + 1;
    }

    meshopt_optimizeVertexCache(indices32.data(), indices32.data(), indexCount, vertex_count);

    strip.numIndices = static_cast<int>(indexCount);
    strip.indices.resize(indexCount);
    for (size_t i = 0; i < indexCount; i++)
        strip.indices[i] = static_cast<uint16_t>(indices32[i]);
}

// BuildHWSkinnedStrips
void BuildHWSkinnedStrips(Builder& b, std::vector<Face>& faceList,
                          std::vector<fmt::vtx::Vertex_t>& vertices, StripGroup* pStripGroup,
                          int maxBonesPerStrip, fmt::mstudiomodel_t* pStudioModel,
                          fmt::mstudiomesh_t* pStudioMesh) {
    b.hwState.Init(maxBonesPerStrip);

    std::vector<uint16_t> facesToStrip;
    facesToStrip.reserve(faceList.size() * 3);

    Face* pSeedFace = GetNextUntouched(faceList);
    while (pSeedFace) {
        AllocateHardwareBonesForFace(b, pSeedFace);
        BuildStripsFloodFill(b, facesToStrip, faceList, pSeedFace);

        pSeedFace = GetNextFace(b, faceList, false);
        if (pSeedFace)
            continue;

        pStripGroup->strips.emplace_back();
        Strip& newStrip = pStripGroup->strips.back();
        newStrip.flags = fmt::vtx::STRIP_IS_TRILIST;

        Stripify(facesToStrip, newStrip);

        // meshopt_optimizeOverdraw at 1.05 (positions from the vvd copy)
        if (newStrip.numIndices > 0) {
            int nVerts = static_cast<int>(vertices.size());
            std::vector<float> positions(nVerts * 3);
            for (int vi = 0; vi < nVerts; vi++) {
                // origMeshVertID is MESH-relative; the reference reads it through
                // pStudioMesh->GetVertexData()->Position(), which adds the mesh's
                // vertexoffset. Without it every mesh after the first feeds
                // meshopt_optimizeOverdraw the wrong positions -> different
                // triangle order -> different strip-group vertex order.
                fmt::mstudiovertex_t* pv = b.vvdVertex(
                    pStudioModel, pStudioMesh->vertexoffset + vertices[vi].origMeshVertID);
                positions[vi * 3 + 0] = pv->m_vecPosition.x;
                positions[vi * 3 + 1] = pv->m_vecPosition.y;
                positions[vi * 3 + 2] = pv->m_vecPosition.z;
            }
            std::vector<unsigned int> indices32(newStrip.numIndices);
            for (int ii = 0; ii < newStrip.numIndices; ii++)
                indices32[ii] = newStrip.indices[ii];
            meshopt_optimizeOverdraw(indices32.data(), indices32.data(),
                                     static_cast<size_t>(newStrip.numIndices), positions.data(),
                                     static_cast<size_t>(nVerts), sizeof(float) * 3, 1.05f);
            for (int ii = 0; ii < newStrip.numIndices; ii++)
                newStrip.indices[ii] = static_cast<uint16_t>(indices32[ii]);
        }

        newStrip.verts = vertices;

        newStrip.numBoneStateChanges = b.hwState.AllocatedMatrixCount();
        for (int i = 0; i < b.hwState.AllocatedMatrixCount(); i++) {
            newStrip.boneStateChanges[i].hardwareID = i;
            newStrip.boneStateChanges[i].newBoneID = b.hwState.GetNthBoneGlobalID(i);
        }

        facesToStrip.clear();
        pSeedFace = GetNextFace(b, faceList, true);
    }
}

// ProcessStripGroup
void ProcessStripGroup(Builder& b, StripGroup* pStripGroup, bool bIsHWSkinned, bool bIsFlexed,
                       fmt::mstudiomodel_t* pStudioModel, fmt::mstudiomesh_t* pStudioMesh,
                       const std::vector<source::SrcFace>& srcFaces,
                       std::vector<bool>& facesProcessed, int maxBonesPerVert,
                       int maxBonesPerFace, int maxBonesPerStrip, bool bForceNoFlex) {
    // ComputeStripGroupFlags: a flexed group sets BOTH the
    // software-flex and hardware-delta-flex bits
    pStripGroup->flags = 0;
    if (bIsFlexed) {
        pStripGroup->flags |= fmt::vtx::STRIPGROUP_IS_FLEXED;
        pStripGroup->flags |= fmt::vtx::STRIPGROUP_IS_DELTA_FLEXED;
    }
    if (bIsHWSkinned)
        pStripGroup->flags |= fmt::vtx::STRIPGROUP_IS_HWSKINNED;

    std::vector<Face> stripGroupSourceFaces;
    std::vector<fmt::vtx::Vertex_t> stripGroupVertices;
    std::map<int, int> vertexLookup;

    for (size_t n = 0; n < srcFaces.size(); ++n) {
        if (facesProcessed[n])
            continue;

        const source::SrcFace& face = srcFaces[n];
        int preferredBones = bIsHWSkinned ? maxBonesPerVert : 0;

        fmt::vtx::Vertex_t stripGroupVert[3];
        bool bFaceIsFlexed = GenerateStripGroupVerticesFromFace(b, face, pStudioModel, pStudioMesh,
                                                               preferredBones, stripGroupVert);

        // nomorphs/nofacial: this LOD ignores the morphs entirely, so every
        // face lands in the non-flexed group (reference bForceNoFlex)
        if (bForceNoFlex)
            bFaceIsFlexed = false;

        // a face belongs to exactly one of the flexed / non-flexed groups
        if (bFaceIsFlexed != bIsFlexed)
            continue;

        if (bIsHWSkinned) {
            // count max bones per vert + unique bones in the face
            int numVertexBones = 0;
            for (int c = 0; c < 3; c++)
                if (stripGroupVert[c].numBones > numVertexBones)
                    numVertexBones = stripGroupVert[c].numBones;
            int uniqueBones = 0;
            int seenList[kMaxBonesPerVert * 3];
            for (int c = 0; c < 3; c++) {
                for (int bi = 0; bi < stripGroupVert[c].numBones; bi++) {
                    int bone = stripGroupVert[c].boneID[bi];
                    bool found = false;
                    for (int s = 0; s < uniqueBones; s++)
                        if (seenList[s] == bone) { found = true; break; }
                    if (!found)
                        seenList[uniqueBones++] = bone;
                }
            }
            bool bFaceIsHWSkinned =
                (uniqueBones <= maxBonesPerFace) && (numVertexBones <= maxBonesPerVert);
            if (!bFaceIsHWSkinned)
                continue;
        }

        Face newFace;
        newFace.vertID[0] = FindOrCreateVertex(stripGroupVertices, vertexLookup, stripGroupVert[0]);
        newFace.vertID[1] = FindOrCreateVertex(stripGroupVertices, vertexLookup, stripGroupVert[1]);
        newFace.vertID[2] = FindOrCreateVertex(stripGroupVertices, vertexLookup, stripGroupVert[2]);
        newFace.vertID[3] = -1;

        stripGroupSourceFaces.push_back(newFace);
        BuildFaceBoneData(b, stripGroupVertices, stripGroupSourceFaces.back());

        facesProcessed[n] = true;
    }

    if (stripGroupSourceFaces.empty())
        return;

    BuildNeighborInfo(stripGroupSourceFaces, static_cast<int>(stripGroupVertices.size()));

    if (bIsHWSkinned)
        BuildHWSkinnedStrips(b, stripGroupSourceFaces, stripGroupVertices, pStripGroup,
                             maxBonesPerStrip, pStudioModel, pStudioMesh);
    // (software path: all faces already consumed by the HW pass in phase 1)
}

// how many verts this strip will contribute to its group (each strip gets its
// own copy of every vertex it touches - see PostProcessStripGroup's per-strip
// lookup, which starts empty)
int CountStripVerts(const Strip& strip) {
    std::map<int, int> seen;
    for (int j = 0; j < strip.numIndices; j++)
        seen[strip.verts[strip.indices[j]].origMeshVertID] = 0;
    return static_cast<int>(seen.size());
}

// PostProcessStripGroup, packing `src`'s strips into the
// groups appended to `out`.
//
// Strip group indices are uint16, so a group tops out at kMaxStripGroupVerts.
// The reference never splits here because stock studiomdl rejects any model big
// enough to reach it; we do split meshes (BuildOutputMeshes), and a mesh right
// at the cap still overflows once its strips duplicate shared verts - which is
// what many bones (jigglebones, procedural bones) causes, since a full hardware
// bone palette ends the current strip and starts another. Overflowing wrapped
// the indices, folding the tail verts onto live ones as stretched triangles.
void PostProcessStripGroup(StripGroup* src, std::vector<StripGroup>& out) {
    const size_t firstGroup = out.size();
    for (Strip& strip : src->strips) {
        if (out.size() == firstGroup ||
            static_cast<int>(out.back().verts.size()) + CountStripVerts(strip) >
                lim::kMaxStripGroupVerts) {
            out.emplace_back();
            out.back().flags = src->flags;
        }
        StripGroup* pStripGroup = &out.back();

        int vertOffset = static_cast<int>(pStripGroup->verts.size());
        strip.stripGroupVertexOffset = vertOffset;
        strip.stripGroupIndexOffset = static_cast<int>(pStripGroup->indices.size());

        std::map<int, int> lookup; // origMeshVertID -> stripgroup vert id
        for (size_t k = vertOffset; k < pStripGroup->verts.size(); k++)
            lookup[pStripGroup->verts[k].origMeshVertID] = static_cast<int>(k);

        int maxNumBones = 0;
        for (int j = 0; j < strip.numIndices; j++) {
            int index = strip.indices[j];
            fmt::vtx::Vertex_t* pVert = &strip.verts[index];
            int newIndex;
            auto it = lookup.find(pVert->origMeshVertID);
            if (it != lookup.end()) {
                newIndex = it->second;
            } else {
                newIndex = static_cast<int>(pStripGroup->verts.size());
                pStripGroup->verts.push_back(*pVert);
                lookup[pVert->origMeshVertID] = newIndex;
            }
            pStripGroup->indices.push_back(static_cast<uint16_t>(newIndex));
            if (pVert->numBones > maxNumBones)
                maxNumBones = pVert->numBones;
        }

        strip.numStripGroupIndices =
            static_cast<int>(pStripGroup->indices.size()) - strip.stripGroupIndexOffset;
        strip.numStripGroupVerts =
            static_cast<int>(pStripGroup->verts.size()) - strip.stripGroupVertexOffset;
        strip.numBones = maxNumBones; // non-fixed-function

        pStripGroup->strips.push_back(std::move(strip));
    }
    // an empty group (every strip empty) is not written, same as before
    if (out.size() > firstGroup && out.back().indices.empty())
        out.pop_back();
}

} // namespace

std::vector<uint8_t> BuildVtx(cm::CompiledModel& m, std::vector<uint8_t>& mdlBuf,
                              std::vector<uint8_t>& vvdBuf, bool legacyVtx) {
    Builder b;
    b.m = &m;
    b.mdlBuf = &mdlBuf;
    // work on a COPY of the vvd: the reference mutates its cached vvd (bone
    // weight merge / renormalize) but the on-disk vvd keeps the original bytes
    std::vector<uint8_t> vvdCopy = vvdBuf;
    b.vvdBuf = &vvdCopy;
    b.numBones = static_cast<int>(m.bones.size());

    fmt::studiohdr_t* phdr = b.hdr();

    // MergeLikeBoneIndicesWithinVerts on the cached copy
    for (int bp = 0; bp < phdr->numbodyparts; bp++) {
        fmt::mstudiobodyparts_t* pBodyPart = b.bodypart(bp);
        for (int mo = 0; mo < pBodyPart->nummodels; mo++) {
            fmt::mstudiomodel_t* pModel = b.model(pBodyPart, mo);
            for (int v = 0; v < pModel->numvertices; v++)
                MergeLikeBoneIndicesWithinVert(&b.vvdVertex(pModel, v)->m_BoneWeights);
        }
    }

    const int numLODs = static_cast<int>(m.scriptLods.size());

    // ProcessModel: dx90, non-fixed-function, hw flex
    int modelIdx = 0;
    for (int bp = 0; bp < phdr->numbodyparts; bp++) {
        fmt::mstudiobodyparts_t* pBodyPart = b.bodypart(bp);
        for (int mo = 0; mo < pBodyPart->nummodels; mo++, modelIdx++) {
            fmt::mstudiomodel_t* pStudioModel = b.model(pBodyPart, mo);
            cm::Model& model = m.models[modelIdx];

            b.models.emplace_back();
            VtxModel& newModel = b.models.back();

            for (int lodID = 0; lodID < numLODs; lodID++) {
                const cm::ScriptLod& scriptLod = m.scriptLods[lodID];
                newModel.modelLODs.emplace_back();
                VtxModelLOD& newLOD = newModel.modelLODs.back();
                newLOD.switchPoint = scriptLod.switchValue;

                // a blank choice has no meshes at any LOD
                if (!model.source)
                    continue;

                // null = removemodel dropped this model at this LOD
                const source::Source* pLodSource =
                    lodID < static_cast<int>(model.lodSources.size()) ? model.lodSources[lodID]
                                                                     : nullptr;

                for (int meshID = 0; meshID < pStudioModel->nummeshes; meshID++) {
                    fmt::mstudiomesh_t* pStudioMesh = b.mesh(pStudioModel, meshID);
                    const cm::OutMesh& outMesh = model.outMeshes[meshID];

                    newLOD.meshes.emplace_back();
                    VtxMesh& newMesh = newLOD.meshes.back();
                    // ComputeMeshFlags: a non-zero materialtype
                    // means the eyeball pass claimed this mesh. MESH_IS_TEETH needs
                    // $mouth teeth material matching, which is not implemented.
                    newMesh.flags = pStudioMesh->materialtype != 0 ? kMeshIsEyes : 0;

                    // removemodel / removemesh leave the mesh header in place
                    // with no strip groups: the per-LOD mesh count has to stay
                    // equal to the .mdl's or FixupToSortedLODVertexes rejects
                    // the pair. (The reference `continue`s the whole LOD here,
                    // writing zero meshes - which only holds together for a
                    // blank model, and is why its removemodel produces nothing.)
                    // BuildOutputMeshes already left the list empty for both.
                    if (!pLodSource || lodID >= static_cast<int>(outMesh.lodFaces.size()))
                        continue;
                    const std::vector<source::SrcFace>& meshFaces = outMesh.lodFaces[lodID];
                    if (meshFaces.empty())
                        continue;

                    std::vector<bool> facesProcessed(meshFaces.size(), false);

                    // 4 passes: hw+flexed, hw+nonflexed, sw+flexed, sw+nonflexed.
                    // Empty groups are dropped below, so a model with no flexes
                    // still emits only hw+nonflexed (byte-parity with phase 1/2).
                    // We write .dx90.vtx = hardware flex, so the bone maxima are
                    // NOT clamped to 1 (reference bHWFlex path).
                    for (int isHWSkinned = 1; isHWSkinned >= 0; --isHWSkinned) {
                        for (int isFlexed = 1; isFlexed >= 0; --isFlexed) {
                            StripGroup sg;
                            ProcessStripGroup(b, &sg, isHWSkinned != 0, isFlexed != 0, pStudioModel,
                                              pStudioMesh, meshFaces, facesProcessed,
                                              b.maxBonesPerVert, b.maxBonesPerFace,
                                              b.maxBonesPerStrip, !scriptLod.facialAnimation);
                            PostProcessStripGroup(&sg, newMesh.stripGroups);
                        }
                    }
                }
            }
        }
    }

    // ---- WriteVTXFile: compute offsets, write blocks ----
    int totalBodyParts = phdr->numbodyparts;
    int totalModels = 0, totalModelLODs = 0, totalMeshes = 0;
    int totalStripGroups = 0, totalStrips = 0, totalVerts = 0, totalIndices = 0;
    int totalBoneStateChanges = 0;
    for (VtxModel& mo : b.models) {
        totalModels++;
        for (VtxModelLOD& lod : mo.modelLODs) {
            totalModelLODs++;
            for (VtxMesh& me : lod.meshes) {
                totalMeshes++;
                for (StripGroup& sg : me.stripGroups) {
                    totalStripGroups++;
                    totalVerts += static_cast<int>(sg.verts.size());
                    totalIndices += static_cast<int>(sg.indices.size());
                    for (Strip& st : sg.strips) {
                        totalStrips++;
                        totalBoneStateChanges += st.numBoneStateChanges;
                    }
                }
            }
        }
    }

    // hmm: totalMeshes must count ALL studio meshes (even for blank models the
    // LOD has zero meshes). The reference counts per ProcessModel iteration:
    // stats.m_TotalMeshes increments per studio mesh visited; blank models are
    // skipped before the mesh loop (no lod source). The tally above already
    // matches that (meshes only added for non-blank models).

    size_t bodyPartsOffset = sizeof(fmt::vtx::FileHeader_t);
    size_t modelsOffset = bodyPartsOffset + sizeof(fmt::vtx::BodyPartHeader_t) * totalBodyParts;
    size_t modelLODsOffset = modelsOffset + sizeof(fmt::vtx::ModelHeader_t) * totalModels;
    size_t meshesOffset = modelLODsOffset + sizeof(fmt::vtx::ModelLODHeader_t) * totalModelLODs;
    size_t stripGroupsOffset = meshesOffset + sizeof(fmt::vtx::MeshHeader_t) * totalMeshes;
    size_t stripGroupSize = legacyVtx ? sizeof(fmt::vtx::LegacyStripGroupHeader_t)
                                      : sizeof(fmt::vtx::StripGroupHeader_t);
    size_t stripSize =
        legacyVtx ? sizeof(fmt::vtx::LegacyStripHeader_t) : sizeof(fmt::vtx::StripHeader_t);
    size_t stripsOffset = stripGroupsOffset + stripGroupSize * totalStripGroups;
    size_t vertsOffset = stripsOffset + stripSize * totalStrips;
    size_t indicesOffset = vertsOffset + sizeof(fmt::vtx::Vertex_t) * totalVerts;
    size_t boneStateChangesOffset = indicesOffset + sizeof(uint16_t) * totalIndices;
    // AddMaterialReplacementsToStringTable: only the
    // replacement names go in, LOD order then entry order, deduped
    // case-insensitively. Offsets are the running byte position.
    std::vector<std::string> stringTable;
    std::map<std::string, int> stringOffsets; // lowercased name -> byte offset
    size_t stringTableSize = 0;
    for (const cm::ScriptLod& lod : m.scriptLods) {
        for (const cm::LodReplacement& r : lod.materialReplacements) {
            std::string key = r.dst;
            for (char& ch : key)
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            if (stringOffsets.count(key))
                continue;
            stringOffsets[key] = static_cast<int>(stringTableSize);
            stringTable.push_back(r.dst);
            stringTableSize += r.dst.size() + 1;
        }
    }

    int totalMaterialReplacements = 0;
    for (const cm::ScriptLod& lod : m.scriptLods)
        totalMaterialReplacements += static_cast<int>(lod.materialReplacements.size());

    size_t stringTableOffset =
        boneStateChangesOffset + sizeof(fmt::vtx::BoneStateChangeHeader_t) * totalBoneStateChanges;
    size_t materialReplacementsOffset = stringTableOffset + stringTableSize;
    size_t materialReplacementsListOffset =
        materialReplacementsOffset +
        totalMaterialReplacements * sizeof(fmt::vtx::MaterialReplacementHeader_t);
    size_t topologyOffset =
        materialReplacementsListOffset + numLODs * sizeof(fmt::vtx::MaterialReplacementListHeader_t);
    size_t endOfFile = topologyOffset; // legacy: no topology block

    // reference section report, deferred to save time
    {
        auto line = [](const char* label, size_t bytes) {
            char b[128];
            std::snprintf(b, sizeof(b), "%-13s %7zu bytes", label, bytes);
            g_vtxReport.emplace_back(b);
        };
        g_vtxReport.clear();
        line("body parts:", modelsOffset - bodyPartsOffset);
        line("models:", meshesOffset - modelsOffset);
        line("model LODs:", meshesOffset - modelLODsOffset);
        line("meshes:", stripGroupsOffset - meshesOffset);
        line("strip groups:", stripsOffset - stripGroupsOffset);
        line("strips:", vertsOffset - stripsOffset);
        line("verts:", indicesOffset - vertsOffset);
        line("indices:", boneStateChangesOffset - indicesOffset);
        line("bone changes:", endOfFile - boneStateChangesOffset);
        line("everything:", endOfFile);
    }

    std::vector<uint8_t> out(endOfFile, 0);

    // WriteStringTable
    {
        size_t at = stringTableOffset;
        for (const std::string& s : stringTable) {
            memcpy(out.data() + at, s.c_str(), s.size() + 1);
            at += s.size() + 1;
        }
    }

    // WriteMaterialReplacements / WriteMaterialReplacementLists
    {
        size_t replacementAt = materialReplacementsOffset;
        size_t listAt = materialReplacementsListOffset;
        for (const cm::ScriptLod& lod : m.scriptLods) {
            for (const cm::LodReplacement& r : lod.materialReplacements) {
                fmt::vtx::MaterialReplacementHeader_t hdr;
                // FindMaterialByName: exact name match
                // against the texture table, then that texture's material slot
                hdr.materialID = -1;
                if (m.mats) {
                    for (size_t t = 0; t < m.mats->textures.size(); t++) {
                        if (_stricmp(m.mats->textures[t].name.c_str(), r.src.c_str()) != 0)
                            continue;
                        hdr.materialID = static_cast<int16_t>(m.mats->textures[t].material);
                        break;
                    }
                }
                std::string key = r.dst;
                for (char& ch : key)
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                hdr.replacementMaterialNameOffset = static_cast<int32_t>(
                    stringTableOffset + stringOffsets[key] - replacementAt);
                memcpy(out.data() + replacementAt, &hdr, sizeof(hdr));
                replacementAt += sizeof(hdr);
            }

            fmt::vtx::MaterialReplacementListHeader_t listHdr;
            listHdr.numReplacements = static_cast<int32_t>(lod.materialReplacements.size());
            // the list points at the FIRST of its own replacements, which the
            // running cursor has already moved past - so back it up
            listHdr.replacementOffset = static_cast<int32_t>(
                replacementAt -
                lod.materialReplacements.size() * sizeof(fmt::vtx::MaterialReplacementHeader_t) -
                listAt);
            memcpy(out.data() + listAt, &listHdr, sizeof(listHdr));
            listAt += sizeof(listHdr);
        }
    }

    int curModel = 0, curLOD = 0, curMesh = 0, curStrip = 0, curStripGroup = 0;
    int curVert = 0, curIndex = 0, curBoneStateChange = 0;
    int deltaModel = 0, deltaLOD = 0, deltaMesh = 0, deltaStrip = 0, deltaStripGroup = 0;
    int deltaVert = 0, deltaIndex = 0, deltaBoneStateChange = 0;

    modelIdx = 0;
    for (int bp = 0; bp < phdr->numbodyparts; bp++) {
        fmt::mstudiobodyparts_t* pBodyPart = b.bodypart(bp);
        for (int mo = 0; mo < pBodyPart->nummodels; mo++, modelIdx++) {
            VtxModel& vmodel = b.models[curModel + deltaModel];
            fmt::mstudiomodel_t* pStudioModel = b.model(pBodyPart, mo);
            for (int lodID = 0; lodID < numLODs; lodID++) {
                VtxModelLOD& vlod = vmodel.modelLODs[lodID];
                // empty for a blank model, or one this LOD removed
                for (size_t meshID = 0; meshID < vlod.meshes.size(); meshID++) {
                    VtxMesh& vmesh = vlod.meshes[meshID];
                    for (size_t sgID = 0; sgID < vmesh.stripGroups.size(); sgID++) {
                        StripGroup& sg = vmesh.stripGroups[sgID];

                        // verts
                        {
                            size_t off = vertsOffset +
                                         (curVert + deltaVert) * sizeof(fmt::vtx::Vertex_t);
                            memcpy(out.data() + off, sg.verts.data(),
                                   sg.verts.size() * sizeof(fmt::vtx::Vertex_t));
                            deltaVert += static_cast<int>(sg.verts.size());
                        }
                        // indices
                        {
                            size_t off = indicesOffset + (curIndex + deltaIndex) * sizeof(uint16_t);
                            memcpy(out.data() + off, sg.indices.data(),
                                   sg.indices.size() * sizeof(uint16_t));
                            deltaIndex += static_cast<int>(sg.indices.size());
                        }

                        for (size_t stripID = 0; stripID < sg.strips.size(); stripID++) {
                            Strip& strip = sg.strips[stripID];
                            for (int bsc = 0; bsc < strip.numBoneStateChanges; bsc++) {
                                fmt::vtx::BoneStateChangeHeader_t bh;
                                bh.hardwareID = strip.boneStateChanges[bsc].hardwareID;
                                bh.newBoneID = strip.boneStateChanges[bsc].newBoneID;
                                size_t off = boneStateChangesOffset +
                                             (curBoneStateChange + deltaBoneStateChange) *
                                                 sizeof(fmt::vtx::BoneStateChangeHeader_t);
                                memcpy(out.data() + off, &bh, sizeof(bh));
                                deltaBoneStateChange++;
                            }
                            // WriteStrip
                            size_t stripFileOffset =
                                stripsOffset + (curStrip + deltaStrip) * stripSize;
                            if (legacyVtx) {
                                fmt::vtx::LegacyStripHeader_t sh{};
                                sh.numIndices = strip.numStripGroupIndices;
                                sh.indexOffset = strip.stripGroupIndexOffset;
                                sh.numVerts = strip.numStripGroupVerts;
                                sh.vertOffset = strip.stripGroupVertexOffset;
                                sh.numBones = static_cast<int16_t>(strip.numBones);
                                sh.flags = strip.flags;
                                sh.numBoneStateChanges = strip.numBoneStateChanges;
                                sh.boneStateChangeOffset = static_cast<int32_t>(
                                    (boneStateChangesOffset +
                                     curBoneStateChange * sizeof(fmt::vtx::BoneStateChangeHeader_t)) -
                                    stripFileOffset);
                                memcpy(out.data() + stripFileOffset, &sh, sizeof(sh));
                            } else {
                                fmt::vtx::StripHeader_t sh{};
                                sh.numIndices = strip.numStripGroupIndices;
                                sh.indexOffset = strip.stripGroupIndexOffset;
                                sh.numVerts = strip.numStripGroupVerts;
                                sh.vertOffset = strip.stripGroupVertexOffset;
                                sh.numBones = static_cast<int16_t>(strip.numBones);
                                sh.flags = strip.flags;
                                sh.numBoneStateChanges = strip.numBoneStateChanges;
                                sh.boneStateChangeOffset = static_cast<int32_t>(
                                    (boneStateChangesOffset +
                                     curBoneStateChange * sizeof(fmt::vtx::BoneStateChangeHeader_t)) -
                                    stripFileOffset);
                                sh.numTopologyIndices = 0;
                                sh.topologyOffset = 0;
                                memcpy(out.data() + stripFileOffset, &sh, sizeof(sh));
                            }
                            deltaStrip++;
                            curBoneStateChange += deltaBoneStateChange;
                            deltaBoneStateChange = 0;
                        }

                        // WriteStripGroup
                        size_t sgFileOffset =
                            stripGroupsOffset + (curStripGroup + deltaStripGroup) * stripGroupSize;
                        if (legacyVtx) {
                            fmt::vtx::LegacyStripGroupHeader_t sgh{};
                            sgh.numVerts = static_cast<int32_t>(sg.verts.size());
                            sgh.numIndices = static_cast<int32_t>(sg.indices.size());
                            sgh.numStrips = static_cast<int32_t>(sg.strips.size());
                            sgh.flags = sg.flags;
                            sgh.vertOffset = static_cast<int32_t>(
                                (vertsOffset + curVert * sizeof(fmt::vtx::Vertex_t)) - sgFileOffset);
                            sgh.indexOffset = static_cast<int32_t>(
                                (indicesOffset + curIndex * sizeof(uint16_t)) - sgFileOffset);
                            sgh.stripOffset = static_cast<int32_t>(
                                (stripsOffset + curStrip * stripSize) - sgFileOffset);
                            memcpy(out.data() + sgFileOffset, &sgh, sizeof(sgh));
                        } else {
                            fmt::vtx::StripGroupHeader_t sgh{};
                            sgh.numVerts = static_cast<int32_t>(sg.verts.size());
                            sgh.numIndices = static_cast<int32_t>(sg.indices.size());
                            sgh.numTopologyIndices = 0;
                            sgh.numStrips = static_cast<int32_t>(sg.strips.size());
                            sgh.flags = sg.flags;
                            sgh.vertOffset = static_cast<int32_t>(
                                (vertsOffset + curVert * sizeof(fmt::vtx::Vertex_t)) - sgFileOffset);
                            sgh.indexOffset = static_cast<int32_t>(
                                (indicesOffset + curIndex * sizeof(uint16_t)) - sgFileOffset);
                            sgh.topologyOffset = static_cast<int32_t>(topologyOffset - sgFileOffset);
                            sgh.stripOffset = static_cast<int32_t>(
                                (stripsOffset + curStrip * stripSize) - sgFileOffset);
                            memcpy(out.data() + sgFileOffset, &sgh, sizeof(sgh));
                        }
                        deltaStripGroup++;
                        curStrip += deltaStrip;
                        deltaStrip = 0;
                        curVert += deltaVert;
                        deltaVert = 0;
                        curIndex += deltaIndex;
                        deltaIndex = 0;
                    }
                    // WriteMesh
                    {
                        fmt::vtx::MeshHeader_t mh{};
                        mh.numStripGroups = static_cast<int32_t>(vmesh.stripGroups.size());
                        size_t meshFileOffset =
                            meshesOffset + (curMesh + deltaMesh) * sizeof(fmt::vtx::MeshHeader_t);
                        size_t sgFileOffset = stripGroupsOffset + curStripGroup * stripGroupSize;
                        mh.stripGroupHeaderOffset =
                            static_cast<int32_t>(sgFileOffset - meshFileOffset);
                        mh.flags = vmesh.flags;
                        memcpy(out.data() + meshFileOffset, &mh, sizeof(mh));
                        deltaMesh++;
                        curStripGroup += deltaStripGroup;
                        deltaStripGroup = 0;
                    }
                }
                // WriteModelLOD
                {
                    fmt::vtx::ModelLODHeader_t lh{};
                    size_t lodFileOffset =
                        modelLODsOffset + (curLOD + deltaLOD) * sizeof(fmt::vtx::ModelLODHeader_t);
                    size_t meshFileOffset =
                        meshesOffset + curMesh * sizeof(fmt::vtx::MeshHeader_t);
                    lh.meshOffset = static_cast<int32_t>(meshFileOffset - lodFileOffset);
                    lh.numMeshes = static_cast<int32_t>(vlod.meshes.size());
                    lh.switchPoint = vlod.switchPoint;
                    memcpy(out.data() + lodFileOffset, &lh, sizeof(lh));
                    deltaLOD++;
                    curMesh += deltaMesh;
                    deltaMesh = 0;
                }
            }
            // WriteModel
            {
                fmt::vtx::ModelHeader_t mh{};
                mh.numLODs = numLODs;
                size_t modelFileOffset =
                    modelsOffset + (curModel + deltaModel) * sizeof(fmt::vtx::ModelHeader_t);
                size_t lodFileOffset = modelLODsOffset + curLOD * sizeof(fmt::vtx::ModelLODHeader_t);
                mh.lodOffset = static_cast<int32_t>(lodFileOffset - modelFileOffset);
                memcpy(out.data() + modelFileOffset, &mh, sizeof(mh));
                deltaModel++;
                curLOD += deltaLOD;
                deltaLOD = 0;
            }
        }
        // WriteBodyPart
        {
            fmt::vtx::BodyPartHeader_t bph{};
            bph.numModels = pBodyPart->nummodels;
            size_t bodyPartOffset = bodyPartsOffset + bp * sizeof(fmt::vtx::BodyPartHeader_t);
            size_t modelFileOffset = modelsOffset + curModel * sizeof(fmt::vtx::ModelHeader_t);
            bph.modelOffset = static_cast<int32_t>(modelFileOffset - bodyPartOffset);
            memcpy(out.data() + bodyPartOffset, &bph, sizeof(bph));
            curModel += deltaModel;
            deltaModel = 0;
        }
    }

    // WriteHeader
    {
        fmt::vtx::FileHeader_t fh{};
        fh.version = fmt::vtx::kOptimizedModelFileVersion;
        fh.vertCacheSize = 24;
        fh.maxBonesPerFace = 9;
        fh.maxBonesPerVert = 3;
        fh.maxBonesPerStrip = 53;
        fh.numBodyParts = totalBodyParts;
        fh.bodyPartOffset = sizeof(fmt::vtx::FileHeader_t);
        fh.checkSum = phdr->checksum;
        fh.numLODs = numLODs;
        fh.materialReplacementListOffset = static_cast<int32_t>(materialReplacementsListOffset);
        memcpy(out.data(), &fh, sizeof(fh));
    }

    // MapGlobalBonesToHardwareBoneIDsAndSortBones
    {
        std::vector<int> globalToHardware(b.numBones, -1);
        fmt::vtx::FileHeader_t* header = reinterpret_cast<fmt::vtx::FileHeader_t*>(out.data());
        for (int bodyPartID = 0; bodyPartID < header->numBodyParts; bodyPartID++) {
            auto* bodyPart = reinterpret_cast<fmt::vtx::BodyPartHeader_t*>(
                out.data() + header->bodyPartOffset) + bodyPartID;
            for (int lodID = 0; lodID < header->numLODs; lodID++) {
                for (int i = 0; i < b.numBones; i++)
                    globalToHardware[i] = -1;
                for (int modelID = 0; modelID < bodyPart->numModels; modelID++) {
                    auto* mh = reinterpret_cast<fmt::vtx::ModelHeader_t*>(
                        reinterpret_cast<uint8_t*>(bodyPart) + bodyPart->modelOffset) + modelID;
                    auto* lh = reinterpret_cast<fmt::vtx::ModelLODHeader_t*>(
                        reinterpret_cast<uint8_t*>(mh) + mh->lodOffset) + lodID;
                    for (int meshID = 0; meshID < lh->numMeshes; meshID++) {
                        auto* meshHdr = reinterpret_cast<fmt::vtx::MeshHeader_t*>(
                            reinterpret_cast<uint8_t*>(lh) + lh->meshOffset) + meshID;
                        for (int sgID = 0; sgID < meshHdr->numStripGroups; sgID++) {
                            uint8_t* sgRaw = reinterpret_cast<uint8_t*>(meshHdr) +
                                             meshHdr->stripGroupHeaderOffset + sgID * stripGroupSize;
                            int32_t sgNumVerts = *reinterpret_cast<int32_t*>(sgRaw);
                            int32_t sgVertOffset = *reinterpret_cast<int32_t*>(sgRaw + 4);
                            int32_t sgNumStrips = *reinterpret_cast<int32_t*>(sgRaw + 16);
                            int32_t sgStripOffset = *reinterpret_cast<int32_t*>(sgRaw + 20);
                            // flags sits at +24 in both layouts (the six shared
                            // int32s); same for the strip field offsets below
                            uint8_t sgFlags = *(sgRaw + 24);
                            if (!(sgFlags & fmt::vtx::STRIPGROUP_IS_HWSKINNED))
                                continue;
                            for (int stripID = 0; stripID < sgNumStrips; stripID++) {
                                uint8_t* stRaw = sgRaw + sgStripOffset + stripID * stripSize;
                                int32_t stNumVerts = *reinterpret_cast<int32_t*>(stRaw + 8);
                                int32_t stVertOffset = *reinterpret_cast<int32_t*>(stRaw + 12);
                                int32_t stNumBSC = *reinterpret_cast<int32_t*>(stRaw + 19);
                                int32_t stBSCOffset = *reinterpret_cast<int32_t*>(stRaw + 23);
                                auto* bsc = reinterpret_cast<fmt::vtx::BoneStateChangeHeader_t*>(
                                    stRaw + stBSCOffset);
                                for (int c = 0; c < stNumBSC; c++)
                                    globalToHardware[bsc[c].newBoneID] = bsc[c].hardwareID;
                                for (int vID = 0; vID < stNumVerts; vID++) {
                                    auto* vert = reinterpret_cast<fmt::vtx::Vertex_t*>(
                                        sgRaw + sgVertOffset) + (vID + stVertOffset);
                                    for (int boneID = 0; boneID < header->maxBonesPerVert;
                                         boneID++) {
                                        int globalBoneID = vert->boneID[boneID];
                                        if (globalBoneID == 255) {
                                            vert->boneID[boneID] = 0;
                                            continue;
                                        }
                                        vert->boneID[boneID] =
                                            static_cast<uint8_t>(globalToHardware[globalBoneID]);
                                    }
                                }
                            }
                            (void)sgNumVerts;
                        }
                    }
                }
            }
        }
    }

    // ZeroNumBones, gated on g_staticprop && !g_bLegacyVTX
    // It hits every vert and strip of every strip group,
    // so walking the flat verts/strips blocks is the same set.
    // (RemoveRedundantBoneStateChanges runs here too, but its whole mutation
    // body is commented out in the reference - it is a no-op)
    if (!legacyVtx && (phdr->flags & fmt::STUDIOHDR_FLAGS_STATIC_PROP)) {
        for (int i = 0; i < totalVerts; i++)
            (reinterpret_cast<fmt::vtx::Vertex_t*>(out.data() + vertsOffset) + i)->numBones = 0;
        for (int i = 0; i < totalStrips; i++) {
            auto* st = reinterpret_cast<fmt::vtx::StripHeader_t*>(out.data() + stripsOffset +
                                                                 i * stripSize);
            st->numBones = 0;
            st->numBoneStateChanges = 0;
        }
    }

    return out;
}

} // namespace pulse::writer
