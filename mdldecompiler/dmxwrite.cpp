// dmxwrite.cpp - rebuilds a DMX render mesh per model in the .mdl.
//
// Binary `format model 15` by default, keyvalues2 text on -dmxencoding; either
// way it is flat - every element sits at top level with an id and is referenced
// by guid. Positions/normals/UVs/skinning come from the .vvd, faces from the
// .vtx, morph deltas from the .mdl's flexes.
//
// Every LOD the .vtx carries gets its own file - LOD 0 under the $rendermesh
// alias, the rest as <alias>_lod<n>.dmx for the $lod blocks to replacemodel
// against. Triangle-list strips only; nothing in the Source 1 pipeline emits a
// quad list.
//
// The .phy's collision hulls come out the same way, as one more mesh with the
// same skeleton - see WritePhysicsMesh.

#include "dmxwrite.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <system_error>
#include <vector>

#include "format/phy.h"
#include "format/vtx.h"
#include "format/vvd.h"
#include "math/compressed.h"
#include "minicollision/ivp_compact.h"

namespace mdldecompiler {
namespace {

namespace vtx = pulse::format::vtx;

// %.9g round-trips a float exactly, so recompiling what we write reproduces the
// vertex bit for bit. F()'s %.6f is for a human reading a .pulseqc, not for this.
std::string G(float v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.9g", v);
    return b;
}

// --- .vvd -------------------------------------------------------------------

// The vertex array for one LOD. With fixups the on-disk block is grouped by
// LOD; the runs with `lod >= n` concatenated in table order are the array that
// mstudiomodel_t::vertexindex and mstudiomesh_t::vertexoffset are measured
// against at LOD n - a lower LOD simply drops the runs that do not reach it. A
// single-LOD model has no fixups and the block is already that array.
bool BuildLodVerts(const std::vector<char>& buf, int lod,
                   std::vector<fm::mstudiovertex_t>& out) {
    out.clear();
    if (buf.size() < sizeof(fm::vertexFileHeader_t))
        return false;
    const auto* h = reinterpret_cast<const fm::vertexFileHeader_t*>(buf.data());
    if (h->id != fm::kIdVertexFile || h->version != fm::kVertexFileVersion)
        return false;
    if (h->vertexDataStart < 0 || static_cast<size_t>(h->vertexDataStart) > buf.size())
        return false;

    const auto* verts =
        reinterpret_cast<const fm::mstudiovertex_t*>(buf.data() + h->vertexDataStart);
    const size_t avail =
        (buf.size() - static_cast<size_t>(h->vertexDataStart)) / sizeof(fm::mstudiovertex_t);

    if (h->numFixups <= 0) {
        const int lodv = h->numLODVertexes[lod < fm::kMaxNumLods ? lod : 0];
        size_t n = avail;
        if (lodv > 0 && static_cast<size_t>(lodv) < n)
            n = static_cast<size_t>(lodv);
        out.assign(verts, verts + n);
        return !out.empty();
    }

    if (h->fixupTableStart < 0 ||
        static_cast<size_t>(h->fixupTableStart) +
                static_cast<size_t>(h->numFixups) * sizeof(fm::vertexFileFixup_t) >
            buf.size())
        return false;
    const auto* fx =
        reinterpret_cast<const fm::vertexFileFixup_t*>(buf.data() + h->fixupTableStart);
    for (int i = 0; i < h->numFixups; ++i) {
        if (fx[i].lod < 0 || fx[i].sourceVertexID < 0 || fx[i].numVertexes < 0 ||
            static_cast<size_t>(fx[i].sourceVertexID) + static_cast<size_t>(fx[i].numVertexes) >
                avail)
            return false;
        if (fx[i].lod < lod)
            continue; // this run does not reach the LOD being built
        out.insert(out.end(), verts + fx[i].sourceVertexID,
                   verts + fx[i].sourceVertexID + fx[i].numVertexes);
    }
    return !out.empty();
}

// --- .vtx -------------------------------------------------------------------

using MeshTris = std::vector<int>;                    // model-relative, 3 per triangle
using ModelTris = std::vector<MeshTris>;              // per mesh
using PartTris = std::vector<std::vector<ModelTris>>; // [bodypart][model][mesh]

// Where one model's vertices sit inside a given LOD's vertex array. The .mdl
// only stores the ROOT LOD's vertexindex/vertexoffset; a lower LOD keeps just
// mstudiomesh_t::vertexdata.numLODVertexes[n] verts per mesh, packed in the
// same bodypart -> model -> mesh order the fixup table was written in. So the
// offsets are re-derived by running that walk rather than read off the file.
struct ModelLod {
    int base = 0;  // into the LOD vertex array
    int count = 0; // verts this model owns at this LOD
    std::vector<int> meshOffset, meshCount; // within the model
};
using LodLayout = std::vector<std::vector<ModelLod>>; // [bodypart][model]

LodLayout BuildLodLayout(const Mdl& m, int lod) {
    const fm::studiohdr_t& h = *m.hdr;
    LodLayout out;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    int running = 0;
    for (int i = 0; parts && i < h.numbodyparts; ++i) {
        out.emplace_back();
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            ModelLod ml;
            ml.base = running;
            const fm::mstudiomesh_t* meshes =
                m.At<fm::mstudiomesh_t>(&models[j], models[j].meshindex, models[j].nummeshes);
            for (int k = 0; meshes && k < models[j].nummeshes; ++k) {
                const int n = meshes[k].vertexdata.numLODVertexes[lod];
                ml.meshOffset.push_back(ml.count);
                ml.meshCount.push_back(n > 0 ? n : 0);
                ml.count += (n > 0 ? n : 0);
            }
            running += ml.count;
            out.back().push_back(std::move(ml));
        }
    }
    return out;
}

// The strip-group header grew topology fields in the Alien Swarm era without a
// version bump (see vtx.h), so the array stride is 25 or 33 and nothing in the
// file says which. Only the first four fields are read, and those are common to
// both - so the stride is found by parsing with each and keeping the one whose
// offsets and indices all land in bounds.
bool ExtractTris(const std::vector<char>& buf, const Mdl& m, size_t sgStride, int lod,
                 const LodLayout& layout, PartTris& out) {
    out.clear();
    const fm::studiohdr_t& h = *m.hdr;
    const char* base = buf.data();
    const char* end = base + buf.size();
    auto ok = [&](const void* p, size_t bytes) {
        const char* c = static_cast<const char*>(p);
        return c >= base && c <= end && bytes <= static_cast<size_t>(end - c);
    };
    // offsets are relative to the struct that holds them
    auto at = [&](const void* owner, int32_t off, size_t bytes) -> const char* {
        const char* c = static_cast<const char*>(owner) + off;
        return ok(c, bytes) ? c : nullptr;
    };

    if (buf.size() < sizeof(vtx::FileHeader_t))
        return false;
    const auto* fh = reinterpret_cast<const vtx::FileHeader_t*>(base);
    if (fh->version != vtx::kOptimizedModelFileVersion || fh->numBodyParts != h.numbodyparts)
        return false;

    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    if (!parts)
        return false;

    const char* bpRaw = at(fh, fh->bodyPartOffset,
                           sizeof(vtx::BodyPartHeader_t) * static_cast<size_t>(fh->numBodyParts));
    if (!bpRaw)
        return false;
    const auto* bps = reinterpret_cast<const vtx::BodyPartHeader_t*>(bpRaw);

    out.resize(static_cast<size_t>(h.numbodyparts));
    for (int i = 0; i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* mdlModels =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        if (!mdlModels || bps[i].numModels != parts[i].nummodels)
            return false;
        const char* mRaw = at(&bps[i], bps[i].modelOffset,
                              sizeof(vtx::ModelHeader_t) * static_cast<size_t>(bps[i].numModels));
        if (!mRaw)
            return false;
        const auto* models = reinterpret_cast<const vtx::ModelHeader_t*>(mRaw);

        out[i].resize(static_cast<size_t>(parts[i].nummodels));
        for (int j = 0; j < parts[i].nummodels; ++j) {
            const fm::mstudiomesh_t* mdlMeshes = m.At<fm::mstudiomesh_t>(
                &mdlModels[j], mdlModels[j].meshindex, mdlModels[j].nummeshes);
            out[i][j].resize(static_cast<size_t>(mdlModels[j].nummeshes));
            // a blank body has no meshes at all, in either file
            if (models[j].numLODs <= lod || mdlModels[j].nummeshes <= 0)
                continue;
            const char* lodRaw =
                at(&models[j], models[j].lodOffset,
                   sizeof(vtx::ModelLODHeader_t) * static_cast<size_t>(models[j].numLODs));
            if (!lodRaw)
                return false;
            const auto& lodHdr =
                reinterpret_cast<const vtx::ModelLODHeader_t*>(lodRaw)[lod];
            if (!mdlMeshes || lodHdr.numMeshes != mdlModels[j].nummeshes)
                return false;
            const char* meshRaw = at(&lodHdr, lodHdr.meshOffset,
                                     sizeof(vtx::MeshHeader_t) * static_cast<size_t>(lodHdr.numMeshes));
            if (!meshRaw)
                return false;
            const auto* meshes = reinterpret_cast<const vtx::MeshHeader_t*>(meshRaw);

            const ModelLod& ml = layout[i][j];
            for (int k = 0; k < lodHdr.numMeshes && static_cast<size_t>(k) < ml.meshCount.size();
                 ++k) {
                MeshTris& tris = out[i][j][k];
                const int meshVerts = ml.meshCount[k];
                if (meshes[k].numStripGroups <= 0 || meshVerts <= 0)
                    continue;
                const char* sgRaw = at(&meshes[k], meshes[k].stripGroupHeaderOffset,
                                       sgStride * static_cast<size_t>(meshes[k].numStripGroups));
                if (!sgRaw)
                    return false;

                for (int g = 0; g < meshes[k].numStripGroups; ++g) {
                    const char* sgp = sgRaw + sgStride * static_cast<size_t>(g);
                    // the leading four fields are identical in both layouts
                    const auto& sg = *reinterpret_cast<const vtx::LegacyStripGroupHeader_t*>(sgp);
                    if (sg.numVerts < 0 || sg.numIndices < 0)
                        return false;
                    if (sg.numVerts == 0 || sg.numIndices < 3)
                        continue;
                    const char* vRaw = at(sgp, sg.vertOffset,
                                          sizeof(vtx::Vertex_t) * static_cast<size_t>(sg.numVerts));
                    const char* iRaw = at(sgp, sg.indexOffset,
                                          sizeof(uint16_t) * static_cast<size_t>(sg.numIndices));
                    if (!vRaw || !iRaw)
                        return false;
                    const auto* sgVerts = reinterpret_cast<const vtx::Vertex_t*>(vRaw);
                    const auto* idx = reinterpret_cast<const uint16_t*>(iRaw);

                    // Strips within a group are consecutive runs of the same
                    // index array and are all trilists, so the group's whole
                    // index array reads as triangles without walking them.
                    for (int n = 0; n + 2 < sg.numIndices; n += 3) {
                        for (int c = 0; c < 3; ++c) {
                            const uint16_t vi = idx[n + c];
                            if (vi >= sg.numVerts)
                                return false;
                            const int mv = sgVerts[vi].origMeshVertID;
                            if (mv >= meshVerts)
                                return false;
                            tris.push_back(ml.meshOffset[k] + mv);
                        }
                    }
                }
            }
        }
    }
    return true;
}

// The $lod switch value, taken off the first model that has this LOD. A
// negative one is the $shadowlod.
float LodSwitchPoint(const std::vector<char>& buf, const Mdl& m, int lod) {
    const auto* fh = reinterpret_cast<const vtx::FileHeader_t*>(buf.data());
    const char* end = buf.data() + buf.size();
    auto at = [&](const void* owner, int32_t off, size_t bytes) -> const char* {
        const char* c = static_cast<const char*>(owner) + off;
        return (c >= buf.data() && c <= end && bytes <= static_cast<size_t>(end - c)) ? c
                                                                                     : nullptr;
    };
    const char* bpRaw = at(fh, fh->bodyPartOffset,
                           sizeof(vtx::BodyPartHeader_t) * static_cast<size_t>(fh->numBodyParts));
    if (!bpRaw)
        return 0.0f;
    const auto* bps = reinterpret_cast<const vtx::BodyPartHeader_t*>(bpRaw);
    for (int i = 0; i < fh->numBodyParts; ++i) {
        const char* mRaw = at(&bps[i], bps[i].modelOffset,
                              sizeof(vtx::ModelHeader_t) * static_cast<size_t>(bps[i].numModels));
        if (!mRaw)
            continue;
        const auto* models = reinterpret_cast<const vtx::ModelHeader_t*>(mRaw);
        for (int j = 0; j < bps[i].numModels; ++j) {
            if (models[j].numLODs <= lod)
                continue;
            const char* lodRaw =
                at(&models[j], models[j].lodOffset,
                   sizeof(vtx::ModelLODHeader_t) * static_cast<size_t>(models[j].numLODs));
            if (lodRaw)
                return reinterpret_cast<const vtx::ModelLODHeader_t*>(lodRaw)[lod].switchPoint;
        }
    }
    return 0.0f;
}

// $lod replacematerial. One list per LOD; an entry names the texture it swaps
// out by id and the replacement by a raw string, so the replacement never joins
// the model's material table.
std::vector<std::pair<std::string, std::string>> LodMaterialReplacements(
    const std::vector<char>& buf, const Mdl& m, int lod) {
    std::vector<std::pair<std::string, std::string>> out;
    const auto* fh = reinterpret_cast<const vtx::FileHeader_t*>(buf.data());
    const char* end = buf.data() + buf.size();
    auto at = [&](const void* owner, int32_t off, size_t bytes) -> const char* {
        const char* c = static_cast<const char*>(owner) + off;
        return (c >= buf.data() && c <= end && bytes <= static_cast<size_t>(end - c)) ? c
                                                                                     : nullptr;
    };
    const char* listRaw =
        at(fh, fh->materialReplacementListOffset,
           sizeof(vtx::MaterialReplacementListHeader_t) * static_cast<size_t>(fh->numLODs));
    if (!listRaw)
        return out;
    const auto& list =
        reinterpret_cast<const vtx::MaterialReplacementListHeader_t*>(listRaw)[lod];
    const char* repRaw =
        at(&list, list.replacementOffset,
           sizeof(vtx::MaterialReplacementHeader_t) * static_cast<size_t>(list.numReplacements));
    if (!repRaw)
        return out;

    const std::vector<std::string> texNames = TextureNames(m);
    for (int i = 0; i < list.numReplacements; ++i) {
        const auto& r = reinterpret_cast<const vtx::MaterialReplacementHeader_t*>(
            repRaw)[i];
        const char* name = at(&r, r.replacementMaterialNameOffset, 1);
        if (!name || r.materialID < 0 || static_cast<size_t>(r.materialID) >= texNames.size())
            continue;
        // the raw string has to terminate inside the file
        const char* z = name;
        while (z < end && *z)
            ++z;
        if (z >= end)
            continue;
        out.emplace_back(BaseName(texNames[r.materialID]), std::string(name));
    }
    return out;
}

// --- the DMX emitter --------------------------------------------------------

// The binary encoding version that goes with a `format model <n>`: model 15 is
// binary 4 (int32 table count, uint16 indices, pooled string values), model 1 is
// binary 2 (uint16 table count, element names and string values written inline),
// model 18 is binary 5 (model 15's content, table indices widened to int32),
// model 22 (modeldoc) is binary 9 (int32 indices, a zero prefix-element count and
// array type ids as `scalar | 0x20`).
// Another model version gets its own case here; 0 means we do not write that one.
int BinaryVersionFor(int formatModel) {
    switch (formatModel) {
        case 1: return 2;
        case 15: return 4;
        case 18: return 5;
        case 22: return 9;
        default: return 0;
    }
}

// -dmxencoding / -dmxmodel, defaulting to what a Source 1 model wants.
bool g_binary = true;
int g_formatModel = 15;

// Binary attribute type ids for encoding versions 1-5: scalars 1..14, the
// matching array type 14 higher. Only the ones we emit are listed.
enum : uint8_t {
    kElement = 1, kInt = 2, kFloat = 3, kBool = 4, kString = 5, kTime = 7,
    kVector2 = 9, kVector3 = 10, kQuaternion = 13,
    kElementArray = 15, kIntArray = 16, kFloatArray = 17, kStringArray = 19,
    kTimeArray = 21, kVector2Array = 23, kVector3Array = 24, kQuaternionArray = 27,
};

// A time is seconds in memory and in keyvalues2, int ticks of 1/10000 s on disk.
int32_t Ticks(float seconds) {
    return static_cast<int32_t>(std::floor(10000.0f * seconds + 0.5f));
}

bool IsArray(uint8_t t) { return t >= kElementArray; }

// The on-disk type byte. Encoding 9 drops the contiguous array block and tags an
// array as `scalar | 0x20` instead.
uint8_t TypeId(uint8_t t, int ev) {
    if (ev < 9 || !IsArray(t))
        return t;
    return static_cast<uint8_t>((t - kElementArray + 1) | 0x20);
}

// Vertex-stream names. Model 22 (modeldoc) spells them stream-indexed; 15 and
// below use the long names. Anything not listed is the same in both.
const char* F(const char* n) {
    if (g_formatModel < 22)
        return n;
    static const struct { const char* v15; const char* v22; } kMap[] = {
        {"positions", "position$0"},   {"positionsIndices", "position$0Indices"},
        {"normals", "normal$0"},       {"normalsIndices", "normal$0Indices"},
        {"textureCoordinates", "texcoord$0"},
        {"textureCoordinatesIndices", "texcoord$0Indices"},
        {"jointWeights", "blendweights$0"}, {"jointIndices", "blendindices$0"},
        {"balance", "balance$0"},      {"balanceIndices", "balance$0Indices"},
        {"speed", "speed$0"},          {"speedIndices", "speed$0Indices"},
        {"wrinkle", "wrinkle$0"},      {"wrinkleIndices", "wrinkle$0Indices"},
    };
    for (const auto& p : kMap)
        if (std::strcmp(n, p.v15) == 0)
            return p.v22;
    return n;
}

const char* TypeName(uint8_t t) {
    switch (t) {
        case kElement: return "element";
        case kInt: return "int";
        case kFloat: return "float";
        case kBool: return "bool";
        case kString: return "string";
        case kTime: return "time";
        case kTimeArray: return "time_array";
        case kVector2: return "vector2";
        case kVector3: return "vector3";
        case kQuaternion: return "quaternion";
        case kElementArray: return "element_array";
        case kIntArray: return "int_array";
        case kFloatArray: return "float_array";
        case kStringArray: return "string_array";
        case kVector2Array: return "vector2_array";
        case kVector3Array: return "vector3_array";
        case kQuaternionArray: return "quaternion_array";
    }
    return "";
}

// Floats per value, so a float run splits back into vectors.
int Width(uint8_t t) {
    switch (t) {
        case kVector2: case kVector2Array: return 2;
        case kVector3: case kVector3Array: return 3;
        case kQuaternion: case kQuaternionArray: return 4;
        default: return 1;
    }
}

// One attribute. Every type we emit is a run of floats, of ints or of strings
// (element refs are held as the target's id), so the type tag alone says which
// of the three to read and how to group it.
struct Attr {
    std::string name;
    uint8_t type = 0;
    std::vector<float> f;
    std::vector<int> i;
    std::vector<std::string> s;
};

struct Elem {
    std::string cls, id, name;
    std::vector<Attr> attrs;
};

// FNV-1a, only ever used to spread mesh names across the id space.
uint32_t Hash(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s)
        h = (h ^ c) * 16777619u;
    return h;
}

// Elements are buffered, not streamed: binary wants the string table and the
// whole element list before any attribute, and refs resolve to element indices.
//
// Ids are derived, never random: the same .mdl decompiles to the same bytes
// twice, so a diff of two runs means something. The first three groups are a
// hash of the mesh name and the last is a counter, which keeps the ids of two
// meshes out of the same model apart when a tool loads both into one scene.
struct Dmx {
    std::FILE* f;
    uint32_t seed = 0;
    int next = 1;
    std::vector<Elem> elems;

    std::string NewId() {
        char b[48];
        std::snprintf(b, sizeof b, "%08x-%04x-%04x-%04x-%012x", seed,
                      static_cast<unsigned>(seed >> 16) & 0xffffu,
                      static_cast<unsigned>(seed) & 0xffffu,
                      static_cast<unsigned>(seed >> 8) & 0xffffu,
                      static_cast<unsigned>(next++));
        return b;
    }
    void Begin(const char* cls, const std::string& id, const std::string& name) {
        elems.push_back(Elem{cls, id, name, {}});
    }
    void End() {} // blocks are implicit; kept so the emit sites still read as blocks

    Attr& Push(const char* k, uint8_t t) {
        elems.back().attrs.push_back(Attr{k, t, {}, {}, {}});
        return elems.back().attrs.back();
    }
    void Str(const char* k, const std::string& v) { Push(k, kString).s.push_back(v); }
    void Time(const char* k, float seconds) { Push(k, kTime).f.push_back(seconds); }
    void Int(const char* k, int v) { Push(k, kInt).i.push_back(v); }
    void Float(const char* k, float v) { Push(k, kFloat).f.push_back(v); }
    void Bool(const char* k, bool v) { Push(k, kBool).i.push_back(v ? 1 : 0); }
    void Vec3(const char* k, const pm::Vector3& v) { Push(k, kVector3).f = {v.x, v.y, v.z}; }
    void Quat(const char* k, const pm::Quaternion& q) {
        Push(k, kQuaternion).f = {q.x, q.y, q.z, q.w};
    }
    void Ref(const char* k, const std::string& id) { Push(k, kElement).s.push_back(id); }
    void RefArray(const char* k, const std::vector<std::string>& ids) {
        Push(k, kElementArray).s = ids;
    }
    void IntArray(const char* k, const std::vector<int>& v) { Push(k, kIntArray).i = v; }
    void FloatArray(const char* k, const std::vector<float>& v) { Push(k, kFloatArray).f = v; }
    void TimeArray(const char* k, const std::vector<float>& v) { Push(k, kTimeArray).f = v; }
    void QuatArray(const char* k, const std::vector<pm::Quaternion>& v) {
        Attr& a = Push(k, kQuaternionArray);
        a.f.reserve(v.size() * 4);
        for (const pm::Quaternion& q : v) {
            a.f.push_back(q.x);
            a.f.push_back(q.y);
            a.f.push_back(q.z);
            a.f.push_back(q.w);
        }
    }
    void V3Array(const char* k, const std::vector<pm::Vector3>& v) {
        Attr& a = Push(k, kVector3Array);
        a.f.reserve(v.size() * 3);
        for (const pm::Vector3& p : v) {
            a.f.push_back(p.x);
            a.f.push_back(p.y);
            a.f.push_back(p.z);
        }
    }
    void V2Array(const char* k, const std::vector<pm::Vector2>& v) {
        Attr& a = Push(k, kVector2Array);
        a.f.reserve(v.size() * 2);
        for (const pm::Vector2& p : v) {
            a.f.push_back(p.x);
            a.f.push_back(p.y);
        }
    }
    void StrArray(const char* k, const std::vector<std::string>& v) {
        Push(k, kStringArray).s = v;
    }

    void Save() { g_binary ? SaveBinary() : SaveKv2(); }

    // --- keyvalues2 text ---
    void Kv2Attr(const Attr& a) {
        const int w = Width(a.type);
        if (!IsArray(a.type)) {
            std::string v;
            if (!a.s.empty())
                v = a.s[0];
            else if (!a.i.empty())
                v = std::to_string(a.i[0]);
            else
                for (int k = 0; k < w; ++k)
                    v += (k ? " " : "") + G(a.f[k]);
            std::fprintf(f, "\t\"%s\" \"%s\" \"%s\"\n", a.name.c_str(), TypeName(a.type),
                         v.c_str());
            return;
        }
        const size_t n = a.type == kIntArray ? a.i.size()
                       : a.f.empty()         ? a.s.size()
                                             : a.f.size() / static_cast<size_t>(w);
        std::fprintf(f, "\t\"%s\" \"%s\"\n\t[\n", a.name.c_str(), TypeName(a.type));
        for (size_t k = 0; k < n; ++k) {
            const char* sep = k + 1 < n ? "," : "";
            if (a.type == kElementArray) {
                std::fprintf(f, "\t\t\"element\" \"%s\"%s\n", a.s[k].c_str(), sep);
            } else if (a.type == kStringArray) {
                std::fprintf(f, "\t\t\"%s\"%s\n", a.s[k].c_str(), sep);
            } else if (a.type == kIntArray) {
                std::fprintf(f, "\t\t\"%d\"%s\n", a.i[k], sep);
            } else {
                std::string v;
                for (int c = 0; c < w; ++c)
                    v += (c ? " " : "") + G(a.f[k * static_cast<size_t>(w) + c]);
                std::fprintf(f, "\t\t\"%s\"%s\n", v.c_str(), sep);
            }
        }
        std::fprintf(f, "\t]\n");
    }

    void SaveKv2() {
        std::fprintf(f, "<!-- dmx encoding keyvalues2 1 format model %d -->\n\n", g_formatModel);
        for (const Elem& e : elems) {
            std::fprintf(f,
                         "\"%s\"\n{\n\t\"id\" \"elementid\" \"%s\"\n\t\"name\" \"string\" \"%s\"\n",
                         e.cls.c_str(), e.id.c_str(), e.name.c_str());
            for (const Attr& a : e.attrs)
                Kv2Attr(a);
            std::fprintf(f, "}\n\n");
        }
    }

    // --- binary ---
    void SaveBinary() {
        // The string table pools element class names and attribute names, plus -
        // from encoding 4 on - element names and scalar string values; before 4
        // those two are written inline. Array strings are always inline.
        const int ev = BinaryVersionFor(g_formatModel);
        const bool pooledValues = ev >= 4;
        const bool wideIndex = ev >= 5; // table indices widen to int32
        std::map<std::string, int> pool;
        std::vector<const std::string*> strings;
        auto intern = [&](const std::string& s) {
            const auto ins = pool.emplace(s, static_cast<int>(strings.size()));
            if (ins.second)
                strings.push_back(&ins.first->first); // map nodes are stable
            return ins.first->second;
        };
        std::map<std::string, int> byId;
        for (size_t k = 0; k < elems.size(); ++k) {
            byId[elems[k].id] = static_cast<int>(k);
            intern(elems[k].cls);
            if (pooledValues)
                intern(elems[k].name);
            for (const Attr& a : elems[k].attrs) {
                intern(a.name);
                if (a.type == kString && pooledValues)
                    intern(a.s[0]);
            }
        }

        auto u8 = [&](uint8_t v) { std::fwrite(&v, 1, 1, f); };
        auto u16 = [&](int v) {
            const uint16_t x = static_cast<uint16_t>(v);
            std::fwrite(&x, 2, 1, f);
        };
        auto i32 = [&](int32_t v) { std::fwrite(&v, 4, 1, f); };
        auto f32 = [&](float v) { std::fwrite(&v, 4, 1, f); };
        auto cstr = [&](const std::string& s) { std::fwrite(s.c_str(), 1, s.size() + 1, f); };
        auto sidx = [&](int v) {
            if (wideIndex)
                i32(v);
            else
                u16(v);
        };
        auto ref = [&](const std::string& id) {
            const auto it = byId.find(id);
            i32(it == byId.end() ? -1 : it->second); // -1 is a null reference
        };
        // the id string is 32 hex digits with dashes, in byte order
        auto guid = [&](const std::string& id) {
            auto hex = [](char c) { return c >= 'a' ? c - 'a' + 10 : c - '0'; };
            uint8_t g[16] = {};
            int n = 0;
            for (size_t k = 0; k + 1 < id.size() && n < 16; ++k) {
                if (id[k] == '-')
                    continue;
                g[n++] = static_cast<uint8_t>((hex(id[k]) << 4) | hex(id[k + 1]));
                ++k;
            }
            std::fwrite(g, 1, sizeof g, f);
        };

        // the header line is null-terminated, the way the reader expects it
        std::fprintf(f, "<!-- dmx encoding binary %d format model %d -->\n", ev, g_formatModel);
        u8(0);

        if (ev >= 9)
            i32(0); // prefix element count - nothing we write needs one

        // the table count widens to int32 at encoding 4
        if (pooledValues)
            i32(static_cast<int32_t>(strings.size()));
        else
            u16(static_cast<int>(strings.size()));
        for (const std::string* s : strings)
            cstr(*s);

        i32(static_cast<int32_t>(elems.size()));
        for (const Elem& e : elems) {
            sidx(pool[e.cls]);
            if (pooledValues)
                sidx(pool[e.name]);
            else
                cstr(e.name);
            guid(e.id);
        }

        for (const Elem& e : elems) {
            i32(static_cast<int32_t>(e.attrs.size()));
            for (const Attr& a : e.attrs) {
                sidx(pool[a.name]);
                u8(TypeId(a.type, ev));
                const int w = Width(a.type);
                switch (a.type) {
                    case kElement: ref(a.s[0]); break;
                    case kInt: i32(a.i[0]); break;
                    case kFloat: f32(a.f[0]); break;
                    case kBool: u8(static_cast<uint8_t>(a.i[0])); break;
                    case kTime: i32(Ticks(a.f[0])); break;
                    case kString:
                        if (pooledValues)
                            sidx(pool[a.s[0]]);
                        else
                            cstr(a.s[0]);
                        break;
                    case kVector2:
                    case kVector3:
                    case kQuaternion:
                        for (float v : a.f)
                            f32(v);
                        break;
                    case kElementArray:
                        i32(static_cast<int32_t>(a.s.size()));
                        for (const std::string& id : a.s)
                            ref(id);
                        break;
                    case kStringArray:
                        i32(static_cast<int32_t>(a.s.size()));
                        for (const std::string& s : a.s)
                            cstr(s);
                        break;
                    case kIntArray:
                        i32(static_cast<int32_t>(a.i.size()));
                        for (int v : a.i)
                            i32(v);
                        break;
                    case kTimeArray:
                        i32(static_cast<int32_t>(a.f.size()));
                        for (float v : a.f)
                            i32(Ticks(v));
                        break;
                    default: // float / vector2 / vector3 arrays
                        i32(static_cast<int32_t>(a.f.size() / static_cast<size_t>(w)));
                        for (float v : a.f)
                            f32(v);
                        break;
                }
            }
        }
    }
};

// One morph target, accumulated across the meshes of a model. Indices are
// model-relative vertex ids, which are also the data indices of the position /
// normal / texcoord streams below - those are written 1:1 with the .vvd.
struct Delta {
    std::vector<int> posIdx, normIdx, wrinkleIdx;
    std::vector<pm::Vector3> pos, norm;
    std::vector<float> wrinkle;
    bool stereo = false;
};

// --- the skeleton every model file carries ----------------------------------

// Ids are handed out in emit order, so allocation is split from writing: the
// caller takes its own ids in between, after the joints and before the mesh.
struct Skel {
    std::string idRoot, idModel, idModelXform, idBind, idAxis;
    std::vector<std::string> idJointDag, idJointXform;
    int numbones = 0;
};

Skel AllocSkel(Dmx& q, const Mdl& m) {
    Skel s;
    s.numbones = m.At<fm::mstudiobone_t>(m.buf.data(), m.hdr->boneindex, m.hdr->numbones)
                     ? m.hdr->numbones
                     : 0;
    s.idRoot = q.NewId();
    s.idModel = q.NewId();
    s.idModelXform = q.NewId();
    s.idJointDag.resize(s.numbones);
    s.idJointXform.resize(s.numbones);
    for (int i = 0; i < s.numbones; ++i) {
        s.idJointDag[i] = q.NewId();
        s.idJointXform[i] = q.NewId();
    }
    if (s.numbones)
        s.idBind = q.NewId();
    if (g_formatModel >= 22)
        s.idAxis = q.NewId();
    return s;
}

// The root DmElement, the DmeModel and one dag+transform per bone. `idMeshDag`
// is the caller's mesh, which hangs off the model beside the root bones; an
// animation file has none. `pose0` overrides the bind pose the joints are
// written at, for a clip whose frame 0 is its own reference pose.
void WriteSkel(Dmx& q, const Mdl& m, const Skel& s, const std::string& name,
               const std::string& idMeshDag, const std::string& idCombo,
               const std::string& idAnimList = std::string(),
               const std::vector<AnimPose>* pose0 = nullptr) {
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), m.hdr->boneindex, m.hdr->numbones);
    const std::vector<std::string> boneNames = BoneNames(m);
    const std::vector<LocalPose> poses = LocalPoses(m);

    // An animation file is `skeleton` only. A root that also carries `model`
    // reads as a mesh export, and an importer that types the file off that
    // attribute takes the ANIM branch away and drops every clip.
    q.Begin("DmElement", s.idRoot, name);
    if (idAnimList.empty())
        q.Ref("model", s.idModel);
    q.Ref("skeleton", s.idModel);
    if (!idCombo.empty())
        q.Ref("combinationOperator", idCombo);
    if (!idAnimList.empty())
        q.Ref("animationList", idAnimList);
    q.End();

    // The root bones and the mesh hang off `children`, and every bone is listed
    // in `jointList` in .mdl order so a vertex's stored bone index is its joint
    // index unchanged.
    std::vector<std::string> rootDags, allJoints;
    for (int i = 0; i < s.numbones; ++i) {
        allJoints.push_back(s.idJointDag[i]);
        if (bones[i].parent < 0)
            rootDags.push_back(s.idJointDag[i]);
    }
    if (!idMeshDag.empty())
        rootDags.push_back(idMeshDag);

    q.Begin("DmeModel", s.idModel, name);
    q.Str("upAxis", "Z");
    q.Ref("transform", s.idModelXform);
    q.RefArray("children", rootDags);
    q.RefArray("jointList", allJoints);
    if (!s.idBind.empty())
        q.RefArray("baseStates", {s.idBind});
    if (!s.idAxis.empty())
        q.Ref("axisSystem", s.idAxis);
    q.End();

    // model 22 states the source axes explicitly: Z up, Y forward, right-handed.
    if (!s.idAxis.empty()) {
        q.Begin("DmeAxisSystem", s.idAxis, "axisSystem");
        q.Int("upAxis", 3);
        q.Int("forwardParity", 1);
        q.Int("coordSys", 0);
        q.End();
    }

    q.Begin("DmeTransform", s.idModelXform, name);
    q.Vec3("position", {0.0f, 0.0f, 0.0f});
    q.Quat("orientation", {0.0f, 0.0f, 0.0f, 1.0f});
    q.End();

    // The joints are written at their bind pose, so the bind transform list is
    // the same elements again rather than a second copy of the numbers.
    if (!s.idBind.empty()) {
        q.Begin("DmeTransformList", s.idBind, "bind");
        q.RefArray("transforms", s.idJointXform);
        q.End();
    }

    // Joints go out in the space the .mdl stores them in, unrotated: the bind
    // pose the compiler ends up with is the $definebone the .pulseqc carries,
    // and vertices are already in that same model space. Pre-applying an
    // inverse g_defaultrotation here only tips the whole model 90 degrees.
    for (int i = 0; i < s.numbones; ++i) {
        std::vector<std::string> kids;
        for (int c = 0; c < s.numbones; ++c)
            if (bones[c].parent == i)
                kids.push_back(s.idJointDag[c]);
        q.Begin("DmeDag", s.idJointDag[i], boneNames[i]);
        q.Ref("transform", s.idJointXform[i]);
        q.RefArray("children", kids);
        q.End();

        // The euler `rot`, not the stored `quat`: the two are not always the
        // same rotation in a finished file, and $definebone is written from
        // `rot` - so taking `quat` here leaves the source skeleton and the
        // script disagreeing about the same bone.
        const pm::Vector3 pos = pose0 ? (*pose0)[i].pos : poses[i].pos;
        pm::Quaternion rot;
        pm::AngleQuaternion(pose0 ? (*pose0)[i].rot : poses[i].rot, rot);
        q.Begin("DmeTransform", s.idJointXform[i], boneNames[i]);
        q.Vec3("position", pos);
        q.Quat("orientation", rot);
        q.End();
    }
}

void WriteOne(const Mdl& m, const std::string& path, const std::string& meshName,
              const fm::mstudiomodel_t& model, const ModelLod& ml, const ModelTris& tris,
              const std::vector<fm::mstudiovertex_t>& vvd, const FlexRig& rig) {
    const fm::studiohdr_t& h = *m.hdr;
    const size_t vbase = static_cast<size_t>(ml.base);
    const size_t nverts = static_cast<size_t>(ml.count);
    if (nverts == 0)
        return; // this model contributes nothing at this LOD
    if (vbase + nverts > vvd.size()) {
        std::printf("  \"%s\": vertex range is outside the .vvd - skipped\n", meshName.c_str());
        return;
    }
    const fm::mstudiomesh_t* meshes =
        m.At<fm::mstudiomesh_t>(&model, model.meshindex, model.nummeshes);
    if (!meshes)
        return;

    // ---- vertex streams ----
    // Normals, UVs and the per-corner fields stay one entry per compiled vertex,
    // which is what keeps the recompile faithful. Positions are shared instead -
    // see posOf below.
    std::vector<pm::Vector3> positions(nverts), normals(nverts);
    std::vector<pm::Vector2> texcoords(nverts);
    std::vector<float> jointWeights, balance(nverts, 1.0f), speed(nverts, 1.0f);
    std::vector<int> jointIndices;
    int jointCount = 1;
    for (size_t i = 0; i < nverts; ++i)
        if (vvd[vbase + i].m_BoneWeights.numbones > jointCount)
            jointCount = vvd[vbase + i].m_BoneWeights.numbones;
    jointWeights.assign(nverts * static_cast<size_t>(jointCount), 0.0f);
    jointIndices.assign(nverts * static_cast<size_t>(jointCount), 0);
    for (size_t i = 0; i < nverts; ++i) {
        const fm::mstudiovertex_t& v = vvd[vbase + i];
        positions[i] = v.m_vecPosition;
        normals[i] = v.m_vecNormal;
        // V flipped, with flipVCoordinates set so the compiler flips it back -
        // what Valve's own exporters write, and what a modelling tool needs to
        // show the texture the right way up. 1-(1-v) can land a ULP off the
        // original for an awkward float, which is invisible either way.
        texcoords[i] = {v.m_vecTexCoord.x, 1.0f - v.m_vecTexCoord.y};
        for (int b = 0; b < v.m_BoneWeights.numbones && b < jointCount; ++b) {
            jointWeights[i * jointCount + b] = v.m_BoneWeights.weight[b];
            jointIndices[i * jointCount + b] = v.m_BoneWeights.bone[b];
        }
    }

    // ---- morph deltas ----
    // Gathered before the positions are shared: what a vertex's morphs do is
    // part of whether it may share at all.
    const std::vector<std::string> descs = FlexDescNames(m);
    const std::map<int, LidRole> lidDescs = LidRoles(m);
    const bool vtaStereo = VtaStereoOrder(m, descs);
    std::map<std::string, Delta> deltas;
    std::vector<std::string> deltaOrder;
    std::vector<std::string> deltaSig(nverts);
    bool anyBalance = false, anySpeed = false;
    std::set<std::string> lidTypes;      // which lids this model actually carries
    std::vector<char> authored(nverts, 0); // vertex carries a real stereo side
    std::vector<char> touched(nverts, 0);  // ... or a synthesized lid one
    for (int k = 0; k < model.nummeshes; ++k) {
        const fm::mstudioflex_t* flexes =
            m.At<fm::mstudioflex_t>(&meshes[k], meshes[k].flexindex, meshes[k].numflexes);
        for (int fi = 0; flexes && fi < meshes[k].numflexes; ++fi) {
            const fm::mstudioflex_t& fx = flexes[fi];
            const bool stereo = fx.flexpair > 0;
            const bool wrinkled = fx.vertanimtype == fm::STUDIO_VERT_ANIM_WRINKLE;
            const size_t stride = wrinkled ? sizeof(fm::mstudiovertanim_wrinkle_t)
                                           : sizeof(fm::mstudiovertanim_t);
            const char* va = m.At<char>(&fx, fx.vertindex, static_cast<int>(stride) * fx.numverts);
            if (!va || fx.flexdesc < 0 || static_cast<size_t>(fx.flexdesc) >= descs.size())
                continue;

            const std::string name = DeltaName(descs, lidDescs, fx);
            if (!deltas.count(name))
                deltaOrder.push_back(name);
            Delta& d = deltas[name];
            // a merged lid delta is what dmxeyelid splits L/R, so it is stereo
            // even though neither mono half was
            const auto lid = lidDescs.find(fx.flexdesc);
            const bool mergedLid = g_studiomdl && lid != lidDescs.end();
            d.stereo = d.stereo || stereo || mergedLid;
            if (mergedLid)
                lidTypes.insert(lid->second.type);
            const int ord = static_cast<int>(
                std::find(deltaOrder.begin(), deltaOrder.end(), name) - deltaOrder.begin());

            for (int n = 0; n < fx.numverts; ++n) {
                const auto& a = *reinterpret_cast<const fm::mstudiovertanim_t*>(va + stride * n);
                // vertanim indices are mesh-relative, and the mesh sits at a
                // different offset in each LOD's array
                const int mv = (static_cast<size_t>(k) < ml.meshOffset.size()
                                    ? ml.meshOffset[k]
                                    : meshes[k].vertexoffset) +
                               a.index;
                if (mv < 0 || static_cast<size_t>(mv) >= nverts)
                    continue;
                // everything the vertanim says except which vertex it is
                deltaSig[mv].append(reinterpret_cast<const char*>(&ord), sizeof ord);
                deltaSig[mv].append(va + stride * n + sizeof a.index, stride - sizeof a.index);
                d.posIdx.push_back(mv);
                d.pos.push_back({a.delta[0].GetFloat(), a.delta[1].GetFloat(),
                                 a.delta[2].GetFloat()});
                d.normIdx.push_back(mv);
                d.norm.push_back({a.ndelta[0].GetFloat(), a.ndelta[1].GetFloat(),
                                  a.ndelta[2].GetFloat()});
                if (wrinkled) {
                    const auto& w =
                        *reinterpret_cast<const fm::mstudiovertanim_wrinkle_t*>(va + stride * n);
                    d.wrinkleIdx.push_back(mv);
                    // the header field is only written when the scale isn't the
                    // 1/4096 default, so an unflagged model reads back as 0
                    const float scale =
                        (h.flags & fm::STUDIOHDR_FLAGS_VERT_ANIM_FIXED_POINT_SCALE)
                            ? h.flVertAnimFixedPointScale
                            : 1.0f / 4096.0f;
                    d.wrinkle.push_back(static_cast<float>(w.wrinkledelta) * scale);
                }
                // side/speed are the mesh's own per-vertex fields, stored on
                // every vertanim that touches the vertex rather than once. Only
                // a stereo flex carries a real side - a mono one writes a flat
                // 0, which would erase the balance on any vertex it shares.
                if (stereo && a.side != 255) {
                    balance[mv] = vtaStereo ? 1.0f - a.side / 255.0f : a.side / 255.0f;
                    authored[mv] = 1;
                    touched[mv] = 1;
                    anyBalance = true;
                } else if (stereo) {
                    // 255 IS the 1.0 default - keep the ramp off it
                    touched[mv] = 1;
                } else if (mergedLid && !authored[mv]) {
                    // The merged halves carry no side of their own, so it comes
                    // from the eye each came off - the model's right is 1, which
                    // is what the authored sides on these vertices already read.
                    // Never over an authored one: balance is a single per-vertex
                    // stream and every other stereo morph reads the same entry.
                    balance[mv] = lid->second.left ? 0.0f : 1.0f;
                    touched[mv] = 1;
                    anyBalance = true;
                }
                if (a.speed != 255) {
                    speed[mv] = a.speed / 255.0f;
                    anySpeed = true;
                }
            }
        }
    }

    // Balance is a left/right ramp across the WHOLE mesh, not just the vertices
    // some stereo morph happened to touch - a sparse map resamples to noise and
    // any morph added later splits wrong. Authored sides are kept as truth - X is
    // the left/right axis in the script space this mesh is written in, same as
    // the compiler's own GenerateBalance, and the samples say which way it runs.
    if (anyBalance) {
        float k = 0.0f, wide = 0.0f;
        for (size_t i = 0; i < nverts; ++i) {
            wide = std::max(wide, std::fabs(positions[i].x));
            if (touched[i])
                k += (balance[i] - 0.5f) * positions[i].x;
        }
        // dir +1 puts 1.0 at +X, which is what a positive correlation asks for
        const float dir = k > 0.0f ? 1.0f : -1.0f;
        // the crossover is a narrow seam at the midline, not a body-wide fade
        const float band = std::max(0.5f, wide * 0.05f);
        for (size_t i = 0; i < nverts; ++i) {
            if (touched[i])
                continue;
            float t = (band - dir * positions[i].x) / (2.0f * band);
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            balance[i] = 1.0f - (3.0f * t * t - 2.0f * t * t * t);
        }
    }

    // dmxeyelid dereferences all three slots, so a lid pose whose neutral was
    // just the base mesh still needs a delta state to name - an empty one is
    // exactly that pose.
    for (const std::string& t : lidTypes)
        for (int slot = 0; slot < 3; ++slot) {
            const std::string n = LidDeltaName(t, slot);
            if (deltas.count(n))
                continue;
            deltaOrder.push_back(n);
            deltas[n].stereo = true;
        }

    // Which slot each vertex's position lands in. The stream carries one entry
    // per distinct point, with the seam copies sharing it through
    // positionsIndices - a modelling tool builds its vertices from this stream,
    // so writing it pre-split would leave the model needing a merge-by-distance
    // on import, and that also fuses points that are close but genuinely
    // separate. Bit equality, never a tolerance.
    //
    // The morph signature is part of the key: a closed mouth has the upper and
    // lower lip on the SAME point and they have to be able to move apart, so two
    // vertices may only share a slot when their vertanims agree as well.
    //
    // So is the skinning. DMX has no jointWeightsIndices - weights are indexed by
    // position - so sharing a slot means sharing bones, and two points that
    // coincide with different weights (hair cards, finger seams) must not.
    std::map<std::string, int> seen;
    std::vector<int> posOf(nverts);
    int slots = 0;
    for (size_t i = 0; i < nverts; ++i) {
        std::string key(reinterpret_cast<const char*>(&positions[i]), sizeof(pm::Vector3));
        if (jointCount) {
            const size_t w = i * static_cast<size_t>(jointCount);
            key.append(reinterpret_cast<const char*>(&jointWeights[w]),
                       static_cast<size_t>(jointCount) * sizeof(float));
            key.append(reinterpret_cast<const char*>(&jointIndices[w]),
                       static_cast<size_t>(jointCount) * sizeof(int));
        }
        key += deltaSig[i];
        const auto ins = seen.emplace(key, slots);
        posOf[i] = ins.first->second;
        if (ins.second)
            ++slots;
    }

    // ---- faces: one face set per mesh, corners in the order they are listed ----
    // A face set's `faces` are corner indices into the *Indices streams, and the
    // loader reads a triangle back as (c0, c2, c1) - so the corners go out
    // flipped to land on the winding the .vtx holds.
    std::vector<int> corner; // corner -> model-relative vertex (all three streams)
    std::vector<std::vector<int>> faceSetFaces(static_cast<size_t>(model.nummeshes));
    static const MeshTris kNoTris;
    for (int k = 0; k < model.nummeshes; ++k) {
        const MeshTris& t = (static_cast<size_t>(k) < tris.size()) ? tris[k] : kNoTris;
        std::vector<int>& faces = faceSetFaces[k];
        for (size_t n = 0; n + 2 < t.size(); n += 3) {
            const int c = static_cast<int>(corner.size());
            corner.push_back(t[n]);
            corner.push_back(t[n + 2]);
            corner.push_back(t[n + 1]);
            faces.push_back(c);
            faces.push_back(c + 1);
            faces.push_back(c + 2);
            faces.push_back(-1);
        }
    }
    if (corner.empty()) {
        std::printf("  \"%s\": no LOD 0 triangles in the .vtx - skipped\n", meshName.c_str());
        return;
    }

    // A triangle is allowed to put two corners on the same point with different
    // UVs - zero area in 3D, a real face in UV space, and lash/eye cards are full
    // of them. Sharing the slot would make it degenerate, so the second corner's
    // vertex takes a private one. One pass is enough: a split only ever un-shares,
    // and the fresh slot belongs to nothing else.
    for (size_t n = 0; n + 2 < corner.size(); n += 3)
        for (int a = 0; a < 3; ++a)
            for (int b = a + 1; b < 3; ++b)
                if (corner[n + a] != corner[n + b] &&
                    posOf[corner[n + a]] == posOf[corner[n + b]])
                    posOf[corner[n + b]] = slots++;

    // slots -> the written stream, in first-use order. Skinning is indexed by
    // position (DMX has no jointWeightsIndices) and follows.
    std::vector<pm::Vector3> uniquePos;
    std::vector<float> uniqueWeights, uniqueBalance;
    std::vector<int> uniqueIndices;
    std::vector<int> slotTo(static_cast<size_t>(slots), -1);
    for (size_t i = 0; i < nverts; ++i) {
        int& to = slotTo[static_cast<size_t>(posOf[i])];
        if (to < 0) {
            to = static_cast<int>(uniquePos.size());
            uniquePos.push_back(positions[i]);
            uniqueBalance.push_back(balance[i]);
            const size_t w = i * static_cast<size_t>(jointCount);
            uniqueWeights.insert(uniqueWeights.end(), jointWeights.begin() + w,
                                 jointWeights.begin() + w + jointCount);
            uniqueIndices.insert(uniqueIndices.end(), jointIndices.begin() + w,
                                 jointIndices.begin() + w + jointCount);
        }
        posOf[i] = to;
    }

    std::vector<int> posCorner(corner.size());
    for (size_t i = 0; i < corner.size(); ++i)
        posCorner[i] = posOf[corner[i]];

    // A delta's positions index the shared stream, so the seam copies of one
    // point collapse to a single entry - the loader fans it back out over every
    // corner using that position. Lossless: a slot's vertices only got to share
    // it by having identical vertanims. Normals and wrinkle keep their split
    // indices (wrinkle is a texcoord data index, not a position one).
    for (const std::string& n : deltaOrder) {
        Delta& d = deltas[n];
        std::vector<int> idx;
        std::vector<pm::Vector3> pos;
        std::set<int> once;
        for (size_t i = 0; i < d.posIdx.size(); ++i) {
            const int p = posOf[d.posIdx[i]];
            if (!once.insert(p).second)
                continue;
            idx.push_back(p);
            pos.push_back(d.pos[i]);
        }
        d.posIdx.swap(idx);
        d.pos.swap(pos);
    }

    // ---- ids ----
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::printf("  cannot write \"%s\"\n", path.c_str());
        return;
    }
    Dmx q{f, Hash(meshName)};
    const std::vector<std::string> texNames = TextureNames(m);

    const Skel skel = AllocSkel(q, m);
    const std::string idMeshDag = q.NewId(), idMeshXform = q.NewId(), idMesh = q.NewId(),
                      idData = q.NewId();
    std::vector<std::string> idFaceSet, idMaterial;
    for (int k = 0; k < model.nummeshes; ++k) {
        idFaceSet.push_back(q.NewId());
        idMaterial.push_back(q.NewId());
    }
    // With a rebuilt rig the controls are the real ones, not one per delta.
    const size_t numControl = rig.empty() ? deltaOrder.size() : rig.controls.size();
    std::vector<std::string> idDelta, idControl, idDom;
    for (size_t i = 0; i < deltaOrder.size(); ++i)
        idDelta.push_back(q.NewId());
    for (size_t i = 0; i < numControl; ++i)
        idControl.push_back(q.NewId());
    for (size_t i = 0; i < rig.dominations.size(); ++i)
        idDom.push_back(q.NewId());
    const std::string idCombo = deltaOrder.empty() ? std::string() : q.NewId();

    WriteSkel(q, m, skel, meshName, idMeshDag, idCombo);

    // ---- the mesh ----
    q.Begin("DmeDag", idMeshDag, meshName);
    q.Ref("transform", idMeshXform);
    q.Ref("shape", idMesh);
    q.End();

    q.Begin("DmeTransform", idMeshXform, meshName);
    q.Vec3("position", {0.0f, 0.0f, 0.0f});
    q.Quat("orientation", {0.0f, 0.0f, 0.0f, 1.0f});
    q.End();

    q.Begin("DmeMesh", idMesh, meshName);
    q.Ref("currentState", idData);
    q.RefArray("baseStates", {idData});
    q.RefArray("faceSets", idFaceSet);
    q.RefArray("deltaStates", idDelta);
    // parallel to deltaStates - the weight each delta is dialed to, which is
    // nothing at rest. The combination operator reads the pair as a set, so the
    // array has to be there for the deltas to count as targets at all.
    if (!rig.empty())
        q.V2Array("deltaStateWeights", std::vector<pm::Vector2>(idDelta.size(), {0.0f, 0.0f}));
    q.End();

    std::vector<std::string> fmtNames = {F("positions"), F("normals"), F("textureCoordinates"),
                                         F("jointWeights"), F("jointIndices")};
    if (anyBalance)
        fmtNames.push_back(F("balance"));
    if (anySpeed)
        fmtNames.push_back(F("speed"));

    q.Begin("DmeVertexData", idData, "bind");
    q.StrArray("vertexFormat", fmtNames);
    q.Int("jointCount", jointCount);
    q.Bool("flipVCoordinates", true);
    q.V3Array(F("positions"), uniquePos);
    q.IntArray(F("positionsIndices"), posCorner);
    q.V3Array(F("normals"), normals);
    q.IntArray(F("normalsIndices"), corner);
    q.V2Array(F("textureCoordinates"), texcoords);
    q.IntArray(F("textureCoordinatesIndices"), corner);
    q.FloatArray(F("jointWeights"), uniqueWeights);
    q.IntArray(F("jointIndices"), uniqueIndices);
    if (anyBalance) {
        // balance belongs to the point, like the skinning - so it rides the
        // position stream's indices, not the per-corner ones
        q.FloatArray(F("balance"), uniqueBalance);
        q.IntArray(F("balanceIndices"), posCorner);
    }
    if (anySpeed) {
        q.FloatArray(F("speed"), speed);
        q.IntArray(F("speedIndices"), corner);
    }
    q.End();

    for (int k = 0; k < model.nummeshes; ++k) {
        // mtlName is the stored texture name verbatim, path and all: LookupTexture
        // keeps the directory (it only strips the extension), so a model whose
        // path lives in the material name rather than in $cdmaterials - which is
        // how an empty $cdmaterials happens - loses it if this is a basename.
        const std::string mat =
            (meshes[k].material >= 0 && static_cast<size_t>(meshes[k].material) < texNames.size())
                ? texNames[meshes[k].material]
                : std::string("default");
        q.Begin("DmeFaceSet", idFaceSet[k], BaseName(mat));
        q.Ref("material", idMaterial[k]);
        q.IntArray("faces", faceSetFaces[k]);
        q.End();

        q.Begin("DmeMaterial", idMaterial[k], BaseName(mat));
        q.Str("mtlName", mat);
        q.End();
    }

    for (size_t i = 0; i < deltaOrder.size(); ++i) {
        const Delta& d = deltas[deltaOrder[i]];
        std::vector<std::string> dfmt = {F("positions"), F("normals")};
        if (!d.wrinkle.empty())
            dfmt.push_back(F("wrinkle"));
        q.Begin("DmeVertexDeltaData", idDelta[i], deltaOrder[i]);
        q.StrArray("vertexFormat", dfmt);
        q.V3Array(F("positions"), d.pos);
        q.IntArray(F("positionsIndices"), d.posIdx);
        q.V3Array(F("normals"), d.norm);
        q.IntArray(F("normalsIndices"), d.normIdx);
        if (!d.wrinkle.empty()) {
            q.FloatArray(F("wrinkle"), d.wrinkle);
            q.IntArray(F("wrinkleIndices"), d.wrinkleIdx);
        }
        q.End();
    }

    // The rig rebuilt from the .mdl's flex rules: the input controls, and the
    // mesh as the target whose delta names spell the combinations. Without one
    // (no rules, or rules we could not decode) each delta gets its own control,
    // which carries the stereo flag and nothing else.
    if (!idCombo.empty()) {
        q.Begin("DmeCombinationOperator", idCombo, meshName);
        q.RefArray("controls", idControl);
        q.RefArray("targets", rig.empty() ? std::vector<std::string>{}
                                          : std::vector<std::string>{idMesh});
        if (!rig.dominations.empty())
            q.RefArray("dominators", idDom);
        q.End();

        if (rig.empty()) {
            for (size_t i = 0; i < deltaOrder.size(); ++i) {
                q.Begin("DmeCombinationInputControl", idControl[i], deltaOrder[i]);
                q.StrArray("rawControlNames", {deltaOrder[i]});
                q.Bool("stereo", deltas[deltaOrder[i]].stereo);
                q.Bool("eyelid", false);
                q.End();
            }
        } else {
            for (size_t i = 0; i < rig.controls.size(); ++i) {
                const RigControl& c = rig.controls[i];
                q.Begin("DmeCombinationInputControl", idControl[i], c.name);
                q.StrArray("rawControlNames", c.rawControls);
                q.Bool("stereo", c.stereo);
                q.Bool("eyelid", c.eyelid);
                if (!c.eyesUpDownFlex.empty())
                    q.Str("eyesUpDownFlex", c.eyesUpDownFlex);
                if (c.hasRange) {
                    q.Float("flexMin", c.min);
                    q.Float("flexMax", c.max);
                }
                q.End();
            }
            for (size_t i = 0; i < rig.dominations.size(); ++i) {
                q.Begin("DmeCombinationDominationRule", idDom[i], "dominationRule");
                q.StrArray("dominators", rig.dominations[i].dominators);
                q.StrArray("suppressed", rig.dominations[i].suppressed);
                q.End();
            }
        }
    }

    q.Save();
    std::fclose(f);
    std::printf("  wrote %s (%d verts, %d tris, %d deltas)\n", path.c_str(),
                static_cast<int>(nverts), static_cast<int>(corner.size() / 3),
                static_cast<int>(deltaOrder.size()));
}

// --- .phy -------------------------------------------------------------------

// One IVP compact ledge, back in source units and model space.
struct PhyHull {
    int bone = 0;
    std::vector<pm::Vector3> verts;
    std::vector<int> tris;
};

// IVP stores Y-down metres; source is Z-up inches. The swap is a rotation, so
// triangle winding survives it unchanged.
pm::Vector3 IvpToSrc(const float* k) {
    const float s = 1.0f / 0.0254f;
    return {k[0] * s, k[2] * s, -k[1] * s};
}

// libs/math's Vector3 is a bare POD; these are the only three ops needed here.
pm::Vector3 Sub(const pm::Vector3& a, const pm::Vector3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
pm::Vector3 Cross(const pm::Vector3& a, const pm::Vector3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float Dot(const pm::Vector3& a, const pm::Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Offsets inside a .phy are signed and relative to the struct holding them -
// the ledge tree sits after the ledges it points back at.
bool RelOff(size_t base, int32_t rel, size_t size, size_t& out) {
    const ptrdiff_t v = static_cast<ptrdiff_t>(base) + rel;
    if (v < 0 || static_cast<size_t>(v) >= size)
        return false;
    out = static_cast<size_t>(v);
    return true;
}

// bone -> model space, rebuilt from the same pos/rot the joints go out with, so
// a hull lands exactly where the reimported skeleton puts it back.
std::vector<pm::matrix3x4> BindPose(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* b =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    const std::vector<LocalPose> poses = LocalPoses(m);
    std::vector<pm::matrix3x4> out;
    for (int i = 0; b && i < h.numbones; ++i) {
        pm::matrix3x4 local;
        pm::AngleMatrix(poses[i].rot, poses[i].pos, local);
        out.push_back(b[i].parent >= 0 && b[i].parent < i
                          ? pm::ConcatTransforms(out[b[i].parent], local)
                          : local);
    }
    return out;
}

void CollectLedges(const std::vector<char>& buf, size_t node, std::vector<size_t>& out, int depth) {
    if (depth > 64 || out.size() > 4096)
        return;
    ivp_ledgetree_node_t n;
    if (node + sizeof(n) > buf.size())
        return;
    std::memcpy(&n, &buf[node], sizeof n);
    if (n.offset_right_node == 0) {
        size_t ledge = 0;
        if (n.offset_compact_ledge != 0 &&
            RelOff(node, n.offset_compact_ledge, buf.size(), ledge))
            out.push_back(ledge);
        return;
    }
    size_t right = 0;
    if (!RelOff(node, n.offset_right_node, buf.size(), right))
        return;
    CollectLedges(buf, node + sizeof(n), out, depth + 1); // left child is adjacent
    CollectLedges(buf, right, out, depth + 1);
}

bool ReadLedge(const std::vector<char>& buf, size_t off, PhyHull& out) {
    ivp_compact_ledge_t l;
    if (off + sizeof(l) > buf.size())
        return false;
    std::memcpy(&l, &buf[off], sizeof l);
    const int nTri = l.n_triangles;
    const int nPts = static_cast<int>(l.bitfield >> 8) - nTri - 1;
    size_t ptOff = 0;
    if (nTri <= 0 || nPts <= 0 || nPts > 0xffff ||
        !RelOff(off, l.c_point_offset, buf.size(), ptOff) ||
        ptOff + 16 * static_cast<size_t>(nPts) > buf.size() ||
        off + 16 + 16 * static_cast<size_t>(nTri) > buf.size())
        return false;

    out.verts.resize(nPts);
    for (int i = 0; i < nPts; ++i) {
        float k[3];
        std::memcpy(k, &buf[ptOff + 16 * static_cast<size_t>(i)], sizeof k);
        out.verts[i] = IvpToSrc(k);
    }

    // the ledge's three edges start at its three corners, in order
    for (int t = 0; t < nTri; ++t) {
        const size_t to = off + 16 + 16 * static_cast<size_t>(t);
        int v[3];
        for (int e = 0; e < 3; ++e) {
            uint32_t d = 0;
            std::memcpy(&d, &buf[to + 4 + 4 * static_cast<size_t>(e)], sizeof d);
            v[e] = static_cast<int>(d & 0xffffu);
        }
        if (v[0] >= nPts || v[1] >= nPts || v[2] >= nPts || v[0] == v[1] || v[1] == v[2] ||
            v[0] == v[2])
            continue;
        out.tris.insert(out.tris.end(), {v[0], v[1], v[2]});
    }
    if (out.tris.empty())
        return false;

    // A hull is convex, so "outward" is just "away from the centre" - which
    // makes the export display right whatever winding the .phy was built with.
    pm::Vector3 mid{0.0f, 0.0f, 0.0f};
    for (const pm::Vector3& p : out.verts) {
        mid.x += p.x;
        mid.y += p.y;
        mid.z += p.z;
    }
    const float inv = 1.0f / static_cast<float>(nPts);
    mid = {mid.x * inv, mid.y * inv, mid.z * inv};
    for (size_t i = 0; i + 2 < out.tris.size(); i += 3) {
        const pm::Vector3& a = out.verts[out.tris[i]];
        const pm::Vector3 n = Cross(Sub(out.verts[out.tris[i + 1]], a),
                                    Sub(out.verts[out.tris[i + 2]], a));
        if (Dot(n, Sub(a, mid)) < 0.0f)
            std::swap(out.tris[i + 1], out.tris[i + 2]);
    }

    // client_data is the 1-based bone the hull is rigged to; 0 is a single body,
    // whose hulls are already in model space.
    out.bone = l.client_data > 0 ? static_cast<int>(l.client_data) - 1 : -1;
    return true;
}

// Every hull in the .phy, model space. Solid order is blob order, which is also
// the order of the `solid` sections in the text tail.
bool ReadPhyHulls(const Mdl& m, const std::string& phyPath, std::vector<PhyHull>& out) {
    std::vector<char> buf;
    if (!ReadWhole(phyPath, buf) || buf.size() < sizeof(fm::phyheader_t))
        return false;
    const auto* h = reinterpret_cast<const fm::phyheader_t*>(buf.data());
    const std::vector<pm::matrix3x4> bindPose = BindPose(m);

    size_t p = sizeof(fm::phyheader_t);
    for (int i = 0; i < h->solidCount; ++i) {
        int32_t blob = 0;
        if (p + sizeof(blob) > buf.size())
            break;
        std::memcpy(&blob, &buf[p], sizeof blob);
        p += sizeof(blob);
        if (blob < static_cast<int32_t>(sizeof(ivp_compact_surface_t)) ||
            p + static_cast<size_t>(blob) > buf.size())
            break;
        const size_t next = p + static_cast<size_t>(blob);

        // a VPHY header may or may not sit in front of the compact surface
        size_t surf = p;
        phycollide_hdr_t ch;
        if (blob > static_cast<int32_t>(sizeof(ch))) {
            std::memcpy(&ch, &buf[p], sizeof ch);
            if (ch.vphysicsID == static_cast<int32_t>(MINICOL_VPHY_ID)) {
                if (ch.modelType != 0) {
                    std::printf("  .phy solid %d is not a compact surface - skipped\n", i);
                    p = next;
                    continue;
                }
                surf = p + sizeof(ch);
            }
        }

        ivp_compact_surface_t cs;
        if (surf + sizeof(cs) > buf.size()) {
            p = next;
            continue;
        }
        std::memcpy(&cs, &buf[surf], sizeof cs);
        size_t root = 0;
        if (!RelOff(surf, cs.offset_ledgetree_root, buf.size(), root)) {
            p = next;
            continue;
        }
        std::vector<size_t> ledges;
        CollectLedges(buf, root, ledges, 0);
        for (size_t off : ledges) {
            PhyHull hull;
            if (!ReadLedge(buf, off, hull))
                continue;
            if (hull.bone >= 0 && static_cast<size_t>(hull.bone) < bindPose.size()) {
                for (pm::Vector3& v : hull.verts)
                    v = pm::VectorTransform(v, bindPose[hull.bone]);
            } else {
                hull.bone = 0; // single body: already model space, rig it to the root
            }
            out.push_back(std::move(hull));
        }
        p = next;
    }
    return !out.empty();
}

} // namespace

const char* SetDmxOutput(const std::string& encoding, int formatModel) {
    if (encoding != "binary" && encoding != "keyvalues2")
        return "unknown -dmxencoding (binary or keyvalues2)";
    if (BinaryVersionFor(formatModel) == 0)
        return "unsupported -dmxmodel (1, 15, 18 or 22)";
    g_binary = encoding == "binary";
    g_formatModel = formatModel;
    return nullptr;
}

bool WriteAnimationDmx(const Mdl& m, const std::string& path, const std::string& clipName, int fps,
                       const std::vector<std::vector<AnimPose>>& frames) {
    if (frames.empty() || fps <= 0)
        return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;

    Dmx q{f};
    q.seed = Hash(clipName);
    const Skel s = AllocSkel(q, m);
    const std::string idList = q.NewId(), idClip = q.NewId(), idFrame = q.NewId();
    // three elements per channel: the channel, its log and the log's one layer
    std::vector<std::string> idChan(s.numbones * 2), idLog(s.numbones * 2),
        idLayer(s.numbones * 2);
    for (int i = 0; i < s.numbones * 2; ++i) {
        idChan[i] = q.NewId();
        idLog[i] = q.NewId();
        idLayer[i] = q.NewId();
    }

    // The frame times a DmeChannelsClip is sampled at: whole seconds plus the
    // rounded remainder, which is how the importer reconstructs them - a key
    // that lands anywhere else gets interpolated instead of read.
    std::vector<float> times;
    times.reserve(frames.size());
    for (int k = 0; k < static_cast<int>(frames.size()); ++k) {
        const int whole = k / fps;
        const int ticks = whole * 10000 +
                          Ticks(static_cast<float>(k - whole * fps) / static_cast<float>(fps));
        times.push_back(static_cast<float>(ticks) / 10000.0f);
    }

    WriteSkel(q, m, s, clipName, std::string(), std::string(), idList, &frames[0]);

    q.Begin("DmeAnimationList", idList, clipName);
    q.RefArray("animations", {idClip});
    q.End();

    q.Begin("DmeChannelsClip", idClip, clipName);
    q.Ref("timeFrame", idFrame);
    q.Int("frameRate", fps);
    q.RefArray("channels", idChan);
    q.End();

    q.Begin("DmeTimeFrame", idFrame, "timeFrame");
    q.Time("start", 0.0f);
    q.Time("duration", times.back());
    q.Time("offset", 0.0f);
    q.End();

    const std::vector<std::string> boneNames = BoneNames(m);
    for (int j = 0; j < s.numbones; ++j) {
        // A bone that never moves collapses to one key: FindKey holds the last
        // key past the end of the list, so one is the whole clip.
        std::vector<pm::Vector3> pos;
        std::vector<pm::Quaternion> rot;
        for (const std::vector<AnimPose>& fr : frames) {
            pm::Quaternion qr;
            pm::AngleQuaternion(fr[j].rot, qr);
            pos.push_back(fr[j].pos);
            rot.push_back(qr);
        }
        auto flat = [](const float* v, size_t n, size_t w) {
            for (size_t k = w; k < n * w; ++k)
                if (v[k] != v[k % w])
                    return false;
            return true;
        };
        if (flat(&pos[0].x, pos.size(), 3))
            pos.resize(1);
        if (flat(&rot[0].x, rot.size(), 4))
            rot.resize(1);

        for (int c = 0; c < 2; ++c) {
            const int n = j * 2 + c;
            const bool isPos = c == 0;
            q.Begin("DmeChannel", idChan[n], boneNames[j] + (isPos ? "_p" : "_o"));
            q.Ref("fromElement", std::string());
            q.Str("fromAttribute", "");
            q.Int("fromIndex", 0);
            q.Ref("toElement", s.idJointXform[j]);
            q.Str("toAttribute", isPos ? "position" : "orientation");
            q.Int("toIndex", 0);
            q.Int("mode", 3); // CM_PLAY
            q.Ref("log", idLog[n]);
            q.End();

            q.Begin(isPos ? "DmeVector3Log" : "DmeQuaternionLog", idLog[n], "log");
            q.RefArray("layers", {idLayer[n]});
            q.Bool("usedefaultvalue", false);
            q.End();

            q.Begin(isPos ? "DmeVector3LogLayer" : "DmeQuaternionLogLayer", idLayer[n], "log");
            q.TimeArray("times",
                        std::vector<float>(times.begin(),
                                           times.begin() + (isPos ? pos.size() : rot.size())));
            if (isPos)
                q.V3Array("values", pos);
            else
                q.QuatArray("values", rot);
            q.End();
        }
    }

    q.Save();
    std::fclose(f);
    return true;
}

PhysicsMeshInfo WritePhysicsMesh(const Mdl& m, const std::string& mdlPath,
                                 const std::string& dir, const std::string& name) {
    PhysicsMeshInfo info;
    std::vector<PhyHull> hulls;
    if (!ReadPhyHulls(m, StripExt(mdlPath) + ".phy", hulls))
        return info;

    // more than one hull on a bone only comes back as more than one hull if the
    // script says `concave`, which splits the mesh into islands again
    std::map<int, int> perBone;
    for (const PhyHull& hull : hulls)
        info.concave = info.concave || ++perBone[hull.bone] > 1;

    const std::string meshDir = dir + "/meshes";
    std::error_code ec;
    std::filesystem::create_directories(meshDir, ec);
    const std::string path = meshDir + "/" + name + ".dmx";
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::printf("  cannot write \"%s\"\n", path.c_str());
        return info;
    }

    // A hull's faces share its points, so it arrives as one solid rather than a
    // heap of loose triangles. The sharing stops at the hull boundary: touching
    // hulls have to stay disjoint or the island split welds them into one piece.
    std::vector<pm::Vector3> positions, normals;
    std::vector<pm::Vector2> texcoords;
    std::vector<float> jointWeights;
    std::vector<int> jointIndices, corner, faces;
    for (const PhyHull& hull : hulls) {
        const int base = static_cast<int>(positions.size());
        std::vector<pm::Vector3> vn(hull.verts.size(), pm::Vector3{});
        for (size_t i = 0; i + 2 < hull.tris.size(); i += 3) {
            const pm::Vector3& a = hull.verts[hull.tris[i]];
            const pm::Vector3 n = Cross(Sub(hull.verts[hull.tris[i + 1]], a),
                                        Sub(hull.verts[hull.tris[i + 2]], a));
            for (int e = 0; e < 3; ++e) {
                pm::Vector3& d = vn[hull.tris[i + e]];
                d.x += n.x;
                d.y += n.y;
                d.z += n.z;
            }
        }
        for (size_t i = 0; i < hull.verts.size(); ++i) {
            pm::Vector3 n = vn[i];
            if (pm::VectorNormalize(n) <= 0.0f)
                n = {0.0f, 0.0f, 1.0f};
            positions.push_back(hull.verts[i]);
            normals.push_back(n);
            texcoords.push_back({0.0f, 0.0f});
            jointWeights.push_back(1.0f);
            jointIndices.push_back(hull.bone);
        }
        // ReadLedge already put the corners in outward order, so they go out as
        // they are - a collision source is hulled from its points alone and the
        // compiler never looks at the winding.
        for (size_t i = 0; i + 2 < hull.tris.size(); i += 3) {
            const int c = static_cast<int>(corner.size());
            for (int e = 0; e < 3; ++e)
                corner.push_back(base + hull.tris[i + e]);
            faces.insert(faces.end(), {c, c + 1, c + 2, -1});
        }
    }

    Dmx q{f, Hash(name)};
    const Skel skel = AllocSkel(q, m);
    const std::string idMeshDag = q.NewId(), idMeshXform = q.NewId(), idMesh = q.NewId(),
                      idData = q.NewId(), idFaceSet = q.NewId(), idMaterial = q.NewId();

    WriteSkel(q, m, skel, name, idMeshDag, std::string());

    q.Begin("DmeDag", idMeshDag, name);
    q.Ref("transform", idMeshXform);
    q.Ref("shape", idMesh);
    q.End();

    q.Begin("DmeTransform", idMeshXform, name);
    q.Vec3("position", {0.0f, 0.0f, 0.0f});
    q.Quat("orientation", {0.0f, 0.0f, 0.0f, 1.0f});
    q.End();

    q.Begin("DmeMesh", idMesh, name);
    q.Ref("currentState", idData);
    q.RefArray("baseStates", {idData});
    q.RefArray("faceSets", {idFaceSet});
    q.RefArray("deltaStates", {});
    q.End();

    q.Begin("DmeVertexData", idData, "bind");
    q.StrArray("vertexFormat", {F("positions"), F("normals"), F("textureCoordinates"),
                                F("jointWeights"), F("jointIndices")});
    q.Int("jointCount", 1);
    q.Bool("flipVCoordinates", true);
    q.V3Array(F("positions"), positions);
    q.IntArray(F("positionsIndices"), corner);
    q.V3Array(F("normals"), normals);
    q.IntArray(F("normalsIndices"), corner);
    q.V2Array(F("textureCoordinates"), texcoords);
    q.IntArray(F("textureCoordinatesIndices"), corner);
    q.FloatArray(F("jointWeights"), jointWeights);
    q.IntArray(F("jointIndices"), jointIndices);
    q.End();

    // A collision source's materials never reach the model, so the name is only
    // there because a face set has to have one.
    q.Begin("DmeFaceSet", idFaceSet, "phys");
    q.Ref("material", idMaterial);
    q.IntArray("faces", faces);
    q.End();

    q.Begin("DmeMaterial", idMaterial, "phys");
    q.Str("mtlName", "phys");
    q.End();

    q.Save();
    std::fclose(f);
    std::printf("  wrote %s (%d hulls, %d tris)\n", path.c_str(), static_cast<int>(hulls.size()),
                static_cast<int>(corner.size() / 3));
    info.written = true;
    return info;
}

std::vector<LodInfo> WriteRenderMeshes(const Mdl& m, const std::string& mdlPath,
                                       const std::string& dir,
                                       const std::vector<std::vector<std::string>>& names,
                                       const FlexRig& rig) {
    std::vector<LodInfo> lods;
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    if (!parts)
        return lods;

    const std::string stem = StripExt(mdlPath);
    std::vector<char> vvdBuf;
    if (!ReadWhole(stem + ".vvd", vvdBuf)) {
        std::printf("no usable \"%s.vvd\" - meshes not extracted\n", stem.c_str());
        return lods;
    }
    std::vector<char> vtxBuf;
    // .dx90.vtx is what ships; the others are the same data for dead renderers
    if (!ReadWhole(stem + ".dx90.vtx", vtxBuf) && !ReadWhole(stem + ".vtx", vtxBuf) &&
        !ReadWhole(stem + ".dx80.vtx", vtxBuf)) {
        std::printf("no usable \"%s.dx90.vtx\" - meshes not extracted\n", stem.c_str());
        return lods;
    }
    if (vtxBuf.size() < sizeof(vtx::FileHeader_t))
        return lods;
    const auto* fh = reinterpret_cast<const vtx::FileHeader_t*>(vtxBuf.data());
    const int numLods = fh->numLODs > 0 ? fh->numLODs : 1;

    // the strip-group stride is probed once and reused for every LOD
    size_t stride = sizeof(vtx::LegacyStripGroupHeader_t);
    PartTris tris;
    const LodLayout probe = BuildLodLayout(m, 0);
    if (!ExtractTris(vtxBuf, m, stride, 0, probe, tris)) {
        stride = sizeof(vtx::StripGroupHeader_t);
        const LodLayout probe = BuildLodLayout(m, 0);
    if (!ExtractTris(vtxBuf, m, stride, 0, probe, tris)) {
            std::printf("the .vtx does not parse as either strip-group layout - meshes not "
                        "extracted\n");
            return lods;
        }
    }

    // meshes/ next to the script, the way animations go in anims/
    const std::string meshDir = dir + "/meshes";
    std::error_code ec;
    std::filesystem::create_directories(meshDir, ec);

    static const ModelTris empty;
    std::vector<fm::mstudiovertex_t> vvd;
    for (int lod = 0; lod < numLods && lod < fm::kMaxNumLods; ++lod) {
        const LodLayout layout = BuildLodLayout(m, lod);
        if (!ExtractTris(vtxBuf, m, stride, lod, layout, tris))
            break;
        if (!BuildLodVerts(vvdBuf, lod, vvd))
            break;

        lods.push_back(LodInfo{});
        LodInfo& info = lods.back();
        info.switchPoint = LodSwitchPoint(vtxBuf, m, lod);
        info.materialReplacements = LodMaterialReplacements(vtxBuf, m, lod);

        // LOD 0 keeps the plain alias; the rest hang a _lod<n> off it, which is
        // what the $lod block's replacemodel points at
        const std::string suffix = lod ? "_lod" + std::to_string(lod) : std::string();
        for (int i = 0; i < h.numbodyparts && static_cast<size_t>(i) < names.size(); ++i) {
            const fm::mstudiomodel_t* models =
                m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
            for (int j = 0; models && j < parts[i].nummodels; ++j) {
                if (static_cast<size_t>(j) >= names[i].size() || names[i][j].empty())
                    continue; // blank body
                const ModelTris& mt = (static_cast<size_t>(i) < tris.size() &&
                                       static_cast<size_t>(j) < tris[i].size())
                                          ? tris[i][j]
                                          : empty;
                WriteOne(m, meshDir + "/" + names[i][j] + suffix + ".dmx", names[i][j] + suffix,
                         models[j], layout[i][j], mt, vvd, rig);
            }
        }
    }
    return lods;
}

} // namespace mdldecompiler
