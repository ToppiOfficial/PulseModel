// vtx.h - PulseMDL
//
// On-disk .vtx (OptimizedModel, file version 7) structures, re-declared from
// format facts. The whole format is #pragma pack(1) - no padding anywhere;
// all offsets are relative to the struct that holds them.
//
// Two strip/strip-group layouts exist for the SAME version number 7:
// - legacy (TF2 / L4D2, "vtx_archetype" 0, the default): StripGroup 25 bytes,
//   Strip 27 bytes - no topology fields.
// - full (Alien Swarm / CS:GO / SFM, "vtx_archetype" 1): StripGroup 33 bytes,
//   Strip 35 bytes - trailing numTopologyIndices/topologyOffset pairs.
//
// File block order (as the original writer emits it): FileHeader_t, body part
// headers, model headers, model LOD headers, mesh headers, strip group
// headers, strip headers, Vertex_t array, uint16 index array, bone state
// changes, string table, material replacement headers, material replacement
// list headers, [topology indices - full layout only].

#ifndef PULSEMDL_FORMAT_VTX_H
#define PULSEMDL_FORMAT_VTX_H

#include <cstdint>

namespace pulse::format::vtx {

inline constexpr int32_t kOptimizedModelFileVersion = 7;
inline constexpr int kMaxBonesPerStrip = 512;

// StripHeaderFlags_t
inline constexpr uint8_t STRIP_IS_TRILIST = 0x01;
inline constexpr uint8_t STRIP_IS_QUADLIST_REG = 0x02;
inline constexpr uint8_t STRIP_IS_QUADLIST_EXTRA = 0x04;

// StripGroupFlags_t
inline constexpr uint8_t STRIPGROUP_IS_FLEXED = 0x01;
inline constexpr uint8_t STRIPGROUP_IS_HWSKINNED = 0x02;
inline constexpr uint8_t STRIPGROUP_IS_DELTA_FLEXED = 0x04;

// MeshFlags_t
inline constexpr uint8_t MESH_IS_TEETH = 0x01;
inline constexpr uint8_t MESH_IS_EYES = 0x02;

#pragma pack(push, 1)

struct BoneStateChangeHeader_t {
    int32_t hardwareID;
    int32_t newBoneID;
};
static_assert(sizeof(BoneStateChangeHeader_t) == 8, "BoneStateChangeHeader_t layout");

struct Vertex_t {
    uint8_t boneWeightIndex[3]; // into the vvd vert's bone slots
    uint8_t numBones;
    uint16_t origMeshVertID; // mesh-relative vvd vertex id
    uint8_t boneID[3];       // hw palette ids for hw-skinned strip groups
};
static_assert(sizeof(Vertex_t) == 9, "Vertex_t layout");

// full (Alien Swarm+) strip / strip group
struct StripHeader_t {
    int32_t numIndices;
    int32_t indexOffset;
    int32_t numVerts;
    int32_t vertOffset;
    int16_t numBones;
    uint8_t flags;
    int32_t numBoneStateChanges;
    int32_t boneStateChangeOffset;
    int32_t numTopologyIndices;
    int32_t topologyOffset;
};
static_assert(sizeof(StripHeader_t) == 35, "StripHeader_t layout");

struct StripGroupHeader_t {
    int32_t numVerts;
    int32_t vertOffset;
    int32_t numIndices;
    int32_t indexOffset;
    int32_t numStrips;
    int32_t stripOffset;
    uint8_t flags;
    int32_t numTopologyIndices;
    int32_t topologyOffset;
};
static_assert(sizeof(StripGroupHeader_t) == 33, "StripGroupHeader_t layout");

// legacy (TF2/L4D2) strip / strip group - same fields minus topology
struct LegacyStripHeader_t {
    int32_t numIndices;
    int32_t indexOffset;
    int32_t numVerts;
    int32_t vertOffset;
    int16_t numBones;
    uint8_t flags;
    int32_t numBoneStateChanges;
    int32_t boneStateChangeOffset;
};
static_assert(sizeof(LegacyStripHeader_t) == 27, "LegacyStripHeader_t layout");

struct LegacyStripGroupHeader_t {
    int32_t numVerts;
    int32_t vertOffset;
    int32_t numIndices;
    int32_t indexOffset;
    int32_t numStrips;
    int32_t stripOffset;
    uint8_t flags;
};
static_assert(sizeof(LegacyStripGroupHeader_t) == 25, "LegacyStripGroupHeader_t layout");

struct MeshHeader_t {
    int32_t numStripGroups;
    int32_t stripGroupHeaderOffset;
    uint8_t flags;
};
static_assert(sizeof(MeshHeader_t) == 9, "MeshHeader_t layout");

struct ModelLODHeader_t {
    int32_t numMeshes;
    int32_t meshOffset;
    float switchPoint;
};
static_assert(sizeof(ModelLODHeader_t) == 12, "ModelLODHeader_t layout");

struct ModelHeader_t {
    int32_t numLODs;
    int32_t lodOffset;
};
static_assert(sizeof(ModelHeader_t) == 8, "ModelHeader_t layout");

struct BodyPartHeader_t {
    int32_t numModels;
    int32_t modelOffset;
};
static_assert(sizeof(BodyPartHeader_t) == 8, "BodyPartHeader_t layout");

struct MaterialReplacementHeader_t {
    int16_t materialID;
    int32_t replacementMaterialNameOffset;
};
static_assert(sizeof(MaterialReplacementHeader_t) == 6, "MaterialReplacementHeader_t layout");

struct MaterialReplacementListHeader_t {
    int32_t numReplacements;
    int32_t replacementOffset;
};
static_assert(sizeof(MaterialReplacementListHeader_t) == 8, "MaterialReplacementListHeader_t layout");

struct FileHeader_t {
    int32_t version; // 7
    // hardware params the model was optimized for
    int32_t vertCacheSize;
    uint16_t maxBonesPerStrip;
    uint16_t maxBonesPerFace;
    int32_t maxBonesPerVert;
    int32_t checkSum; // must match studiohdr_t
    int32_t numLODs;
    int32_t materialReplacementListOffset; // one list header per LOD
    int32_t numBodyParts;
    int32_t bodyPartOffset;
};
static_assert(sizeof(FileHeader_t) == 36, "FileHeader_t layout");

#pragma pack(pop)

} // namespace pulse::format::vtx

#endif // PULSEMDL_FORMAT_VTX_H
