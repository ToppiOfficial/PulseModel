// vvd.h - PulseMDL
//
// On-disk .vvd (vertex data file, version 4) structures, re-declared from
// format facts. Sizes locked by static_asserts.
//
// File layout (as the original writer emits it):
//   vertexFileHeader_t
//   [fixup table]            (ALIGN4; absent when numFixups == 0)
//   vertex block             (ALIGN16; numLODVertexes[0] x mstudiovertex_t)
//   tangent block            (ALIGN16 after fixup pass; Vector4 per vertex)

#ifndef PULSEMDL_FORMAT_VVD_H
#define PULSEMDL_FORMAT_VVD_H

#include <cstdint>

#include "math/math.h"

namespace pulse::format {

// "IDSV" little-endian.
inline constexpr int32_t kIdVertexFile = ('V' << 24) + ('S' << 16) + ('D' << 8) + 'I';
inline constexpr int32_t kVertexFileVersion = 4;

inline constexpr int kMaxNumLods = 8;
inline constexpr int kMaxNumBonesPerVert = 3;

struct mstudioboneweight_t {
    float weight[kMaxNumBonesPerVert];
    uint8_t bone[kMaxNumBonesPerVert];
    uint8_t numbones;
};
static_assert(sizeof(mstudioboneweight_t) == 16, "mstudioboneweight_t layout");

struct mstudiovertex_t {
    mstudioboneweight_t m_BoneWeights;
    math::Vector3 m_vecPosition;
    math::Vector3 m_vecNormal;
    math::Vector2 m_vecTexCoord;
};
static_assert(sizeof(mstudiovertex_t) == 48, "mstudiovertex_t layout");

struct vertexFileHeader_t {
    int32_t id;       // kIdVertexFile
    int32_t version;  // 4
    int32_t checksum; // must match studiohdr_t
    int32_t numLODs;
    int32_t numLODVertexes[kMaxNumLods];
    int32_t numFixups;
    int32_t fixupTableStart;
    int32_t vertexDataStart;
    int32_t tangentDataStart;
};
static_assert(sizeof(vertexFileHeader_t) == 64, "vertexFileHeader_t layout");

struct vertexFileFixup_t {
    int32_t lod;
    int32_t sourceVertexID;
    int32_t numVertexes;
};
static_assert(sizeof(vertexFileFixup_t) == 12, "vertexFileFixup_t layout");

} // namespace pulse::format

#endif // PULSEMDL_FORMAT_VVD_H
