#pragma once
// IVP compact surface binary format structures.
// These define the binary layout of the per-solid blob written by CollideWrite
// and read at runtime by vphysics.dll (TF2/CS:GO Source Engine).
//
// Format reference: VPhysicsBullet (Triang3l), Valve Developer Community PHY page,
// and IVP/Havok compact surface reverse engineering by the Source modding community.

#include <stdint.h>

// -----------------------------------------------------------------------
// Top-level header - first thing in every CollideWrite blob
// -----------------------------------------------------------------------
//   int32  vphysicsID     = MINICOL_VPHY_ID (little-endian 'VPHY')
//   int16  version        = MINICOL_VERSION_IVP
//   int16  modelType      = 0  (IVP compact surface, convex-only)
//   int32  surfaceSize    = bytes after this header
//   float  dragAxisAreas[3]
//   int32  axisMapSize    = 0
struct phycollide_hdr_t {
    int32_t vphysicsID;
    int16_t version;
    int16_t modelType;
    int32_t surfaceSize;
    float   dragAxisAreas[3];
    int32_t axisMapSize;
};
static_assert(sizeof(phycollide_hdr_t) == 28, "phycollide_hdr_t size");

// -----------------------------------------------------------------------
// IVP Compact Surface - root of the IVP binary blob
// -----------------------------------------------------------------------
//   float  mass_center[3]
//   float  rotation_inertia[3]      diagonal of inertia tensor
//   float  upper_limit_radius       bounding sphere radius
//   uint32 bitfield:                 { max_factor_surface_deviation:8, byte_size:24 }
//   int32  offset_ledgetree_root    byte offset from THIS struct to first tree node
//   int32  dummy[3]                 dummy[2] = MINICOL_IVPS_ID
struct ivp_compact_surface_t {
    float   mass_center[3];
    float   rotation_inertia[3];
    float   upper_limit_radius;
    uint32_t bitfield;
    int32_t  offset_ledgetree_root;
    int32_t  dummy[3];
};
static_assert(sizeof(ivp_compact_surface_t) == 48, "ivp_compact_surface_t size");

// byte_size lives in bits [8..31] (signed 24-bit)
static inline void ivp_surface_set_byte_size(ivp_compact_surface_t& s, int sz) {
    s.bitfield = (s.bitfield & 0xFFu) | ((uint32_t)(sz & 0xFFFFFF) << 8);
}

// -----------------------------------------------------------------------
// IVP Compact Ledge Tree Node - 28 bytes
// -----------------------------------------------------------------------
//   int32  offset_right_node     byte offset from THIS node to right child; 0 = leaf
//   int32  offset_compact_ledge  byte offset from THIS node to ledge; 0 = internal node
//   float  center[3]             bounding sphere center
//   float  radius                bounding sphere radius
//   uint8  box_sizes[3]          quantized AABB half-extents (optional, can be 0)
//   uint8  free_0                padding
struct ivp_ledgetree_node_t {
    int32_t offset_right_node;
    int32_t offset_compact_ledge;
    float   center[3];
    float   radius;
    uint8_t box_sizes[3];
    uint8_t free_0;
};
static_assert(sizeof(ivp_ledgetree_node_t) == 28, "ivp_ledgetree_node_t size");

// -----------------------------------------------------------------------
// IVP Compact Ledge - 16 bytes header, followed by triangles then points
// -----------------------------------------------------------------------
//   int32  c_point_offset   byte offset from &ledge to first point
//   int32  client_data      game data (bone index), also used as ledgetree_node_offset
//   uint32 bitfield:         { has_children_flag:2, is_compact_flag:2, dummy:4, size_div_16:24 }
//   int16  n_triangles
//   int16  for_future_use   = 0
//
//   n_points = size_div_16 - n_triangles - 1
//   size_div_16 = 1 + n_triangles + n_points
//
//   Immediately after header:
//     ivp_compact_triangle_t [n_triangles]   (16 bytes each)
//     ivp_compact_poly_point_t [n_points]    (16 bytes each)
struct ivp_compact_ledge_t {
    int32_t  c_point_offset;
    int32_t  client_data;
    uint32_t bitfield;
    int16_t  n_triangles;
    int16_t  for_future_use;
};
static_assert(sizeof(ivp_compact_ledge_t) == 16, "ivp_compact_ledge_t size");

// has_children_flag = 0 (terminal), is_compact_flag = IVP_TRUE (1);
// size_div_16 in bits [8..31]
static inline void ivp_ledge_init(ivp_compact_ledge_t& l, int nTri, int nPts) {
    int sdv = 1 + nTri + nPts;
    l.bitfield = (1u << 2) | ((uint32_t)(sdv & 0xFFFFFF) << 8);
    l.c_point_offset  = (int32_t)(sizeof(ivp_compact_ledge_t) + nTri * 16);
    l.n_triangles     = (int16_t)nTri;
    l.for_future_use  = 0;
}

// -----------------------------------------------------------------------
// IVP Compact Triangle - 16 bytes
// -----------------------------------------------------------------------
//   uint32 bitfield: { tri_index:12, pierce_index:12, material_index:7, is_virtual:1 }
//   ivp_compact_edge_t c_three_edges[3]
//
// Each edge: { start_point_index:16, opposite_index:15(signed), is_virtual:1 }
// opposite_index is relative pointer arithmetic in 4-byte IVP_Compact_Edge
// units (IVP_Compact_Edge::get_opposite() returns this + opposite_index).
// A triangle is 16 bytes = 4 edge slots (bitfield dword + 3 edges), so:
//   opposite_index = (opp_tri*4 + opp_edge) - (this_tri*4 + this_edge)
struct ivp_compact_edge_t {
    uint32_t data; // see accessor below
};

static inline void ivp_edge_set(ivp_compact_edge_t& e, uint16_t startPt, int16_t opp) {
    // bits [0..15]: start_point_index
    // bits [16..30]: opposite_index (signed 15-bit)
    // bit  [31]:     is_virtual = 0
    e.data = (uint32_t)startPt | (((uint32_t)(opp & 0x7FFF)) << 16);
}

struct ivp_compact_triangle_t {
    uint32_t         bitfield;
    ivp_compact_edge_t c_three_edges[3];
};
static_assert(sizeof(ivp_compact_triangle_t) == 16, "ivp_compact_triangle_t size");

static inline void ivp_tri_set(ivp_compact_triangle_t& t, int triIdx, int pierceIdx, int matIdx) {
    // tri_index:12, pierce_index:12, material_index:7, is_virtual:1
    t.bitfield = ((uint32_t)(triIdx & 0xFFF))
               | (((uint32_t)(pierceIdx & 0xFFF)) << 12)
               | (((uint32_t)(matIdx & 0x7F))  << 24);
}

// -----------------------------------------------------------------------
// IVP Compact Poly Point - 16 bytes
// -----------------------------------------------------------------------
//   float k[3]       vertex position
//   float hesse_val  = 0
struct ivp_compact_poly_point_t {
    float k[3];
    float hesse_val;
};
static_assert(sizeof(ivp_compact_poly_point_t) == 16, "ivp_compact_poly_point_t size");

// -----------------------------------------------------------------------
// Magic constants
// -----------------------------------------------------------------------
#define MINICOL_VPHY_ID      0x59485056u  // 'VPHY' LE
#define MINICOL_IVPS_ID      0x53505649u  // 'IVPS' LE
#define MINICOL_VERSION_IVP  0x0100
