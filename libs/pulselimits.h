// pulselimits.h - PulseMDL
//
// Single source of truth for every hard compile limit. No other file may
// define its own limit constant - add it here instead.
// Named pulselimits.h (not limits.h) to avoid shadowing <limits.h>.
//
// Each limit is tagged with where its number comes from:
//   [format]  a field width on disk proves it - immovable.
//   [file]    nothing narrower than int32 counts it, so the only ceiling is
//             the 2 GB a signed 32-bit byte offset can address.
//   [tool]    ours, because a fixed array or a per-vertex cost depends on it.
//
// These are deliberately NOT copies of an engine's runtime caps. Valve's
// studio.h numbers differ per branch and forks raise them, so the compiler
// rejects only what the FORMAT cannot express and leaves the target engine to
// be the thing that says no.

#ifndef PULSEMDL_LIMITS_H
#define PULSEMDL_LIMITS_H

#include <cstdint>

namespace pulse::limits {

// --- File format version --------------------------------------------------
inline constexpr int kStudioVersion = 49; // .mdl studiohdr version

// [file] Largest count that cannot overflow an int32 byte offset when
// multiplied by any struct the format defines (all are <= 128 bytes on disk).
// Used wherever the format stores a count as int32 and nothing narrower
// constrains it - i.e. "the compiler will not be what rejects this".
inline constexpr int kUncapped = 1 << 23; // 8388608

// --- Geometry -------------------------------------------------------------
// [file] The .vvd holds 48 bytes of vertex + 16 of tangent, and its offsets
// must address the whole payload.
inline constexpr int kMaxVerts = INT32_MAX / 64; // 33554431
// [file] Three uint16 indices per triangle in the .vtx index array.
inline constexpr int kMaxTriangles = INT32_MAX / 6; // 357913941
inline constexpr int kMaxFlexVerts = 65536; // [format] mstudiovertanim_t::index uint16
inline constexpr int kMaxMeshes    = kUncapped; // [file] mstudiomesh_t is 116 bytes
// [format] vtx::Vertex_t::origMeshVertID / mstudiovertanim_t::index are uint16
// mesh-relative. Overflow splits across meshes (BuildOutputMeshes).
inline constexpr int kMaxMeshVerts = 65536;
// [format] Strip group indices are uint16; strips don't share verts
// (PostProcessStripGroup copies each), so a group can exceed its source mesh.
// Overflow splits, not rejects.
inline constexpr int kMaxStripGroupVerts = 65536;
inline constexpr int kMaxTexCoords       = 8;

inline constexpr int kMaxBonesPerVert  = 3; // [format] mstudioboneweight_t arrays are [3]
inline constexpr int kMaxBoneWeights   = 3; // [format] same
// [tool] Pre-collapse per-vertex cap, clipped to kMaxBoneWeights after bone
// collapse. Sizes a fixed array on every SrcVertex - raising costs memory per
// vertex, so it stays a budget. Reference MAXSTUDIOSRCBONEWEIGHTS.
inline constexpr int   kMaxSrcBoneWeights = 16;
inline constexpr float kMinBoneWeight     = 0.0001f;
// [tool] Above the format: vtx::Vertex_t::boneID is uint8, so a vertex can
// only name palette slot 0-255. Kept at the reference's 512 rather than
// lowered - hardware palettes never approach either number.
inline constexpr int kMaxBonesPerStrip = 512;
inline constexpr int kMaxNumLods       = 8; // [format] numLODVertexes[8]

// --- Skeleton -------------------------------------------------------------
// [tool] Total bones. Already past what most of the format can address:
// mstudioboneweight_t::bone and mstudio_rle_anim_t::bone are both uint8, so
// only bones 0-255 can be skinned to or RLE-animated. Bones beyond that are
// reachable through frame animation (mstudio_frame_anim_t indexes positionally)
// and as attachment/hitbox parents. kMaxBoneBits must track this.
inline constexpr int kMaxBones     = 1024;
inline constexpr int kMaxBoneBits  = 10; // must match kMaxBones (2^10)
inline constexpr int kMaxSrcBones  = kUncapped; // [file] pre-collapse, never written as-is
inline constexpr int kMaxBoneCtrls = 4; // [tool] mstudiobone_t has 6 slots, every engine networks 4
inline constexpr int kMaxAttachments = kUncapped; // [file] 92 bytes each
inline constexpr int kMaxJiggleBones = kMaxBones; // one per bone
inline constexpr int kMaxHitboxSets     = kUncapped; // [file] 12 bytes each
inline constexpr int kMaxHitboxesPerSet = kUncapped; // [file] 68 bytes each
inline constexpr int kMaxProceduralBones = kMaxBones; // one per bone
// [tool] s_quatinterpbone_t trigger arrays are [32] and the .mdl reader walks
// exactly numtriggers entries - authored ceiling, not a soft budget.
inline constexpr int kMaxProceduralTriggers = 32;
inline constexpr int kMaxBoneTransformEdits = kUncapped; // [file] 100 bytes each

// --- Textures / materials -------------------------------------------------
// [format] vtx::MaterialReplacementHeader_t::materialID and the .mdl skin ref
// table are int16. NOTE: sizes the per-material arrays on Source and Model
// (~655 KB per Source at this value).
// ponytail: flat kMaxSkins-sized arrays, size them to the live material count
// if the memory ever shows up in a profile.
inline constexpr int kMaxSkins = INT16_MAX; // 32767
inline constexpr int kMaxSkinFamilies = kUncapped; // [file] one short per skin per family

// --- Flex / morph ---------------------------------------------------------
inline constexpr int kMaxFlexDesc = kUncapped; // [file] 4 bytes each
inline constexpr int kMaxFlexCtrl = kUncapped; // [file] 20 bytes each
inline constexpr int kMaxFlexRules = kUncapped; // [file] 12 bytes each
inline constexpr int kMaxFlexKeys = kMaxFlexDesc / 2; // reference: always half of FlexDesc
inline constexpr int kMaxFlexOps = kUncapped; // [file] 8 bytes each
inline constexpr int kBudgetFlexMorphVerts = 32768; // [tool] checked before kMaxFlexVerts
inline constexpr int kMaxEyeballs = 4; // [tool] engine-side eyeball arrays are fixed

// --- Animation / sequences ------------------------------------------------
// [format] Frames per animation. NOT int32-free: mstudio_rle_anim_t::nextoffset
// and mstudioanim_valueptr_t::offset are int16, so one bone's block in one
// section must stay under 32767 bytes - roughly 2700 frames of fully noisy
// motion across 6 channels. Sectioning ($sectionframes) is what buys headroom
// past that, so this stays a budget rather than rising to the int32 count.
inline constexpr int kMaxAnimFrames = 5000;
inline constexpr int kMaxAnims      = INT16_MAX; // [format] sequence blend anim indices are int16
inline constexpr int kMaxSequences  = INT16_MAX; // [format] mstudioautolayer_t::iSequence int16
inline constexpr int kMaxAnimBlocks = kUncapped; // [file] 8 bytes each
inline constexpr int kMaxEvents     = kUncapped; // [file] 76 bytes each
inline constexpr int kMaxMoveKeys   = kUncapped; // [file]
inline constexpr int kMaxActivityModifiers = kUncapped; // [file] 8 bytes each
inline constexpr int kMaxTags       = kUncapped; // [file] 8 bytes each
inline constexpr int kMaxIncludeModels = kUncapped; // [file] 8 bytes each

// --- IK / pose / constraints ----------------------------------------------
inline constexpr int kMaxIkRules        = kUncapped; // [file] 88 bytes each
inline constexpr int kMaxIkChains       = INT16_MAX; // [format] mstudioikrulezeroframe_t::chain int16
inline constexpr int kMaxIkAutoplayLocks = kUncapped; // [file]
inline constexpr int kMaxSeqIkLocks     = kUncapped; // [file] 20 bytes each
inline constexpr int kMaxAutolayers     = kUncapped; // [file] 16 bytes each
inline constexpr int kMaxPoseParam      = INT16_MAX; // [format] mstudioautolayer_t::iPose int16
inline constexpr int kMaxBoneConstraints = kUncapped; // [file]
inline constexpr int kMaxCmds           = kUncapped; // [tool] script-side only

// --- Weightlists ------------------------------------------------------------
inline constexpr int kMaxWeightlists    = kUncapped; // [tool] script-side only
inline constexpr int kMaxWeightsPerList = kMaxBones; // one per bone

// --- Script preprocessing --------------------------------------------------
inline constexpr int kMaxMacros          = kUncapped; // [tool] script-side only
inline constexpr int kMaxMacroParams     = kUncapped; // [tool] script-side only
// [tool] Runaway-recursion guard - raising it defeats the purpose.
inline constexpr int kMaxMacroExpansions = 65536;

// --- Names ----------------------------------------------------------------
// [tool] String-table names are unbounded, but mstudiomodel_t::name and
// mstudioevent_t::options are inline char[64] - those two truncate at 63.
inline constexpr int kMaxName            = 128;
inline constexpr int kMaxHitboxSetName   = 64;

// --- Physics / collision ---------------------------------------------------
inline constexpr int kMaxPhysSolids      = kMaxBones; // one per physics bone
inline constexpr int kMaxConvexPieces    = kUncapped; // [tool] reference m_maxConvex default was 40
// [format] Convex hull of V verts has 2V-4 tris; 1536 verts -> 3068 tris,
// clearing the 12-bit triangle index below (the real ceiling here).
inline constexpr int kMaxHullVerts       = 1536;
inline constexpr int kMaxHullTris        = 4095; // [format] IVP tri/pierce_index are 12-bit
inline constexpr int kMaxPhysShapes      = kUncapped; // [tool] reference MAX_EXTRA_COLLISION_MODELS + 1
inline constexpr int kMaxExtraSkinnedBones = kUncapped; // compile-time cost only
// [tool] Budgets, not format ceilings - the format caps per-hull, not per-solid.
inline constexpr int kMaxGeneratedHulls = 128;   // VHACD merge target when max_hulls is 0
inline constexpr int kMaxSolidVerts     = 16384; // fatal, not advisory - dense hulls crash vphysics on load

} // namespace pulse::limits

#endif // PULSEMDL_LIMITS_H
