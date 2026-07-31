#pragma once
// .phy on-disk format - collision/physics data for a compiled model.
//
// Structs re-declared from the Source SDK 2013 format documentation, not copied.
//
// A .phy is:
//   phyheader_t
//   per solid: int32 blob size, then that many bytes of IVP compact surface
//              (built by libs/minicollision)
//   plaintext keyvalues tail: solid { }, ragdollconstraint { },
//              collisionrules { }, editparams { } ...
//   one NUL terminator byte
//
// The tail is genuinely plain text, not a length-prefixed block - the engine
// parses until the NUL.

#include <cstdint>

namespace pulse::format {

// The SDK declares checkSum as `long`, which is 32-bit under MSVC but 64-bit on
// the Linux ABI. It is a 4-byte field on disk, so it is spelled int32_t here.
struct phyheader_t {
    int32_t size;        // sizeof(phyheader_t) = 16
    int32_t id;          // 0
    int32_t solidCount;  // number of blobs following
    int32_t checkSum;    // must match the .mdl's checksum
};
static_assert(sizeof(phyheader_t) == 16, "phyheader_t size");

} // namespace pulse::format
