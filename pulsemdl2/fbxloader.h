// fbxloader.h - PulseMDL
//
// Loads an .fbx mesh + skeleton into the compiler's per-file Source (source.h),
// via the vendored ufbx reader (libs/ufbx, MIT). DMX stays the primary format;
// this is an import convenience for authoring tools that only export FBX.
//
// The whole surface: node hierarchy (skeleton, rest pose), polygon meshes
// (triangulated, skinned, one UV set) and blend shapes (morphs). Markup and
// animation stacks are not read - author those in .pulseqc.

#ifndef PULSEMDL_FBXLOADER_H
#define PULSEMDL_FBXLOADER_H

#include <string>

#include "meshedit.h"
#include "source.h"

namespace pulse::source {

// Load one .fbx from `path` into `out`. `mats` is the compile-wide material
// registry, `scale` the per-source import scale (reference g_currentscale).
// `morphSource` = blend shapes become morphs/flexkeys (the $rendermesh gate).
// `filter` = $rendermesh $exceptionlist, matched against the mesh element's
// name and its node's; a rejected mesh contributes no geometry and no
// materials, but its node still becomes a bone (same rule as the DMX loader).
// `animOnly` = skeleton only, so meshes contribute no geometry and no
// materials (same rule as LoadDmxSource / LoadSmdSource).
bool LoadFbxSource(const std::string& path, Source& out, MaterialTable& mats, float scale,
                   std::string* err, bool morphSource = false, MeshFilter* filter = nullptr,
                   bool animOnly = false);

} // namespace pulse::source

#endif // PULSEMDL_FBXLOADER_H
