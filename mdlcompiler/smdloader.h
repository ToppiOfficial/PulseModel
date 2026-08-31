// smdloader.h - PulseMDL
//
// Loads a legacy Valve .smd (StudioMdl Data, keyvalues-free text) into the
// compiler's per-file Source (source.h), filling the same fields the DMX
// loader does so both feed the identical compile stage.
//
// SMD is legacy support; DMX stays the primary format.
//
// Current surface: `version`, `nodes` (skeleton), `skeleton` (the BindPose
// animation, one or many frames) and `triangles` (render mesh). Like the DMX
// loader an animation SMD keeps all its frames in the single "BindPose" source
// anim; the compile stage falls back to it when a source carries no separate
// named clip. A mesh SMD must still go through a rendermesh reference, exactly
// like a DMX body - the front ends pick this loader purely by file extension.
//
// A `vertexanimation` block inside a mesh .smd is skipped; VTA morphs come in
// through LoadVtaMorphs, which a $rendermesh's $vta option drives.

#ifndef PULSEMDL_SMDLOADER_H
#define PULSEMDL_SMDLOADER_H

#include <string>
#include <vector>

#include "source.h"

namespace pulse::source {

struct MeshFilter; // meshedit.h - $removemeshword material keywords

// Load one .smd from `path` into `out`. `mats` is the compile-wide material
// registry (unused until `triangles` lands). `scale` is the per-source import
// scale (reference g_currentscale, applied to every position). `morphSource` is
// accepted for signature parity with LoadDmxSource; SMD carries no flex rig.
// `filter` supplies $removemeshword keywords ($exceptionlist needs named meshes
// an SMD lacks, so only the material keywords apply here).
// `animOnly` = loaded as an animation source ($animation / $sequence): `nodes`
// and `skeleton` only, so a `triangles` block contributes no geometry and no
// materials (same rule as LoadDmxSource).
bool LoadSmdSource(const std::string& path, Source& out, MaterialTable& mats,
                   float scale, std::string* err, bool morphSource = false,
                   const MeshFilter* filter = nullptr, bool animOnly = false);

// One `flex` line in a $rendermesh's $vta { } body: the morph a .vta frame
// becomes. A VTA carries no frame names of its own (what an exporter writes
// after `#` is a comment), so the script names each one and picks it by index,
// like stock studiomdl's `flex ... frame N`.
struct VtaFlexOption {
    std::string name;      // delta state / flexdesc name
    int frame = 0;         // `time N` in the vertexanimation block; 0 is the basis
    float position = 1.0f; // s_flexkey_t.target1
    float decay = 1.0f;    // per-vertex morph speed falloff; 0 = flat 1.0
};

// Import the named .vta frames onto an already-loaded SMD render mesh as delta
// states (`out.morphs` + `out.flexkeys`), the same shape a DMX render mesh
// produces. Frame 0 is the basis every other frame is differenced against.
// `scale` must be the same import scale the mesh was loaded with. Prints the
// frames the file offers, then one line per imported flex.
bool LoadVtaMorphs(const std::string& path, Source& out, float scale,
                   const std::vector<VtaFlexOption>& opts, std::string* err);

// Import a .vca - the same file format, played back instead of posed. Every
// non-basis frame becomes flexdesc "f0".."fN-1" and `name` becomes ONE NWAY flex
// controller that runs the whole sequence as it sweeps 0..1 (baked cloth/flag
// animation, not authored face flex). Must be the mesh's only flex source.
bool LoadVcaMorphs(const std::string& path, Source& out, float scale, const std::string& name,
                   std::string* err);

} // namespace pulse::source

#endif // PULSEMDL_SMDLOADER_H
