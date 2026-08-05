// dmxloader.h - PulseMDL
//
// Maps a parsed DMX document into the compiler's per-file Source (source.h).
// Behavior port of the reference LoadModelAndSkeleton/LoadBindPose/
// LoadMeshes/LoadAnimations + UnifyIndices/BuildIndividualMeshes.
// Handles both geometry DMX and animation DMX; `animOnly` skips geometry.

#ifndef PULSEMDL_DMXLOADER_H
#define PULSEMDL_DMXLOADER_H

#include <string>

#include "dmx/dmx.h"
#include "meshedit.h"
#include "source.h"

namespace pulse::source {

// `mats` = compile-wide material registry (face materials index it).
// `scale` = per-source import scale (reference g_currentscale).
// `morphSource` = delta states become morphs/flexkeys (reference LoadingModelBody
// gate); the combination-operator rig itself is $datamodelflexes' job, see
// LoadDmxFlexRig.
// `filter` = $rendermesh $exceptionlist: rejected mesh dags contribute nothing
// and are written back through so the caller can flag names that matched nothing.
// `animOnly` = skeleton + channel clips only, no mesh dags read at all (materials
// never reach `mats`, no morphs).
bool LoadDmxSource(const pulse::dmx::Datamodel& dm, Source& out,
                   MaterialTable& mats, float scale, std::string* err,
                   bool morphSource = false, MeshFilter* filter = nullptr,
                   bool animOnly = false);

// $datamodelflexes: pull just the combination-operator rig out of a DMX (no
// skeleton/mesh/morphs). A file with no combination operator yields an empty
// rig, not an error.
bool LoadDmxFlexRig(const pulse::dmx::Datamodel& dm, FlexRig& out, std::string* err);

// True when the loaded DmeModel declared upAxis "Y*" (reference sets
// g_defaultrotation = (pi/2, 0, pi/2) in that case). Sticky across loads,
// mirroring the reference's set-once bSetUpAxis behavior.
bool DmxUpAxisY();
void ResetDmxUpAxis(); // test hook

} // namespace pulse::source

#endif // PULSEMDL_DMXLOADER_H
