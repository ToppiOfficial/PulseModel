// dmxrig.h - PulseMDL
//
// $datamodeljoints: DME bone markup (DmeJiggleBone, DmeQuatInterpBone,
// DmeAimAtBone, DmeAttachment dags, root hitboxSetList) - script-requested
// only, never picked up by a render mesh load, and may point at a different
// file entirely. Only touches the markup lists, not the skeleton.
//
// Reference: HandleDmeJiggleBone/HandleDmeQuatInterpBone/
// HandleDmeAimAtBone/LoadAttachments/LoadDmxHitboxes.

#ifndef PULSEMDL_DMXRIG_H
#define PULSEMDL_DMXRIG_H

#include <string>

#include "compile.h"
#include "dmx/dmx.h"

namespace pulse::loader {

// Append requested markup to `in`. `jigglebones`/`proceduralbones`/`attachments`
// walk the skeleton dag tree (DmeJiggleBone, DmeQuatInterpBone+DmeAimAtBone,
// DmeAttachment); `hitboxes` reads the root's hitboxSetList. First definition
// of a name wins, so an earlier $jigglebone beats the DMX; any hitbox set
// already declared makes the DMX list a no-op (all-or-nothing). Imported
// attachments are tagged kAttachIsFromSource and must stay ordered after
// scripted ones (reference ReorderSourceAttachmentsLast).
bool LoadDmxJoints(const pulse::dmx::Datamodel& dm, compile::CompileInput& in,
                   bool jigglebones, bool proceduralbones, bool hitboxes,
                   bool attachments, std::string* err);

} // namespace pulse::loader

#endif // PULSEMDL_DMXRIG_H
