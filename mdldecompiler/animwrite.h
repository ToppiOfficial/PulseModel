// animwrite.h - rebuilds the animation clips the .mdl/.ani were compiled from.

#ifndef MDLDECOMPILER_ANIMWRITE_H
#define MDLDECOMPILER_ANIMWRITE_H

#include <string>

#include "mdlfile.h"

namespace mdldecompiler {

// -smdanimation writes the clips as SMD instead of DMX.
void SetAnimFormat(bool smd);

// ".dmx" or ".smd" - what the .pulseqc has to name.
const char* AnimExt();

// One anims/<name>.<ext> per local animation that carries data, named by the
// AnimRefs() alias the .pulseqc writes. `mdlPath` locates a sibling .ani for
// demand-loaded clips. Reports what it wrote and never fails the decompile.
void WriteAnimationFiles(const Mdl& m, const std::string& mdlPath, const std::string& dir);

} // namespace mdldecompiler

#endif // MDLDECOMPILER_ANIMWRITE_H
