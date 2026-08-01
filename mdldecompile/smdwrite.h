// smdwrite.h - rebuilds the animation clips the .mdl/.ani were compiled from.

#ifndef MDLDECOMPILE_SMDWRITE_H
#define MDLDECOMPILE_SMDWRITE_H

#include <string>

#include "mdlfile.h"

namespace mdldecompile {

// One anims/<name>.smd per local animation that carries data, named by the
// AnimRefs() alias the .pulseqc writes. `mdlPath` locates a sibling .ani for
// demand-loaded clips. Reports what it wrote and never fails the decompile.
void WriteAnimationSmds(const Mdl& m, const std::string& mdlPath, const std::string& dir);

} // namespace mdldecompile

#endif // MDLDECOMPILE_SMDWRITE_H
