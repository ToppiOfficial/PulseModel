// goldsrc.h - decompile of a GoldSrc .mdl (version 10 and earlier).

#ifndef MDLDECOMPILER_GOLDSRC_H
#define MDLDECOMPILER_GOLDSRC_H

#include <string>

namespace mdldecompiler {

// True when the file starts with "IDST" and a version below 44 - the GoldSrc
// layout, which shares nothing with v44+ past the magic.
bool IsGoldSrcMdl(const std::string& path);

// Writes `outPath` plus meshes/*.smd, anims/*.smd and materials/*.bmp under
// `dir`. Returns 0 on success, non-zero after reporting the failure.
int DecompileGoldSrc(const std::string& in, const std::string& dir, const std::string& outPath);

} // namespace mdldecompiler

#endif // MDLDECOMPILER_GOLDSRC_H
