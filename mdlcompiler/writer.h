// writer.h - PulseMDL
//
// Binary writers: .mdl / .vvd / .dx90.vtx. Buffers are built in memory
// (including the post-write fixup pass) and saved once; the bytes match the
// reference's write->reload->fixup->rewrite dance.

#ifndef PULSEMDL_WRITER_H
#define PULSEMDL_WRITER_H

#include <cstdint>
#include <string>
#include <vector>

#include "compile.h"

namespace pulse::writer {

// Deferred console report lines for the .vtx (the .mdl/.vvd equivalents stay
// private to writemdl.cpp). BuildVtx fills this while it lays the file out;
// WriteModelFiles prints it under the "writing <path>:" header, because the
// output path is not resolved until save time.
extern std::vector<std::string> g_vtxReport;

// Every file WriteModelFiles saved, in write order, for -editorinfo. The .mdl
// name comes from $modelname, which is variable-driven, so a caller cannot
// predict the paths.
extern std::vector<std::string> g_writtenFiles;

// Build the .dx90.vtx buffer (reference OptimizedModel::OptimizeFromStudioHdr,
// dx90 params: vertcache 24, 3 bones/vert, 9/tri, 53/strip, hw flex).
// `mdlBuf` is the already-built .mdl buffer (the vtx builder reads the
// studiohdr tables from it and MUTATES vvd bone weights in `vvdBuf` only in
// its in-memory copy, matching the reference's cached-vvd behavior).
std::vector<uint8_t> BuildVtx(compile::CompiledModel& mdl, std::vector<uint8_t>& mdlBuf,
                              std::vector<uint8_t>& vvdBuf, bool legacyVtx);

// Build the .phy buffer (reference CollisionModel_Write). Returns empty when
// the model has no collision solids, in which case no .phy is written. The
// checksum must be the one already stamped into the .mdl.
std::vector<uint8_t> BuildPhy(compile::CompiledModel& mdl, int32_t checksum);

// Write <outDir>/<outname>.mdl/.vvd/.dx90.vtx (+ .phy when there is collision
// data). outDir may be empty (cwd).
bool WriteModelFiles(compile::CompiledModel& mdl, const std::string& outDir, bool legacyVtx,
                     std::string* err);

} // namespace pulse::writer

#endif // PULSEMDL_WRITER_H
