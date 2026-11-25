// mdlasmload.h - the .mdlasm front end: decodes a model assembly and fills compile::CompileInput
// through the same Apply functions the .qc loader uses (scriptapply.h).

#ifndef PULSEMODEL_MDLASMLOAD_H
#define PULSEMODEL_MDLASMLOAD_H

#include <string>

#include "compile.h"

namespace pulse::loader {

// True when the path selects a model assembly (.mdlasm).
bool IsMdlAsmPath(const char* path);

// Reads the assembly and every source embedded in it. Returns false and fills `err` on failure.
bool LoadMdlAsm(const char* path, compile::CompileInput& out, std::string* err);

} // namespace pulse::loader

#endif // PULSEMODEL_MDLASMLOAD_H
