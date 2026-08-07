// pulseloader.h - PulseMDL
//
// HALTED: the .pulsemdl front end is shelved indefinitely. pulseloader.cpp is
// out of mdlcompiler/CMakeLists.txt and main.cpp rejects .pulsemdl scripts;
// .pulseqc is the only front end. Kept on disk as reference only.
//
// Loads a .pulsemdl compile script (DMX keyvalues2, format pulsemodel 1, root
// RootModel - see samplecube.pulsemdl) and the DMX sources it references into
// a compile::CompileInput. Phase-1 subset: name, modelmodifierlist (scale),
// rendermeshlist, cdmaterialslist, bodygrouplist, animationlist.

#ifndef PULSEMDL_PULSELOADER_H
#define PULSEMDL_PULSELOADER_H

#include <string>

#include "compile.h"

namespace pulse::loader {

// Parse the script + load referenced DMX sources (paths relative to the
// script's directory). Returns false + err on failure.
bool LoadPulseScript(const char* path, compile::CompileInput& out, std::string* err);

} // namespace pulse::loader

#endif // PULSEMDL_PULSELOADER_H
