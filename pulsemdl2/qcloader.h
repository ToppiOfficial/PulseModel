// qcloader.h - PulseMDL
//
// keyvalues1 compile-script front end. Reads `.pulseqc` (and `.qc`, which is
// accepted as the same format) into the SAME compile::CompileInput that the
// keyvalues2 `.pulsemdl` loader fills, so nothing downstream - compile stage,
// writers - knows or cares which front end ran.
//
// Commands are removed, renamed and changed from stock QC; the supported set
// is the table at the top of qcloader.cpp. An unrecognized `$command` is a
// hard error, never a silent skip - so a script that compiles did what it said.
//
// `.pulsemdl` stays the primary format; this is an additional front end, not
// a replacement.

#ifndef PULSEMDL_QCLOADER_H
#define PULSEMDL_QCLOADER_H

#include <string>
#include <utility>
#include <vector>

#include "compile.h"

namespace pulse::loader {

// -defvar <name> <value> pairs, in command-line order: script variables that
// exist before the first line runs. A name here is PINNED - a $definevariable
// or $redefinevariable for it is ignored, so the launch value wins.
using ScriptVars = std::vector<std::pair<std::string, std::string>>;

// Parse the script + load referenced DMX sources (paths relative to the
// script's directory). Returns false + err on failure.
bool LoadQcScript(const char* path, compile::CompileInput& out, std::string* err,
                  const ScriptVars& defvars = {});

// True when the path's extension selects this front end (.pulseqc / .qc).
bool IsQcScriptPath(const char* path);

} // namespace pulse::loader

#endif // PULSEMDL_QCLOADER_H
