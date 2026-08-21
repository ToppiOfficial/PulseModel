// qcloader.h - PulseMDL
//
// keyvalues1 compile-script front end, and the only one. Reads `.pulseqc` (and
// `.qc`, which is accepted as the same format) into compile::CompileInput.
//
// Commands are removed, renamed and changed from stock QC; the supported set
// is the kCommands table in qcloader.cpp. An unrecognized `$command` is a
// hard error, never a silent skip - so a script that compiles did what it said.

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
// -includesearchdir / -filesearchdir entries: extra fallback dirs in effect
// before the first line, for $include and for source files respectively. A
// relative dir resolves against the working directory.
using SearchDirs = std::vector<std::string>;

bool LoadQcScript(const char* path, compile::CompileInput& out, std::string* err,
                  const ScriptVars& defvars = {}, const SearchDirs& includeDirs = {},
                  const SearchDirs& fileDirs = {});

// True when the path's extension selects this front end (.pulseqc / .qc).
bool IsQcScriptPath(const char* path);

// Every accepted $command name, one per line, to stdout. The kCommands table is
// the spec, so tools read the surface from the binary instead of a copy of it.
void PrintCommandNames();

} // namespace pulse::loader

#endif // PULSEMDL_QCLOADER_H
