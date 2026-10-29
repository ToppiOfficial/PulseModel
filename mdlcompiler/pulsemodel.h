// pulsemodel.h - PulseModel
//
// `.pulsemodel` front end: a JSON model description. Each section lowers to the
// .pulseqc commands it stands for and runs through the same handlers, so both
// formats share one compile path. The schema table in pulsemodel.cpp is the spec.

#ifndef PULSEMDL_PULSEMODEL_H
#define PULSEMDL_PULSEMODEL_H

#include <string>

#include "compile.h"
#include "qcloader.h"

namespace pulse::pulsemodel {

struct LoadOptions {
    std::string only;                // -only <id>: the model to compile from a multi-model file
    loader::ScriptVars defvars;      // -defvar: pinned, a file cannot change them
    loader::SearchDirs fileDirs;     // -filesearchdir
    const loader::ScriptFiles* unsaved = nullptr;
};

// True when the path's extension selects this front end (.pulsemodel).
bool IsPulseModelPath(const char* path);

// Loads one model of the file into `out`: the `-only` id, else the single model
// whose `compile` is not false. Returns false + err on failure.
bool Load(const char* path, const LoadOptions& options, compile::CompileInput& out,
          std::string* err);

// The whole schema as one API "schema" record.
void PrintSchema();

// After a successful Load: one API "resolved" record, the model with variables,
// prefabs and shared content merged, and the file + JSON pointer each part came from.
void PrintResolved();

// After a successful compile of a loaded model: one API "compiled" record mapping
// each compiled bone, attachment and hitbox to the value that declared it, if any,
// plus the source bones the compile collapsed away.
void PrintCompiled(const compile::CompileInput& input, const compile::CompiledModel& model);

} // namespace pulse::pulsemodel

#endif // PULSEMDL_PULSEMODEL_H
