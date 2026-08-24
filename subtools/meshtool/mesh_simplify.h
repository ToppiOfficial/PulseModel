// mesh_simplify.h - the `simplify` subcommand.
//
// Runs meshoptimizer's simplifier over one or more index runs sharing a vertex
// pool, which is what $lod's decimatemodel / decimateallmodel and $rendermesh's
// $decimate ask for. Request and response are binary files: the caller writes
// one, names both on the command line, and reads the other back.

#pragma once

#include <string>

namespace pulse::tool {

// Returns false and fills `err` on a malformed request or an unwritable answer.
bool RunSimplify(const std::string& requestPath, const std::string& responsePath,
                 std::string* err);

} // namespace pulse::tool
