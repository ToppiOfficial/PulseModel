// convex_decompose.h - the `decompose` subcommand.
//
// Runs VHACD's convex decomposition over one or more closed triangle meshes,
// which is what $physicsshape fromrender asks for. Request and response are
// binary files: the caller writes one, names both on the command line, and
// reads the other back. Each job answers with its pieces as point clouds - the
// caller re-hulls them, exactly as mdlcompiler does.

#pragma once

#include <string>

namespace pulse::tool {

// Returns false and fills `err` on a malformed request or an unwritable answer.
bool RunDecompose(const std::string& requestPath, const std::string& responsePath,
                  std::string* err);

} // namespace pulse::tool
