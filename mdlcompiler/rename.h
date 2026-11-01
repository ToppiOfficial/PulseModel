#pragma once

#include <string>
#include <vector>

namespace pulse::rename {

// <source.mdl> <new name> [-game <dir>] [-overwrite] [-move]. The new name is relative to
// the models folder; without -game that folder is found from the source's current name.
bool Run(const std::vector<std::string>& arguments, std::string* err);

}
