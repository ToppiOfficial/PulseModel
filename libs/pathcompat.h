#pragma once

#include <algorithm>
#include <filesystem>
#include <string>

namespace pulse {

inline std::filesystem::path FilePath(std::string name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    return std::filesystem::path(name);
}

} // namespace pulse
