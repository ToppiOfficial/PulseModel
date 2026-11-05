#pragma once

#include <algorithm>
#include <filesystem>
#include <string>
#include <string_view>

namespace pulse {

inline std::filesystem::path FilePath(std::string name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    return std::filesystem::path(name);
}

// UTF-8 path text as std::string; C++20 types it as char8_t, all our strings are char.
inline std::string Utf8(const std::filesystem::path& path) {
    const std::u8string s = path.u8string();
    return std::string(s.begin(), s.end());
}

inline std::string Utf8Generic(const std::filesystem::path& path) {
    const std::u8string s = path.generic_u8string();
    return std::string(s.begin(), s.end());
}

inline std::filesystem::path Utf8Path(std::string_view utf8) {
    return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()));
}

} // namespace pulse
