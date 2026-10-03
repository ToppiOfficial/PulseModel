// dependencies.h - PulseModel
//
// -dumpdependencies: every file the compile read (the script, its includes and each
// source), so a tool can tell when the model it compiled has gone out of date.

#ifndef PULSEMDL_DEPENDENCIES_H
#define PULSEMDL_DEPENDENCIES_H

#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

namespace pulse::dependencies {

enum class Kind { Script, Include, Source };

inline bool g_enabled = false;

inline std::map<std::string, Kind>& Files() {
    static std::map<std::string, Kind> files;
    return files;
}

inline void Note(const std::string& path, Kind kind = Kind::Source) {
    if (!g_enabled || path.empty())
        return;
    std::error_code ec;
    const auto full = std::filesystem::absolute(path, ec);
    Files().emplace(ec ? path : full.lexically_normal().string(), kind);
}

// One "<kind>\t<absolute path>" per line, kind being script, include or source.
inline bool Write(const std::string& out) {
    std::FILE* f = std::fopen(out.c_str(), "wb");
    if (!f)
        return false;
    for (const auto& [path, kind] : Files()) {
        const char* name = kind == Kind::Script ? "script" : kind == Kind::Include ? "include" : "source";
        std::fprintf(f, "%s\t%s\n", name, path.c_str());
    }
    return std::fclose(f) == 0;
}

} // namespace pulse::dependencies

#endif // PULSEMDL_DEPENDENCIES_H
