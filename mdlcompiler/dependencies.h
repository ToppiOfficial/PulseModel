// dependencies.h - PulseModel
//
// -dumpdependencies: every file the compile read (the script, its includes and each
// source) with a hash of its content, so a tool can tell when the model is out of date.

#ifndef PULSEMDL_DEPENDENCIES_H
#define PULSEMDL_DEPENDENCIES_H

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

namespace pulse::dependencies {

enum class Kind { Script, Include, Source };

struct Entry {
    Kind kind;
    uint64_t hash;
};

inline bool g_enabled = false;

inline std::map<std::string, Entry>& Files() {
    static std::map<std::string, Entry> files;
    return files;
}

// FNV-1a 64; PulseWorkshop computes the same hash to check a recorded file.
inline uint64_t Hash(const unsigned char* p, size_t n, uint64_t h = 0xcbf29ce484222325ull) {
    for (size_t i = 0; i < n; ++i)
        h = (h ^ p[i]) * 0x100000001b3ull;
    return h;
}

inline uint64_t HashFile(const std::string& path) {
    uint64_t h = 0xcbf29ce484222325ull;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return h;
    unsigned char buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
        h = Hash(buf, n, h);
    std::fclose(f);
    return h;
}

// `text` stands in for the file's content (a -stdin script).
inline void Note(const std::string& path, Kind kind = Kind::Source,
                 const std::string* text = nullptr) {
    if (!g_enabled || path.empty())
        return;
    std::error_code ec;
    const auto full = std::filesystem::absolute(path, ec);
    const std::string key = ec ? path : full.lexically_normal().string();
    if (Files().count(key))
        return;
    const uint64_t hash = text ? Hash(reinterpret_cast<const unsigned char*>(text->data()), text->size())
                               : HashFile(key);
    Files().emplace(key, Entry{kind, hash});
}

// "@dep <script|include|source> <16 hex digit hash> <absolute path>" per file, then "@dep end".
inline void Print() {
    for (const auto& [path, e] : Files()) {
        const char* name = e.kind == Kind::Script ? "script" : e.kind == Kind::Include ? "include" : "source";
        std::printf("@dep %s %016" PRIx64 " %s\n", name, e.hash, path.c_str());
    }
    std::printf("@dep end\n");
}

} // namespace pulse::dependencies

#endif // PULSEMDL_DEPENDENCIES_H
