// dependencies.h - PulseModel
//
// API compile and analyze: every file the script read (the script, its includes and each
// source). Compile adds a content hash so a tool can tell when the model is out of date.

#ifndef PULSEMDL_DEPENDENCIES_H
#define PULSEMDL_DEPENDENCIES_H

#include <cinttypes>
#include "api.h"
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

namespace pulse::dependencies {

enum class Kind { Script, Include, Source };

// DME bone markup and physics shape counts of a DMX source.
struct Markup {
    int jiggleBones = 0;
    int proceduralBones = 0;
    int physicsShapes = 0;
};

struct Entry {
    Kind kind;
    uint64_t hash;
    bool dmx = false;
    Markup markup;
};

inline bool g_enabled = false;
inline bool g_hashes = true; // analyze skips them; it already reads every source once

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

inline std::string Key(const std::string& path) {
    std::error_code ec;
    const auto full = std::filesystem::absolute(path, ec);
    return ec ? path : full.lexically_normal().string();
}

// `text` stands in for the file's content (an unsaved API buffer).
inline void Note(const std::string& path, Kind kind = Kind::Source,
                 const std::string* text = nullptr) {
    if (!g_enabled || path.empty())
        return;
    const std::string key = Key(path);
    if (Files().count(key))
        return;

    uint64_t hash = 0;
    if (g_hashes)
        hash = text ? Hash(reinterpret_cast<const unsigned char*>(text->data()), text->size())
                    : HashFile(key);
    Files().emplace(key, Entry{kind, hash});
}

inline void NoteMarkup(const std::string& path, const Markup& markup) {
    if (!g_enabled || path.empty())
        return;
    const auto it = Files().find(Key(path));
    if (it == Files().end())
        return;
    it->second.dmx = true;
    it->second.markup = markup;
}

// "@dep <script|include|source> <16 hex digit hash> <absolute path>" per file, then "@dep end".
// API records add the DMX markup counts; `complete` is false when the load stopped early.
inline void Print(bool complete = true) {
    for (const auto& [path, e] : Files()) {
        const char* name = e.kind == Kind::Script ? "script" : e.kind == Kind::Include ? "include" : "source";
        char hash[17]; std::snprintf(hash, sizeof(hash), "%016" PRIx64, e.hash);
        if (!pulse::api::Active()) {
            std::printf("@dep %s %s %s\n", name, hash, path.c_str());
            continue;
        }

        std::string record = "{\"kind\":" + pulse::api::String(name);
        if (g_hashes) record += ",\"hash\":" + pulse::api::String(hash);
        record += ",\"path\":" + pulse::api::String(path);
        if (e.dmx)
            record += ",\"jiggleBones\":" + std::to_string(e.markup.jiggleBones) +
                      ",\"proceduralBones\":" + std::to_string(e.markup.proceduralBones) +
                      ",\"physicsShapes\":" + std::to_string(e.markup.physicsShapes);
        pulse::api::Record("dependency", record + "}");
    }
    if (pulse::api::Active())
        pulse::api::Record("dependencies", std::string("{\"complete\":") + (complete ? "true" : "false") + "}");
    else std::printf("@dep end\n");
}

} // namespace pulse::dependencies

#endif // PULSEMDL_DEPENDENCIES_H
