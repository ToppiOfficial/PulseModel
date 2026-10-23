// rename.cpp - PulseModel
//
// -api rename: writes a compiled model under a new $modelname without a
// decompile/compile round trip. Only the .mdl carries names; the rest is copied.

#include "rename.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include "apiprotocol.h"
#include "diagnostic.h"
#include "format/mdl.h"
#include "pathcompat.h"

namespace pulse::rename {
namespace {

namespace fs = std::filesystem;
namespace fmt = pulse::format;

// Sibling files sharing the .mdl's stem; the .phy and .ani hold no model name.
constexpr const char* kSiblingExtensions[] = {".vvd",     ".dx90.vtx", ".dx80.vtx", ".sw.vtx",
                                              ".xbox.vtx", ".vtx",     ".phy",      ".ani"};

std::string Lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return text;
}

bool ReadBytes(const fs::path& path, std::vector<char>& bytes) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    bytes.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    return !file.bad();
}

bool WriteBytes(const fs::path& path, const std::vector<char>& bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) return false;
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(file);
}

// Model-root-relative name with forward slashes and a .mdl extension; empty when invalid.
std::string NormalizeName(std::string name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    while (!name.empty() && name.front() == '/') name.erase(name.begin());
    if (Lower(name).rfind("models/", 0) == 0) name.erase(0, 7);
    if (name.size() < 4 || Lower(name.substr(name.size() - 4)) != ".mdl") name += ".mdl";

    const fs::path path(name);
    if (name.size() <= 4 || path.has_root_name() || path.filename() == ".mdl") return {};
    for (const fs::path& part : path)
        if (part == ".." || part == ".") return {};
    return name;
}

// String in the .mdl at a file offset, bounded by the buffer.
std::string StringAt(const std::vector<char>& bytes, int64_t offset) {
    if (offset <= 0 || offset >= static_cast<int64_t>(bytes.size())) return {};
    const char* start = bytes.data() + offset;
    return std::string(start, strnlen(start, bytes.size() - static_cast<size_t>(offset)));
}

// The folder the model's internal name is relative to: the source path minus that
// name, else the nearest ancestor called "models".
fs::path FindModelsRoot(const fs::path& source, const std::string& oldName) {
    const std::string sourceText = Lower(source.generic_string());
    const std::string tail = "/" + Lower(NormalizeName(oldName));
    if (tail.size() > 1 && sourceText.size() > tail.size() &&
        sourceText.compare(sourceText.size() - tail.size(), tail.size(), tail) == 0)
        return FilePath(source.generic_string().substr(0, sourceText.size() - tail.size()));

    for (fs::path dir = source.parent_path(); dir.has_relative_path(); dir = dir.parent_path())
        if (Lower(dir.filename().string()) == "models") return dir;
    return {};
}

void AppendString(std::vector<char>& bytes, const std::string& text) {
    while (bytes.size() % 4) bytes.push_back('\0');
    bytes.insert(bytes.end(), text.begin(), text.end());
    bytes.push_back('\0');
}

void ReportOutput(const fs::path& path) {
    if (api::Active()) api::Record("output", "{\"path\":" + api::String(path.generic_string()) + "}");
}

} // namespace

bool Run(const std::vector<std::string>& arguments, std::string* err) {
    std::string sourceArg, nameArg, gameDir;
    bool overwrite = false, move = false;
    for (size_t i = 0; i < arguments.size(); ++i) {
        const std::string& arg = arguments[i];
        if ((arg == "-game" || arg == "-outdir") && i + 1 < arguments.size()) gameDir = arguments[++i];
        else if (arg == "-overwrite") overwrite = true;
        else if (arg == "-move") move = true;
        else if (!arg.empty() && arg[0] == '-') { *err = "unknown rename option " + arg; return false; }
        else if (sourceArg.empty()) sourceArg = arg;
        else if (nameArg.empty()) nameArg = arg;
        else { *err = "unexpected rename argument " + arg; return false; }
    }
    if (sourceArg.empty() || nameArg.empty()) {
        *err = "rename needs a source .mdl and a new model name";
        return false;
    }

    const fs::path source = FilePath(sourceArg);
    const std::string newName = NormalizeName(nameArg);
    if (!source.is_absolute()) { *err = "rename source must be an absolute path"; return false; }
    if (newName.empty()) { *err = "invalid model name \"" + nameArg + "\""; return false; }

    std::vector<char> mdl;
    if (!ReadBytes(source, mdl)) { *err = "cannot read " + source.string(); return false; }
    if (mdl.size() < sizeof(fmt::studiohdr_t)) { *err = source.string() + " is not a model"; return false; }

    fmt::studiohdr_t header;
    std::memcpy(&header, mdl.data(), sizeof(header));
    if (header.id != fmt::kIdStudioHeader || header.version < 44 || header.version > 49) {
        *err = source.string() + " is not a version 44-49 studio model";
        return false;
    }
    const bool hasHeader2 = header.studiohdr2index > 0 &&
                            static_cast<size_t>(header.studiohdr2index) + sizeof(fmt::studiohdr2_t) <= mdl.size();

    // the full name lives in studiohdr2; the fixed header field truncates at 63 chars
    std::string oldName(header.name, strnlen(header.name, sizeof(header.name)));
    fmt::studiohdr2_t header2{};
    if (hasHeader2) {
        std::memcpy(&header2, mdl.data() + header.studiohdr2index, sizeof(header2));
        const std::string fullName = StringAt(mdl, int64_t{header.studiohdr2index} + header2.sznameindex);
        if (header2.sznameindex && !fullName.empty()) oldName = fullName;
    }

    const fs::path root = gameDir.empty() ? FindModelsRoot(source, oldName) : FilePath(gameDir) / "models";
    if (root.empty()) {
        *err = "cannot find the models folder of " + source.string() + "; pass -game <dir>";
        return false;
    }

    const fs::path target = (root / FilePath(newName)).lexically_normal();
    const fs::path sourceStem = source.parent_path() / source.stem();
    const fs::path targetStem = target.parent_path() / target.stem();
    if (Lower(target.generic_string()) == Lower(source.lexically_normal().generic_string())) {
        *err = "the new name is the model's current path";
        return false;
    }

    std::vector<std::string> siblings;
    for (const char* ext : kSiblingExtensions) {
        std::error_code ec;
        if (fs::is_regular_file(sourceStem.string() + ext, ec)) siblings.push_back(ext);
    }
    if (!overwrite) {
        std::error_code ec;
        std::vector<std::string> outputs{".mdl"};
        outputs.insert(outputs.end(), siblings.begin(), siblings.end());
        size_t conflicts = 0;
        for (const std::string& ext : outputs) {
            const fs::path existing = targetStem.string() + ext;
            if (!fs::exists(existing, ec)) continue;
            if (api::Active()) api::Record("conflict", "{\"path\":" + api::String(existing.generic_string()) + "}");
            ++conflicts;
        }
        if (conflicts) {
            *err = std::to_string(conflicts) + " output file(s) already exist; pass -overwrite to replace them";
            return false;
        }
    }

    // new strings are appended past the old table; the old ones become unused bytes
    const std::string newStem = newName.substr(0, newName.size() - 4);
    std::memset(header.name, 0, sizeof(header.name));
    std::strncpy(header.name, newName.c_str(), sizeof(header.name) - 1);
    if (hasHeader2) {
        AppendString(mdl, newName);
        header2.sznameindex = static_cast<int32_t>(mdl.size() - newName.size() - 1 - header.studiohdr2index);
    }
    if (!StringAt(mdl, header.szanimblocknameindex).empty()) {
        const std::string aniName = "models/" + newStem + ".ani";
        AppendString(mdl, aniName);
        header.szanimblocknameindex = static_cast<int32_t>(mdl.size() - aniName.size() - 1);
    }
    while (mdl.size() % 4) mdl.push_back('\0');
    header.length = static_cast<int32_t>(mdl.size());
    std::memcpy(mdl.data(), &header, sizeof(header));
    if (hasHeader2) std::memcpy(mdl.data() + header.studiohdr2index, &header2, sizeof(header2));

    std::error_code ec;
    fs::create_directories(target.parent_path(), ec);
    std::printf("Renaming: %s -> %s\n", oldName.c_str(), newName.c_str());
    if (!WriteBytes(target, mdl)) { *err = "cannot write " + target.string(); return false; }
    ReportOutput(target);
    for (const std::string& ext : siblings) {
        const fs::path to = targetStem.string() + ext;
        if (!fs::copy_file(sourceStem.string() + ext, to, fs::copy_options::overwrite_existing, ec)) {
            *err = "cannot write " + to.string() + ": " + ec.message();
            return false;
        }
        ReportOutput(to);
    }
    if (api::Active()) api::Record("model", "{\"path\":" + api::String(target.generic_string()) + "}");

    if (move) {
        siblings.insert(siblings.begin(), ".mdl");
        for (const std::string& ext : siblings)
            if (!fs::remove(sourceStem.string() + ext, ec) && ec)
                diag::Warnf("cannot remove %s%s: %s\n", sourceStem.string().c_str(), ext.c_str(),
                            ec.message().c_str());
    }
    return true;
}

} // namespace pulse::rename
