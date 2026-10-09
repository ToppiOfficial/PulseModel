#pragma once
#include <array>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace ag2 {
class Vpk {
    struct Entry { uint16_t archive; uint32_t offset, size; std::vector<uint8_t> preload; };
    std::filesystem::path directory;
    uint64_t embeddedOffset = 0;
    std::map<std::string, Entry> entries;
    static uint32_t Read32(std::istream& stream) {
        uint32_t value = 0;
        for (int i = 0; i < 4; ++i) { int byte = stream.get(); if (byte < 0) throw std::runtime_error("Truncated VPK"); value |= static_cast<uint32_t>(byte) << (8 * i); }
        return value;
    }
public:
    explicit Vpk(const std::filesystem::path& path) : directory(path) {
        std::ifstream file(path, std::ios::binary);
        if (!file || Read32(file) != 0x55aa1234) throw std::runtime_error("Invalid VPK: " + path.u8string());
        uint32_t version = Read32(file), treeSize = Read32(file);
        if ((version != 1 && version != 2) || treeSize > 128 * 1024 * 1024) throw std::runtime_error("Unsupported VPK header");
        size_t header = version == 1 ? 12 : 28;
        file.seekg(static_cast<std::streamoff>(header));
        std::vector<uint8_t> tree(treeSize);
        file.read(reinterpret_cast<char*>(tree.data()), treeSize);
        if (!file) throw std::runtime_error("Truncated VPK directory");
        embeddedOffset = header + treeSize;
        size_t cursor = 0;
        auto number = [&](size_t width) {
            if (width > tree.size() - cursor) throw std::runtime_error("Truncated VPK entry");
            uint32_t value = 0;
            for (size_t i = 0; i < width; ++i) value |= static_cast<uint32_t>(tree[cursor++]) << (8 * i);
            return value;
        };
        auto string = [&]() {
            std::string value;
            while (cursor < tree.size()) { char byte = static_cast<char>(tree[cursor++]); if (!byte) return value; value += byte; }
            throw std::runtime_error("Unterminated VPK string");
        };
        for (auto extension = string(); !extension.empty(); extension = string()) {
            for (auto folder = string(); !folder.empty(); folder = string()) {
                for (auto stem = string(); !stem.empty(); stem = string()) {
                    number(4);
                    auto preloadSize = number(2);
                    Entry entry{};
                    entry.archive = static_cast<uint16_t>(number(2)); entry.offset = number(4); entry.size = number(4);
                    if (number(2) != 0xffff || preloadSize > tree.size() - cursor) throw std::runtime_error("Invalid VPK entry");
                    entry.preload.assign(tree.begin() + cursor, tree.begin() + cursor + preloadSize); cursor += preloadSize;
                    entries.emplace((folder == " " ? "" : folder + "/") + stem + (extension == " " ? "" : "." + extension), std::move(entry));
                }
            }
        }
    }
    std::vector<uint8_t> Read(const std::string& name) const {
        auto it = entries.find(name);
        if (it == entries.end()) throw std::runtime_error("Resource missing from CS2 VPK: " + name);
        const auto& entry = it->second;
        if (entry.size > 256 * 1024 * 1024) throw std::runtime_error("VPK resource exceeds size limit");
        auto archive = directory;
        uint64_t offset = entry.offset;
        if (entry.archive == 0x7fff) offset += embeddedOffset;
        else {
            auto filename = directory.filename().u8string();
            auto suffix = filename.rfind("_dir.vpk");
            if (suffix == std::string::npos) throw std::runtime_error("VPK directory filename must end in _dir.vpk");
            std::array<char, 16> index{};
            std::snprintf(index.data(), index.size(), "_%03u.vpk", static_cast<unsigned>(entry.archive));
            archive = directory.parent_path() / std::filesystem::u8path(filename.substr(0, suffix) + index.data());
        }
        std::ifstream file(archive, std::ios::binary);
        if (!file) throw std::runtime_error("VPK archive missing: " + archive.u8string());
        std::vector<uint8_t> data = entry.preload;
        data.resize(data.size() + entry.size);
        file.seekg(static_cast<std::streamoff>(offset));
        file.read(reinterpret_cast<char*>(data.data() + entry.preload.size()), entry.size);
        if (!file) throw std::runtime_error("Truncated VPK resource: " + name);
        return data;
    }
};
}
