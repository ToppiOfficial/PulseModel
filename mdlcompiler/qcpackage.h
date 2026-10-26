#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <miniz.h>

namespace pulse::qcpackage {

class Package
{
public:
    static bool IsPackage(const std::filesystem::path& path);
    explicit Package(const std::filesystem::path& path);
    ~Package();
    Package(const Package&) = delete;
    Package& operator=(const Package&) = delete;

    const std::filesystem::path& Path() const { return path_; }
    std::filesystem::path Primary() const;
    std::filesystem::path Internal(const std::filesystem::path& path) const;
    bool Contains(const std::filesystem::path& path) const;
    std::string Read(const std::filesystem::path& path);
    std::filesystem::path Materialize(const std::filesystem::path& path);
    std::filesystem::path External(const std::filesystem::path& path) const;
    std::filesystem::path Logical(const std::filesystem::path& path) const;
    std::uint64_t Hash() const;
    std::string Describe(std::string text) const;

private:
    struct Member { unsigned index; std::string name; std::uint64_t size; };
    const Member* Find(const std::filesystem::path& path) const;
    std::filesystem::path path_, temporary_;
    mutable std::ifstream input_;
    mutable std::uint64_t hash_ = 0;
    mz_zip_archive zip_{};
    std::map<std::string, Member> members_;
    std::map<std::filesystem::path, std::filesystem::path> materialized_;
};

inline Package* active = nullptr;
std::string Logical(const std::string& path);

}