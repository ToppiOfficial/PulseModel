#include "qcpackage.h"
#include "pathcompat.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>

namespace pulse::qcpackage {
namespace {
namespace fs = std::filesystem;

std::string Key(std::string value)
{
    std::replace(value.begin(), value.end(), '\\', '/');
    for (char& ch : value)
        if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
    return value;
}

bool Inside(const fs::path& path)
{
    if (path.empty() || path.is_absolute() || path.has_root_name()) return false;
    for (const auto& part : path)
        if (part == "..") return false;
    return true;
}
}

bool Package::IsPackage(const fs::path& path)
{
    return Key(path.extension().string()) == ".qczip";
}

Package::Package(const fs::path& path) : path_(fs::absolute(path).lexically_normal()), input_(path_, std::ios::binary)
{
    if (active) throw std::runtime_error("A QCZip package is already open.");
    if (!input_) throw std::runtime_error("Cannot open QCZip package: " + path_.u8string());

    zip_.m_pIO_opaque = &input_;
    zip_.m_pRead = [](void* context, mz_uint64 offset, void* data, size_t size) -> size_t {
        auto& input = *static_cast<std::ifstream*>(context);
        if (offset > std::uint64_t(std::numeric_limits<std::streamoff>::max())) return 0;
        input.clear();
        input.seekg(static_cast<std::streamoff>(offset));
        input.read(static_cast<char*>(data), static_cast<std::streamsize>(size));
        return static_cast<size_t>(input.gcount());
    };
    if (!mz_zip_reader_init(&zip_, fs::file_size(path_), 0))
        throw std::runtime_error("Invalid QCZip: " + std::string(mz_zip_get_error_string(mz_zip_get_last_error(&zip_))));

    try {
        const auto count = mz_zip_reader_get_num_files(&zip_);
        if (count > 100000) throw std::runtime_error("QCZip package has too many entries.");
        for (unsigned i = 0; i < count; ++i) {
            mz_zip_archive_file_stat stat{};
            if (!mz_zip_reader_file_stat(&zip_, i, &stat)) throw std::runtime_error("Invalid QCZip entry.");
            const auto length = mz_zip_reader_get_filename(&zip_, i, nullptr, 0);
            std::string name(length, '\0');
            if (!length || mz_zip_reader_get_filename(&zip_, i, name.data(), length) != length)
                throw std::runtime_error("Invalid QCZip member name.");
            name.pop_back();
            std::replace(name.begin(), name.end(), '\\', '/');
            const auto member = pulse::FilePath(name);
            if (name.find('\0') != std::string::npos || name.find(':') != std::string::npos ||
                !Inside(member) || ((stat.m_external_attr >> 16) & 0170000) == 0120000)
                throw std::runtime_error("Unsafe QCZip member: " + name);
            if (stat.m_is_encrypted || !stat.m_is_supported)
                throw std::runtime_error("Unsupported compression or encryption in QCZip member: " + name);
            if (stat.m_is_directory) continue;

            name = member.lexically_normal().generic_u8string();
            if (name == "." || !members_.emplace(Key(name), Member{i, name, stat.m_uncomp_size}).second)
                throw std::runtime_error("Duplicate or invalid QCZip member: " + name);
        }
        if (!Contains(Primary())) throw std::runtime_error("QCZip package requires model.qc at its root.");
    } catch (...) {
        mz_zip_reader_end(&zip_);
        throw;
    }
    active = this;
}

Package::~Package()
{
    if (active == this) active = nullptr;
    mz_zip_reader_end(&zip_);
    if (!temporary_.empty()) {
        std::error_code error;
        fs::remove_all(temporary_, error);
    }
}

const Package::Member* Package::Find(const fs::path& path) const
{
    const auto full = fs::absolute(path).lexically_normal();
    const auto relative = full.lexically_relative(path_);
    if (!Inside(relative)) return nullptr;
    const auto found = members_.find(Key(relative.generic_u8string()));
    return found == members_.end() ? nullptr : &found->second;
}

fs::path Package::Internal(const fs::path& path) const
{
    const auto full = fs::absolute(path).lexically_normal();
    auto relative = full.lexically_relative(path_);
    if (!Inside(relative)) relative = full.lexically_relative(path_.parent_path());
    if (!Inside(relative)) return {};
    const auto found = members_.find(Key(relative.generic_u8string()));
    return found == members_.end() ? fs::path{} : path_ / pulse::FilePath(found->second.name);
}

fs::path Package::Primary() const { return Logical(path_ / "model.qc"); }

bool Package::Contains(const fs::path& path) const { return Find(path) != nullptr; }

std::string Package::Read(const fs::path& path)
{
    const auto* member = Find(path);
    if (!member) throw std::runtime_error("Missing QCZip member: " + path.u8string());
    if (member->size > (64u << 20)) throw std::runtime_error("QCZip script exceeds 64 MiB: " + member->name);
    std::string text(static_cast<size_t>(member->size), '\0');
    if (!mz_zip_reader_extract_to_mem(&zip_, member->index, text.data(), text.size(), 0))
        throw std::runtime_error("Corrupt QCZip member: " + member->name);
    return text;
}

fs::path Package::Materialize(const fs::path& path)
{
    const auto* member = Find(path);
    if (!member) return {};
    const auto logical = path_ / pulse::FilePath(member->name);
    for (const auto& [physical, source] : materialized_)
        if (source == logical) return physical;

    if (temporary_.empty()) {
        std::mt19937_64 random(std::random_device{}());
        for (int attempt = 0; attempt < 32; ++attempt) {
            const auto candidate = fs::temp_directory_path() / ("PulseModel-qc-" + std::to_string(random()));
            std::error_code error;
            if (fs::create_directory(candidate, error)) { temporary_ = candidate; break; }
        }
        if (temporary_.empty()) throw std::runtime_error("Cannot create QCZip temporary directory.");
    }
    const auto extension = pulse::FilePath(member->name).extension().string();
    const auto physical = temporary_ / (std::to_string(member->index) + extension);
    std::ofstream output(physical, std::ios::binary);
    if (!output) throw std::runtime_error("Cannot prepare QCZip member: " + member->name);
    const auto write = [](void* context, mz_uint64, const void* data, size_t size) -> size_t {
        auto& stream = *static_cast<std::ofstream*>(context);
        stream.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
        return stream ? size : 0;
    };
    if (!mz_zip_reader_extract_to_callback(&zip_, member->index, write, &output, 0))
        throw std::runtime_error("Cannot extract QCZip member: " + member->name);
    output.close();
    if (!output) throw std::runtime_error("Cannot finish QCZip member: " + member->name);
    materialized_.emplace(physical.lexically_normal(), logical);
    assert(Logical(physical) == logical);
    return physical;
}

fs::path Package::External(const fs::path& path) const
{
    const auto relative = fs::absolute(path).lexically_normal().lexically_relative(path_);
    return Inside(relative) ? path_.parent_path() / relative : path;
}

fs::path Package::Logical(const fs::path& path) const
{
    const auto found = materialized_.find(fs::absolute(path).lexically_normal());
    if (found != materialized_.end()) return found->second;
    const auto* member = Find(path);
    return member ? path_ / pulse::FilePath(member->name) : path;
}

std::uint64_t Package::Hash() const
{
    if (hash_) return hash_;
    auto& input = input_;
    input.clear();
    input.seekg(0);
    if (!input) throw std::runtime_error("Cannot hash QCZip package.");
    std::uint64_t hash = 0xcbf29ce484222325ull;
    std::array<unsigned char, 65536> bytes{};
    while (input) {
        input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
        for (std::streamsize i = 0; i < input.gcount(); ++i)
            hash = (hash ^ bytes[static_cast<size_t>(i)]) * 0x100000001b3ull;
    }
    if (input.bad() || !input.eof()) throw std::runtime_error("Cannot finish hashing QCZip package.");
    hash_ = hash;
    return hash;
}

std::string Package::Describe(std::string text) const
{
    for (const auto& [physical, logical] : materialized_) {
        const auto from = physical.string();
        size_t position = 0;
        while ((position = text.find(from, position)) != std::string::npos) {
            const auto to = logical.string();
            text.replace(position, from.size(), to);
            position += to.size();
        }
    }
    return text;
}

std::string Logical(const std::string& path)
{
    return active && !path.empty() ? active->Logical(pulse::FilePath(path)).string() : path;
}
}