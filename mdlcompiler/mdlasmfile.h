// mdlasmfile.h - the .mdlasm container: a signature, a chunk directory and zstd chunks.
//
// Layout: 8-byte signature, container version, document version, minimum reader version, chunk
// count, directory offset (32 bytes), the chunk streams, then 40-byte directory entries.

#ifndef PULSEMODEL_MDLASMFILE_H
#define PULSEMODEL_MDLASMFILE_H

#include <array>
#include <bit>
#include <cstdint>
#include <istream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace pulse::mdlasm::core {

// A chunk flag: a reader that does not know the chunk's type must refuse the file.
inline constexpr std::uint32_t ChunkRequired = 1;

struct Chunk {
    std::uint32_t type = 0, schema = 1, codec = 1, flags = 0;
    std::uint64_t offset = 0, stored = 0, raw = 0;
};

// Little-endian values and length-prefixed strings with a byte budget, so a bad count fails the
// record instead of allocating.
class BinaryReader {
public:
    BinaryReader(std::istream& stream, std::uint64_t bytes) : stream_(stream), remaining_(bytes) {}

    template<class T> T read() requires std::is_arithmetic_v<T> {
        if constexpr (std::is_same_v<T, bool>) {
            const auto value = read<std::uint8_t>();
            if (value > 1) throw std::runtime_error("Invalid boolean.");
            return value != 0;
        } else if constexpr (std::is_floating_point_v<T>) {
            if constexpr (sizeof(T) == 8) return std::bit_cast<T>(read<std::uint64_t>());
            else return std::bit_cast<T>(read<std::uint32_t>());
        } else {
            using U = std::make_unsigned_t<T>;
            unsigned char bytes[sizeof(T)];
            take(reinterpret_cast<char*>(bytes), sizeof(bytes));
            U value = 0;
            for (std::size_t i = 0; i < sizeof(T); ++i) value |= U(bytes[i]) << (8 * i);
            return std::bit_cast<T>(value);
        }
    }

    std::string text();
    std::uint64_t count(std::uint64_t minimumBytes = 1);
    std::uint64_t remaining() const { return remaining_; }
    void finish() const {
        if (remaining_) throw std::runtime_error("Unexpected trailing data.");
    }

private:
    void take(char* data, std::uint64_t bytes);

    std::istream& stream_;
    std::uint64_t remaining_;
};

// Reads the whole file up front; chunks decompress on demand.
class ChunkReader {
public:
    explicit ChunkReader(const std::string& path);

    std::uint32_t documentVersion() const { return documentVersion_; }
    const std::vector<Chunk>& chunks() const { return chunks_; }

    // Decompresses one chunk and hands it to `consume(stream, rawBytes)`.
    template<class Consume> void read(const Chunk& chunk, Consume&& consume) {
        std::istringstream stream(decompress(chunk), std::ios::binary);
        consume(stream, chunk.raw);
    }

private:
    std::string decompress(const Chunk& chunk) const;

    std::string file_;
    std::uint32_t documentVersion_ = 0;
    std::vector<Chunk> chunks_;
};

template<class Parse>
void parseRecord(const std::string& bytes, Parse&& parse) {
    std::istringstream buffer(bytes, std::ios::binary);
    BinaryReader reader(buffer, bytes.size());
    parse(reader);
}

} // namespace pulse::mdlasm::core

#endif // PULSEMODEL_MDLASMFILE_H
