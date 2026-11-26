// mdlasmfile.cpp - see mdlasmfile.h.

#include "mdlasmfile.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <memory>

#include "zstd/zstd.h"

namespace pulse::mdlasm::core {
namespace {

constexpr std::uint64_t kHeaderBytes = 32, kEntryBytes = 40;
constexpr std::uint32_t kSupportedCodec = 1, kContainerVersion = 1, kNewestDocument = 4;
constexpr std::array<char, 8> kSignature{'M', 'D', 'L', 'A', 'S', 'M', '\0', '\0'};
// The most one chunk may declare; an embedded source is a single chunk.
constexpr std::uint64_t kMaxChunkBytes = std::uint64_t(2) << 30;

template<class T> T peek(const std::string& file, std::uint64_t at) {
    using U = std::make_unsigned_t<T>;
    U value = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
        value |= U(static_cast<unsigned char>(file[at + i])) << (8 * i);
    return static_cast<T>(value);
}

} // namespace

std::string BinaryReader::text() {
    const auto size = count();
    std::string result(static_cast<std::size_t>(size), '\0');
    take(result.data(), size);
    return result;
}

std::uint64_t BinaryReader::count(std::uint64_t minimumBytes) {
    const auto size = read<std::uint64_t>();
    if (size > remaining_ / minimumBytes || size > std::numeric_limits<std::size_t>::max())
        throw std::runtime_error("Invalid data count.");
    return size;
}

void BinaryReader::take(char* data, std::uint64_t bytes) {
    if (bytes > remaining_) throw std::runtime_error("Truncated data.");
    stream_.read(data, static_cast<std::streamsize>(bytes));
    if (!stream_) throw std::runtime_error("Reading data failed.");
    remaining_ -= bytes;
}

ChunkReader::ChunkReader(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Cannot open \"" + path + "\".");
    file_.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());

    const std::uint64_t size = file_.size();
    if (size < kHeaderBytes) throw std::runtime_error("Truncated model assembly header.");
    if (!std::equal(kSignature.begin(), kSignature.end(), file_.begin()))
        throw std::runtime_error("Not a Model Assembler (.mdlasm) file.");

    const auto containerVersion = peek<std::uint32_t>(file_, 8);
    documentVersion_ = peek<std::uint32_t>(file_, 12);
    const auto minimumReader = peek<std::uint32_t>(file_, 16);
    if (containerVersion != kContainerVersion || minimumReader > kNewestDocument)
        throw std::runtime_error("This model assembly was saved by a newer PulseStudio; update PulseModel.");
    if (documentVersion_ < 1 || minimumReader < 1 || minimumReader > documentVersion_)
        throw std::runtime_error("Invalid model assembly format versions.");

    const auto count = peek<std::uint32_t>(file_, 20);
    const auto directory = peek<std::uint64_t>(file_, 24);
    if (directory < kHeaderBytes || directory > size || std::uint64_t(count) * kEntryBytes != size - directory)
        throw std::runtime_error("Invalid model assembly directory.");

    std::uint64_t previousEnd = kHeaderBytes;
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::uint64_t at = directory + std::uint64_t(i) * kEntryBytes;
        Chunk chunk;
        chunk.type = peek<std::uint32_t>(file_, at);
        chunk.schema = peek<std::uint32_t>(file_, at + 4);
        chunk.codec = peek<std::uint32_t>(file_, at + 8);
        chunk.flags = peek<std::uint32_t>(file_, at + 12);
        chunk.offset = peek<std::uint64_t>(file_, at + 16);
        chunk.stored = peek<std::uint64_t>(file_, at + 24);
        chunk.raw = peek<std::uint64_t>(file_, at + 32);
        if (chunk.offset < previousEnd || chunk.offset > directory || chunk.stored > directory - chunk.offset)
            throw std::runtime_error("Invalid model assembly entry range.");
        previousEnd = chunk.offset + chunk.stored;
        chunks_.push_back(chunk);
    }
}

std::string ChunkReader::decompress(const Chunk& chunk) const {
    if (chunk.codec != kSupportedCodec) throw std::runtime_error("A model section uses an unsupported encoding.");
    if (chunk.raw > kMaxChunkBytes) throw std::runtime_error("A model section is too large.");

    std::unique_ptr<ZSTD_DCtx, decltype(&ZSTD_freeDCtx)> context(ZSTD_createDCtx(), ZSTD_freeDCtx);
    if (!context) throw std::runtime_error("Cannot allocate the decompressor.");
    ZSTD_DCtx_setParameter(context.get(), ZSTD_d_windowLogMax, 23);

    std::string raw(static_cast<std::size_t>(chunk.raw), '\0');
    ZSTD_inBuffer input{file_.data() + chunk.offset, static_cast<std::size_t>(chunk.stored), 0};
    ZSTD_outBuffer output{raw.data(), raw.size(), 0};
    std::size_t pending = 1;
    while (pending && (input.pos < input.size || output.pos < output.size)) {
        pending = ZSTD_decompressStream(context.get(), &output, &input);
        if (ZSTD_isError(pending))
            throw std::runtime_error(std::string("Model section decompression failed: ") + ZSTD_getErrorName(pending));
        if (pending && output.pos == output.size)
            throw std::runtime_error("A model section exceeds its declared size.");
    }
    if (pending || output.pos != raw.size() || input.pos != input.size)
        throw std::runtime_error("Invalid compressed model section size.");
    return raw;
}

} // namespace pulse::mdlasm::core
