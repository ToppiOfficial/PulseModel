#include "keyvalues3/kv3.h"
#include "zstd/zstd.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pulse::keyvalues3 {
namespace {
constexpr size_t Limit = 256 * 1024 * 1024;
class Cursor {
public:
    Cursor() = default;
    Cursor(const uint8_t* data, size_t size) : bytes(data), length(size) {}
    explicit Cursor(const std::vector<uint8_t>& data) : Cursor(data.data(), data.size()) {}
    size_t Remaining() const { return length - pos; }
    size_t Position() const { return pos; }
    uint8_t U8() { Need(1); return bytes[pos++]; }
    uint16_t U16() { uint16_t n = U8(); return static_cast<uint16_t>(n | static_cast<uint16_t>(U8()) << 8); }
    uint32_t U32() { uint32_t n = 0; for (int i = 0; i < 4; ++i) n |= static_cast<uint32_t>(U8()) << (8 * i); return n; }
    uint64_t U64() { uint64_t n = 0; for (int i = 0; i < 8; ++i) n |= static_cast<uint64_t>(U8()) << (8 * i); return n; }
    int32_t I32() { const auto n = U32(); int32_t v; std::memcpy(&v, &n, 4); return v; }
    float Float() { const auto n = U32(); float v; std::memcpy(&v, &n, 4); return v; }
    double Double() { const auto n = U64(); double v; std::memcpy(&v, &n, 8); return v; }
    size_t Count() { const auto n = I32(); if (n < 0 || static_cast<size_t>(n) > Limit) throw std::runtime_error("Invalid binary KV3 count"); return static_cast<size_t>(n); }
    Cursor Take(size_t size) { Need(size); Cursor out(bytes ? bytes + pos : nullptr, size); pos += size; return out; }
    std::vector<uint8_t> Vector(size_t size) { auto slice = Take(size); if (!size) return {}; return {slice.bytes, slice.bytes + size}; }
    void Align(size_t alignment) { const auto add = (alignment - pos % alignment) % alignment; Take(add); }
    std::string CString() { std::string text; while (Remaining()) { const auto c = U8(); if (!c) return text; text += static_cast<char>(c); } throw std::runtime_error("Unterminated binary KV3 string"); }
private:
    const uint8_t* bytes = nullptr; size_t length = 0; size_t pos = 0;
    void Need(size_t n) const { if (n > Remaining()) throw std::runtime_error("Truncated binary KV3"); }
};
std::vector<uint8_t> Lz4(const std::vector<uint8_t>& input, size_t size, const std::vector<uint8_t>& history = {}) {
    if (size > Limit) throw std::runtime_error("LZ4 output exceeds limit");
    const auto prefix = std::min<size_t>(65536, history.size());
    std::vector<uint8_t> output; output.reserve(prefix + size);
    output.insert(output.end(), history.end() - static_cast<ptrdiff_t>(prefix), history.end());
    Cursor cursor(input);
    auto length = [&](size_t base) { if (base == 15) { for (;;) { const auto n = cursor.U8(); if (base > size || n > size - base) throw std::runtime_error("LZ4 length overflow"); base += n; if (n != 255) break; } } return base; };
    while (cursor.Remaining()) {
        const auto token = cursor.U8(); const auto literals = length(token >> 4);
        if (literals > size - (output.size() - prefix)) throw std::runtime_error("LZ4 literal overrun");
        const auto bytes = cursor.Vector(literals); output.insert(output.end(), bytes.begin(), bytes.end());
        if (!cursor.Remaining()) break;
        const auto distance = cursor.U16(); if (!distance || distance > output.size()) throw std::runtime_error("Invalid LZ4 match offset");
        const auto extra = length(token & 15);
        if (extra > size - (output.size() - prefix) || size - (output.size() - prefix) - extra < 4) throw std::runtime_error("LZ4 match overrun");
        for (size_t i = 0; i < extra + 4; ++i) output.push_back(output[output.size() - distance]);
    }
    if (output.size() - prefix != size) throw std::runtime_error("LZ4 decoded size mismatch");
    return {output.begin() + static_cast<ptrdiff_t>(prefix), output.end()};
}
std::vector<uint8_t> Decompress(Cursor& source, uint32_t method, size_t compressed, size_t size) {
    if (size > Limit || compressed > Limit) throw std::runtime_error("Binary KV3 decompression limit exceeded");
    if (method == 0) return source.Vector(size);
    auto input = source.Vector(compressed);
    if (method == 1) return Lz4(input, size);
    if (method != 2) throw std::runtime_error("Unsupported KV3 compression");
    std::vector<uint8_t> output(size);
    const auto result = ZSTD_decompress(output.data(), size, input.data(), input.size());
    if (ZSTD_isError(result) || result != size) throw std::runtime_error(std::string("Zstandard KV3 decompression failed: ") + ZSTD_getErrorName(result));
    return output;
}
struct Lanes { Cursor one, two, four, eight; };
Lanes Split(Cursor& raw, size_t one, size_t two, size_t four, size_t eight, bool alignEmpty) {
    Lanes lanes; if (one) lanes.one = raw.Take(one);
    if (two) { raw.Align(2); lanes.two = raw.Take(two * 2); }
    if (four) { raw.Align(4); lanes.four = raw.Take(four * 4); }
    if (eight) { raw.Align(8); lanes.eight = raw.Take(eight * 8); }
    else if (alignEmpty) raw.Align(8);
    return lanes;
}
class Decoder {
public:
    explicit Decoder(const std::vector<uint8_t>& input) : file(input) {}
    Value Decode() {
        const auto magic = file.U32(); version = static_cast<int>(magic & 255);
        if ((magic & 0xffffff00) != 0x4b563300 || version < 1 || version > 5) throw std::runtime_error("Supported binary KV3 versions are 1 through 5");
        file.Take(16); const auto method = file.U32();
        size_t one, four, eight, typesCount = 0, total, compressed, blocks = 0, blobSize = 0; uint16_t frame = 0;
        if (version == 1) { one = file.Count(); four = file.Count(); eight = file.Count(); total = file.Count(); compressed = file.Remaining(); }
        else {
            const auto dictionary = file.U16(); frame = file.U16(); if (dictionary != 0) throw std::runtime_error("KV3 compression dictionary is unsupported");
            one = file.Count(); four = file.Count(); eight = file.Count(); typesCount = file.Count(); file.U16(); file.U16();
            total = file.Count(); compressed = file.Count(); blocks = file.Count(); blobSize = file.Count();
        }
        size_t two = 0, frameSizesBytes = 0;
        if (version >= 4) { two = file.Count(); frameSizesBytes = file.Count(); }
        size_t u1 = total, c1 = compressed, u2 = 0, c2 = 0, one2 = 0, two2 = 0, four2 = 0, eight2 = 0, objects2 = 0;
        if (version >= 5) {
            u1 = file.Count(); c1 = file.Count(); u2 = file.Count(); c2 = file.Count();
            one2 = file.Count(); two2 = file.Count(); four2 = file.Count(); eight2 = file.Count();
            file.Count(); objects2 = file.Count(); file.Count(); file.Count();
            if (u1 > total || u2 != total - u1) throw std::runtime_error("Invalid KV3 buffer sizes");
        }
        if (method > 2 || (method == 1 && version >= 2 && frame != 16384) || (method != 1 && frame != 0)) throw std::runtime_error("Unsupported KV3 compression settings");
        buffer1 = Decompress(file, method, c1, u1 + (version < 5 && method == 2 ? blobSize : 0));
        Cursor raw1(buffer1.data(), u1); aux = Split(raw1, one, two, four, eight, version < 5);
        const auto stringCount = aux.four.Count(); if (stringCount > 1000000) throw std::runtime_error("KV3 string count exceeds limit");
        if (version >= 5) { for (size_t i = 0; i < stringCount; ++i) strings.push_back(aux.one.CString()); }
        else {
            main = aux; const auto start = raw1.Position();
            for (size_t i = 0; i < stringCount; ++i) strings.push_back(raw1.CString());
            const auto stringBytes = raw1.Position() - start;
            const auto count = version == 1 ? raw1.Remaining() - 4 : typesCount >= stringBytes ? typesCount - stringBytes : throw std::runtime_error("Invalid KV3 types length");
            types = raw1.Take(count); tail = raw1;
        }
        if (version >= 5) {
            buffer2 = Decompress(file, method, c2, u2); Cursor raw2(buffer2);
            objectLengths = raw2.Take(objects2 * 4); main = Split(raw2, one2, two2, four2, eight2, false);
            types = raw2.Take(typesCount); tail = raw2;
        }
        if (blocks) {
            blobLengths = tail.Take(blocks * 4); if (tail.U32() != 0xffeedd00) throw std::runtime_error("Invalid KV3 blob trailer");
            if (method == 0) blobs = file.Vector(blobSize);
            else if (method == 2) {
                if (version >= 5) { if (c1 > compressed || c2 > compressed - c1) throw std::runtime_error("Invalid KV3 compressed blob size"); blobs = Decompress(file, method, compressed - c1 - c2, blobSize); }
                else blobs.assign(buffer1.begin() + static_cast<ptrdiff_t>(u1), buffer1.end());
            } else {
                auto lengths = blobLengths;
                while (lengths.Remaining()) {
                    size_t remaining = lengths.Count();
                    while (remaining) {
                        const size_t decoded = std::min<size_t>(frame, remaining);
                        auto part = Lz4(file.Vector(tail.U16()), decoded, blobs);
                        if (blobs.size() > blobSize || part.size() > blobSize - blobs.size()) throw std::runtime_error("KV3 blob size overflow");
                        blobs.insert(blobs.end(), part.begin(), part.end()); remaining -= decoded;
                    }
                }
                if (blobs.size() != blobSize) throw std::runtime_error("KV3 blob size mismatch");
            }
            if (file.U32() != 0xffeedd00) throw std::runtime_error("Invalid KV3 file trailer");
        } else if (tail.U32() != 0xffeedd00) throw std::runtime_error("Invalid KV3 trailer");
        blobData = Cursor(blobs);
        auto result = Read(Type(), main, 0);
        if (types.Remaining() || objectLengths.Remaining() || blobLengths.Remaining() || blobData.Remaining()) throw std::runtime_error("Unconsumed binary KV3 data");
        return result;
    }
private:
    Cursor file, types, objectLengths, blobLengths, blobData, tail; Lanes main, aux; int version = 0; size_t nodes = 0;
    std::vector<uint8_t> buffer1, buffer2, blobs; std::vector<std::string> strings;
    std::pair<uint8_t, uint8_t> Type() {
        auto type = types.U8(); uint8_t flag = 0;
        if (type & 128) flag = types.U8();
        if (version >= 3 && (type & 64)) types.U8();
        return {static_cast<uint8_t>(type & (version >= 3 ? 63 : 127)), flag};
    }
    std::string String(int32_t index) const { if (index == -1) return {}; if (index < 0 || static_cast<size_t>(index) >= strings.size()) throw std::runtime_error("Invalid KV3 string index"); return strings[static_cast<size_t>(index)]; }
    Value Read(std::pair<uint8_t, uint8_t> tag, Lanes& lane, size_t depth) {
        if (++nodes > 2000000 || depth > 256) throw std::runtime_error("KV3 tree exceeds limit");
        Value out;
        switch (tag.first) {
            case 1: break;
            case 2: out = lane.one.U8() != 0; break;
            case 3: { auto bits = lane.eight.U64(); int64_t signedBits; std::memcpy(&signedBits, &bits, 8); out = signedBits; break; }
            case 4: out = lane.eight.U64(); break;
            case 5: out = lane.eight.Double(); break;
            case 6: out = String(main.four.I32()); break;
            case 7: { const auto size = version >= 2 ? blobLengths.Count() : main.four.Count(); out = version >= 2 ? blobData.Vector(size) : main.one.Vector(size); break; }
            case 8: { const auto size = main.four.Count(); if (size > 2000000) throw std::runtime_error("KV3 array exceeds limit"); Value::Array array; array.reserve(size); for (size_t i = 0; i < size; ++i) array.push_back(Read(Type(), main, depth + 1)); out = std::move(array); break; }
            case 9: {
                const auto size = version >= 5 ? objectLengths.Count() : main.four.Count(); if (size > 2000000) throw std::runtime_error("KV3 object exceeds limit"); Value::Object object;
                for (size_t i = 0; i < size; ++i) { const auto type = Type(); const auto name = String(main.four.I32()); object[name] = Read(type, main, depth + 1); }
                out = std::move(object); break;
            }
            case 10: case 24: case 25: {
                const auto size = tag.first == 10 ? main.four.Count() : main.one.U8(); if (size > 2000000) throw std::runtime_error("Typed KV3 array exceeds limit");
                const auto type = Type(); Value::Array array; array.reserve(size); auto& source = tag.first == 25 ? aux : main;
                for (size_t i = 0; i < size; ++i) array.push_back(Read(type, source, depth + 1)); out = std::move(array); break;
            }
            case 11: out = lane.four.I32(); break;
            case 12: out = static_cast<uint64_t>(lane.four.U32()); break;
            case 13: out = true; break; case 14: out = false; break;
            case 15: case 17: out = 0; break; case 16: case 18: out = 1; break;
            case 19: out = static_cast<double>(lane.four.Float()); break;
            case 20: out = static_cast<int>(static_cast<int16_t>(lane.two.U16())); break;
            case 21: out = static_cast<int>(lane.two.U16()); break;
            case 22: out = static_cast<int>(static_cast<int8_t>(lane.one.U8())); break;
            case 23: out = static_cast<int>(lane.one.U8()); break;
            default: throw std::runtime_error("Unsupported binary KV3 type " + std::to_string(tag.first));
        }
        if (tag.second == 1) out.flag = "resource";
        else if (tag.second == 2) out.flag = "resource_name";
        return out;
    }
};
}
Value ParseBinary(const std::vector<uint8_t>& data) { return Decoder(data).Decode(); }
}
