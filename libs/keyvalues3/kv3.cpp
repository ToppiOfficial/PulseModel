#include "keyvalues3/kv3.h"

#include <algorithm>
#include <cmath>
#include <charconv>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <system_error>
#include <stdexcept>

namespace pulse::keyvalues3 {
bool Value::IsObject() const { return std::holds_alternative<Object>(data); }
bool Value::IsArray() const { return std::holds_alternative<Array>(data); }
bool Value::Has(const std::string& key) const { return IsObject() && Members().count(key) != 0; }
Value::Object& Value::Members() { return std::get<Object>(data); }
const Value::Object& Value::Members() const { return std::get<Object>(data); }
Value::Array& Value::Items() { return std::get<Array>(data); }
const Value::Array& Value::Items() const { return std::get<Array>(data); }
Value& Value::operator[](const std::string& key) { return Members()[key]; }
const Value& Value::At(const std::string& key) const { auto it = Members().find(key); if (it == Members().end()) throw std::runtime_error("Missing KV3 field: " + key); return it->second; }
std::string Value::String() const { return std::get<std::string>(data); }
double Value::Number() const {
    if (auto v = std::get_if<double>(&data)) return *v;
    if (auto v = std::get_if<int64_t>(&data)) return static_cast<double>(*v);
    if (auto v = std::get_if<uint64_t>(&data)) return static_cast<double>(*v);
    throw std::runtime_error("Expected KV3 number");
}
int64_t Value::Integer() const {
    if (auto v = std::get_if<int64_t>(&data)) return *v;
    if (auto v = std::get_if<uint64_t>(&data)) { if (*v > static_cast<uint64_t>(INT64_MAX)) throw std::runtime_error("KV3 integer overflow"); return static_cast<int64_t>(*v); }
    const double d = Number(); if (!std::isfinite(d) || d < -9223372036854775808.0 || d >= 9223372036854775808.0 || std::floor(d) != d) throw std::runtime_error("Invalid KV3 integer"); return static_cast<int64_t>(d);
}
bool Value::Boolean() const { return std::get<bool>(data); }
std::vector<uint8_t> ReadFile(const std::string& path) {
    std::ifstream file(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Cannot read: " + path);
    const auto length = file.tellg(); if (length < 0 || length > 1073741824) throw std::runtime_error("Invalid or oversized file: " + path);
    std::vector<uint8_t> out(static_cast<size_t>(length)); file.seekg(0);
    if (!out.empty() && !file.read(reinterpret_cast<char*>(out.data()), length)) throw std::runtime_error("Truncated file: " + path);
    return out;
}
void WriteFile(const std::string& path, const std::vector<uint8_t>& data) {
    const auto dest = std::filesystem::u8path(path); std::filesystem::create_directories(dest.parent_path());
    std::ofstream file(dest, std::ios::binary); if (!file) throw std::runtime_error("Cannot write: " + path);
    if (!data.empty()) file.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    file.close(); if (!file) throw std::runtime_error("Failed writing: " + path);
}
void WriteFile(const std::string& path, const std::string& data) { WriteFile(path, std::vector<uint8_t>(data.begin(), data.end())); }
std::string Quote(const std::string& text) {
    std::string out = "\""; const char* hex = "0123456789abcdef";
    for (unsigned char c : text) {
        if (c == '\\' || c == '"') { out += '\\'; out += static_cast<char>(c); }
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
        else out += static_cast<char>(c);
    }
    return out + '"';
}
namespace {
class TextParser {
public:
    explicit TextParser(const std::string& s) : text(s) {}
    Value Document() { auto v = Read(0); Skip(); if (pos != text.size()) Fail("Unexpected trailing text"); return v; }
private:
    const std::string& text; size_t pos = 0;
    [[noreturn]] void Fail(const std::string& message) const { throw std::runtime_error("KV3 text at " + std::to_string(pos) + ": " + message); }
    void Skip() {
        for (;;) {
            while (pos < text.size() && (static_cast<unsigned char>(text[pos]) <= 32 || text[pos] == ',')) ++pos;
            if (pos == 0 && text.compare(0, 3, "\xef\xbb\xbf") == 0) { pos = 3; continue; }
            if (text.compare(pos, 4, "<!--") == 0) { const auto end = text.find("-->", pos + 4); if (end == std::string::npos) Fail("Unterminated header"); pos = end + 3; }
            else if (text.compare(pos, 2, "/*") == 0) { const auto end = text.find("*/", pos + 2); if (end == std::string::npos) Fail("Unterminated comment"); pos = end + 2; }
            else if (text.compare(pos, 2, "//") == 0) { const auto end = text.find('\n', pos + 2); pos = end == std::string::npos ? text.size() : end + 1; }
            else return;
        }
    }
    void Require(char c) { Skip(); if (pos >= text.size() || text[pos++] != c) Fail(std::string("Expected ") + c); }
    void Utf8(std::string& out, unsigned value) {
        if (value < 128) out += static_cast<char>(value);
        else if (value < 2048) { out += static_cast<char>(192 | value >> 6); out += static_cast<char>(128 | (value & 63)); }
        else if (value < 65536) { out += static_cast<char>(224 | value >> 12); out += static_cast<char>(128 | ((value >> 6) & 63)); out += static_cast<char>(128 | (value & 63)); }
        else { out += static_cast<char>(240 | value >> 18); out += static_cast<char>(128 | ((value >> 12) & 63)); out += static_cast<char>(128 | ((value >> 6) & 63)); out += static_cast<char>(128 | (value & 63)); }
    }
    unsigned Hex4() { unsigned v = 0; for (int i = 0; i < 4; ++i) { if (pos == text.size()) Fail("Truncated unicode escape"); const char c = text[pos++]; int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; if (digit < 0) Fail("Invalid unicode escape"); v = v * 16 + static_cast<unsigned>(digit); } return v; }
    std::string Token() {
        Skip(); if (pos == text.size()) Fail("Unexpected end");
        if (text[pos] == '"') {
            ++pos; std::string out;
            while (pos < text.size()) {
                char c = text[pos++]; if (c == '"') return out;
                if (c != '\\') { out += c; continue; }
                if (pos == text.size()) Fail("Truncated string"); c = text[pos++];
                switch (c) {
                    case 'n': out += '\n'; break; case 'r': out += '\r'; break; case 't': out += '\t'; break;
                    case 'b': out += '\b'; break; case 'f': out += '\f'; break;
                    case '"': case '\\': case '/': out += c; break;
                    case 'u': { unsigned v = Hex4(); if (v >= 0xd800 && v <= 0xdbff) { if (text.compare(pos, 2, "\\u") != 0) Fail("Missing unicode surrogate"); pos += 2; const unsigned tail = Hex4(); if (tail < 0xdc00 || tail > 0xdfff) Fail("Invalid unicode surrogate"); v = 0x10000 + ((v - 0xd800) << 10) + tail - 0xdc00; } else if (v >= 0xdc00 && v <= 0xdfff) Fail("Unexpected unicode surrogate"); Utf8(out, v); break; }
                    default: Fail("Invalid string escape");
                }
            }
            Fail("Unterminated string");
        }
        const auto start = pos;
        while (pos < text.size() && static_cast<unsigned char>(text[pos]) > 32 && std::string("{}[]=,:").find(text[pos]) == std::string::npos) ++pos;
        if (pos == start) Fail("Expected token"); return text.substr(start, pos - start);
    }
    Value Read(size_t depth) {
        if (depth > 256) Fail("Nesting limit exceeded"); Skip(); if (pos == text.size()) Fail("Unexpected end");
        if (text[pos] == '{') { ++pos; Value::Object obj; Skip(); while (pos < text.size() && text[pos] != '}') { const auto key = Token(); Require('='); obj[key] = Read(depth + 1); Skip(); } Require('}'); return obj; }
        if (text[pos] == '[') { ++pos; Value::Array items; Skip(); while (pos < text.size() && text[pos] != ']') { items.push_back(Read(depth + 1)); Skip(); } Require(']'); return items; }
        if (text[pos] == '"') return Token();
        const auto token = Token(); Skip(); if (pos < text.size() && text[pos] == ':') { ++pos; auto v = Read(depth + 1); v.flag = token; return v; }
        if (token == "true") return true; if (token == "false") return false; if (token == "null") return nullptr;
        if (token.find_first_of(".eE") == std::string::npos) {
            int64_t signedValue; auto signedResult = std::from_chars(token.data(),token.data()+token.size(),signedValue);
            if (signedResult.ec == std::errc{} && signedResult.ptr == token.data()+token.size()) return signedValue;
            uint64_t unsignedValue; auto unsignedResult = std::from_chars(token.data(),token.data()+token.size(),unsignedValue);
            if (unsignedResult.ec == std::errc{} && unsignedResult.ptr == token.data()+token.size()) return unsignedValue;
            Fail("Invalid integer " + token);
        }
        std::istringstream stream(token); stream.imbue(std::locale::classic()); double number; stream >> number;
        if (!stream.fail() && stream.eof() && std::isfinite(number)) return number;
        Fail("Unsupported token " + token);
    }
};
}
Value ParseText(const std::string& text) { return TextParser(text).Document(); }
std::string TextBody(const Value& v, size_t depth) {
    if (depth > 256) throw std::runtime_error("KV3 nesting limit exceeded");
    const auto pad = std::string(depth, '\t'); std::string out;
    if (!v.flag.empty()) { Value plain = v; plain.flag.clear(); return v.flag + ":" + TextBody(plain, depth); }
    if (v.IsObject()) { out = "{\n"; for (const auto& kv : v.Members()) out += pad + '\t' + Quote(kv.first) + " = " + TextBody(kv.second, depth + 1) + '\n'; return out + pad + '}'; }
    if (v.IsArray()) { out = "[\n"; for (const auto& item : v.Items()) out += pad + '\t' + TextBody(item, depth + 1) + ",\n"; return out + pad + ']'; }
    if (auto s = std::get_if<std::string>(&v.data)) return Quote(*s);
    if (auto b = std::get_if<bool>(&v.data)) return *b ? "true" : "false";
    if (std::holds_alternative<std::nullptr_t>(v.data)) return "null";
    if (auto i = std::get_if<int64_t>(&v.data)) return std::to_string(*i);
    if (auto i = std::get_if<uint64_t>(&v.data)) return std::to_string(*i);
    // a double always keeps a decimal point, so it reads back as a double, not an int
    if (auto d = std::get_if<double>(&v.data)) { if (!std::isfinite(*d)) throw std::runtime_error("Nonfinite KV3 value"); std::ostringstream s; s.imbue(std::locale::classic()); s << std::setprecision(17) << *d; auto out = s.str(); if (out.find_first_of(".e") == std::string::npos) out += ".0"; return out; }
    throw std::runtime_error("Binary KV3 blobs cannot be written to text by this API");
}
std::string WriteText(const Value& value) { return "<!-- kv3 encoding:text:version{e21c7f3c-8a33-41c5-9977-a76d3a32aa0d} format:generic:version{7412167c-06e9-4698-aff2-e63eb59037e7} -->\n" + TextBody(value) + '\n'; }
std::vector<uint8_t> ResourceBlock(const std::vector<uint8_t>& file, const std::string& name) {
    auto u32 = [&](size_t at) { if (at > file.size() || file.size() - at < 4) throw std::runtime_error("Truncated Source 2 resource header"); return static_cast<uint32_t>(file[at]) | (static_cast<uint32_t>(file[at + 1]) << 8) | (static_cast<uint32_t>(file[at + 2]) << 16) | (static_cast<uint32_t>(file[at + 3]) << 24); };
    if (file.size() < 16 || file[4] != 12 || file[5] != 0) throw std::runtime_error("Unsupported Source 2 resource header");
    const uint64_t table = static_cast<uint64_t>(8) + u32(8); const auto count = u32(12);
    if (count > 10000 || table > file.size() || static_cast<uint64_t>(count) * 12 > file.size() - table) throw std::runtime_error("Invalid Source 2 block directory");
    for (uint32_t i = 0; i < count; ++i) {
        const auto at = static_cast<size_t>(table) + i * 12;
        if (name.size() != 4 || std::memcmp(file.data() + at, name.data(), 4) != 0) continue;
        const uint64_t start = static_cast<uint64_t>(at) + 4 + u32(at + 4); const auto size = u32(at + 8);
        if (start > file.size() || size > file.size() - start) throw std::runtime_error("Invalid Source 2 block range");
        return {file.begin() + static_cast<ptrdiff_t>(start), file.begin() + static_cast<ptrdiff_t>(start + size)};
    }
    throw std::runtime_error("Source 2 resource lacks " + name + " block");
}
Value ReadResource(const std::vector<uint8_t>& file, const std::string& block) { return ParseBinary(ResourceBlock(file, block)); }
}
