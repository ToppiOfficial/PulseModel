#include "api.h"

#include <cstdio>
#include <stdexcept>
#include <utility>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

namespace pulse::api {
namespace {
std::FILE* output = nullptr;
constexpr size_t maxRequest = 64 * 1024 * 1024;
}

bool Active() { return output != nullptr; }

void Begin() {
    std::fflush(stdout);
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
    const int descriptor = _dup(_fileno(stdout));
    if (descriptor < 0) throw std::runtime_error("Cannot open API output");
    output = _fdopen(descriptor, "wb");
    if (!output) { _close(descriptor); throw std::runtime_error("Cannot open API stream"); }
    if (_dup2(_fileno(stderr), _fileno(stdout)) != 0)
        throw std::runtime_error("Cannot redirect compiler log");
#else
    const int descriptor = dup(fileno(stdout));
    if (descriptor < 0) throw std::runtime_error("Cannot open API output");
    output = fdopen(descriptor, "wb");
    if (!output) { close(descriptor); throw std::runtime_error("Cannot open API stream"); }
    if (dup2(fileno(stderr), fileno(stdout)) < 0)
        throw std::runtime_error("Cannot redirect compiler log");
#endif
    std::setvbuf(output, nullptr, _IOFBF, 1 << 16);
    Record("api", "{\"version\":1}");
}

Request ReadRequest() {
    std::string bytes;
    char chunk[65536];
    size_t count;
    while ((count = std::fread(chunk, 1, sizeof(chunk), stdin)) != 0) {
        if (count > maxRequest - bytes.size()) throw std::runtime_error("API request exceeds 64 MiB");
        bytes.append(chunk, count);
    }
    if (std::ferror(stdin)) throw std::runtime_error("Cannot read API request");
    size_t cursor = 0;
    const auto line = [&]() {
        const auto end = bytes.find('\n', cursor);
        if (end == std::string::npos) throw std::runtime_error("Truncated API header");
        auto value = bytes.substr(cursor, end - cursor);
        cursor = end + 1;
        return value;
    };
    const auto number = [&]() {
        const auto value = line();
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
            throw std::runtime_error("Invalid API length");
        size_t used = 0;
        const auto length = std::stoull(value, &used);
        if (used != value.size() || length > maxRequest) throw std::runtime_error("Invalid API length");
        return static_cast<size_t>(length);
    };
    const auto field = [&]() {
        const size_t length = number();
        if (length > bytes.size() - cursor) throw std::runtime_error("Truncated API field");
        auto value = bytes.substr(cursor, length);
        cursor += length;
        return value;
    };
    if (line() != "PULSEMODEL 1") throw std::runtime_error("Unsupported API version");
    Request request;
    request.method = line();
    if (request.method != "catalog" && request.method != "analyze" && request.method != "compile" &&
        request.method != "procedural" && request.method != "materials")
        throw std::runtime_error("Unknown API method");
    const size_t arguments = number();
    if (arguments > 4096) throw std::runtime_error("Too many API arguments");
    for (size_t i = 0; i < arguments; ++i) {
        auto value = field();
        if (value.find('\0') != std::string::npos) throw std::runtime_error("NUL in API argument");
        request.arguments.push_back(std::move(value));
    }
    request.buffers = field();
    if (cursor != bytes.size()) throw std::runtime_error("Trailing API request bytes");
    if (request.method == "catalog" && (!request.arguments.empty() || !request.buffers.empty()))
        throw std::runtime_error("Catalog takes no arguments or buffers");
    if (request.method != "analyze" && !request.buffers.empty())
        throw std::runtime_error("This API method reads saved files");
    return request;
}

std::string String(const std::string& value) {
    std::string out = "\"";
    for (size_t i = 0; i < value.size(); ++i) {
        const auto ch = static_cast<unsigned char>(value[i]);
        if (ch >= 128) {
            const size_t width = ch >= 0xC2 && ch <= 0xDF ? 2 : ch >= 0xE0 && ch <= 0xEF ? 3 :
                ch >= 0xF0 && ch <= 0xF4 ? 4 : 0;
            bool valid = width && width <= value.size() - i;
            for (size_t j = 1; valid && j < width; ++j)
                valid = (static_cast<unsigned char>(value[i + j]) & 0xC0) == 0x80;
            if (valid && width > 2) {
                const auto next = static_cast<unsigned char>(value[i + 1]);
                valid = !(ch == 0xE0 && next < 0xA0) && !(ch == 0xED && next >= 0xA0) &&
                    !(ch == 0xF0 && next < 0x90) && !(ch == 0xF4 && next >= 0x90);
            }
            if (valid) { out.append(value, i, width); i += width - 1; }
            else out += "\\ufffd";
            continue;
        }
        switch (ch) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (ch < 32) {
                const char* hex = "0123456789abcdef";
                out += "\\u00";
                out += hex[ch >> 4];
                out += hex[ch & 15];
            } else out += static_cast<char>(ch);
        }
    }
    return out + "\"";
}

void Line(const std::string& value) {
    std::FILE* stream = output ? output : stdout;
    if (std::fwrite(value.data(), 1, value.size(), stream) != value.size() || std::fputc('\n', stream) == EOF)
        throw std::runtime_error("Cannot write API response");
}

void Record(const char* tag, const std::string& json) { Line(std::string("@") + tag + " " + json); }

void Finish(int exitCode) {
    Record("result", "{\"exitCode\":" + std::to_string(exitCode) + "}");
    if (std::fflush(output) != 0) throw std::runtime_error("Cannot flush API response");
}

}
