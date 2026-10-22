#include "apiprotocol.h"

#include <cstdio>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

namespace pulse::api {
namespace {
std::FILE* output = nullptr;
}

bool Active() { return output != nullptr; }

void Begin(const char* tool) {
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
    Record("api", tool ? "{\"version\":" + std::to_string(kVersion) + ",\"tool\":" + String(tool) + "}"
                : "{\"version\":" + std::to_string(kVersion) + "}");
}

nlohmann::json ReadRequest(size_t maxBytes) {
    std::string bytes;
    char chunk[65536];
    size_t count;
    while ((count = std::fread(chunk, 1, sizeof(chunk), stdin)) != 0) {
        if (count > maxBytes - bytes.size())
            throw std::runtime_error("API request exceeds " + std::to_string(maxBytes >> 20) + " MiB");
        bytes.append(chunk, count);
    }
    if (std::ferror(stdin)) throw std::runtime_error("Cannot read API request");

    nlohmann::json request;
    try { request = nlohmann::json::parse(bytes); }
    catch (const nlohmann::json::parse_error& error) {
        throw std::runtime_error(std::string("Invalid API request JSON: ") + error.what());
    }

    if (!request.is_object()) throw std::runtime_error("API request must be a JSON object");
    const auto version = request.find("version");
    if (version == request.end() || !version->is_number_integer() || version->get<int>() != kVersion)
        throw std::runtime_error("Unsupported API version, expected " + std::to_string(kVersion));
    GetString(request, "method");
    return request;
}

void CheckMembers(const nlohmann::json& request, std::initializer_list<const char*> allowed) {
    for (const auto& member : request.items()) {
        if (member.key() == "version" || member.key() == "method") continue;
        bool known = false;
        for (const char* name : allowed) known |= member.key() == name;
        if (!known) throw std::runtime_error("Unknown API member \"" + member.key() + "\"");
    }
}

std::string GetString(const nlohmann::json& request, const char* name) {
    const auto value = request.find(name);
    if (value == request.end()) return {};
    if (!value->is_string()) throw std::runtime_error(std::string("API member \"") + name + "\" must be a string");
    return value->get<std::string>();
}

std::vector<std::string> GetStrings(const nlohmann::json& request, const char* name) {
    const auto value = request.find(name);
    if (value == request.end()) return {};

    const auto invalid = std::string("API member \"") + name + "\" must be an array of strings";
    if (!value->is_array()) throw std::runtime_error(invalid);
    std::vector<std::string> strings;
    for (const auto& item : *value) {
        if (!item.is_string()) throw std::runtime_error(invalid);
        strings.push_back(item.get<std::string>());
    }
    return strings;
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
