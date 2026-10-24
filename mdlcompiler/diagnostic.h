// diagnostic.h - PulseModel
//
// Warnings and errors with a source location: logged to stderr, and under -api
// also sent as @diagnostic records.

#ifndef PULSEMDL_DIAGNOSTIC_H
#define PULSEMDL_DIAGNOSTIC_H

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include "api.h"

namespace pulse::diag {

struct Location {
    std::string file; // absolute, or empty for none
    int line = 0;
    std::string path; // JSON pointer inside a .pulsemodel `file`; line is 0 then
};

// The script command running now; warnings without their own location get it.
inline Location& Current() {
    static Location location;
    return location;
}

// A .pulsemodel compiles from generated script lines: line N of `script` came
// from lines[N - 1]. Set only while such a script loads.
struct Origins {
    std::string script;
    std::vector<Location> lines;
};

inline Origins& ScriptOrigins() {
    static Origins origins;
    return origins;
}

inline Location Resolve(const Location& at) {
    const Origins& origins = ScriptOrigins();
    if (!at.path.empty() || origins.lines.empty() || at.file != origins.script) return at;
    if (at.line <= 0 || at.line > static_cast<int>(origins.lines.size())) return {at.file, 0, ""};
    return origins.lines[static_cast<size_t>(at.line - 1)];
}

// "file(line)" for a script line, "file: /json/pointer" for a .pulsemodel value.
inline std::string Where(const Location& at, const std::string& file) {
    if (!at.path.empty()) return file + ": " + at.path;
    return at.line > 0 ? file + "(" + std::to_string(at.line) + ")" : file;
}

inline void Record(const char* severity, const Location& at, const std::string& text) {
    if (!pulse::api::Active()) return;
    std::string path;
    if (!at.path.empty()) path = ",\"path\":" + pulse::api::String(at.path);
    pulse::api::Record("diagnostic", std::string("{\"severity\":\"") + severity +
        "\",\"file\":" + pulse::api::String(at.file) + ",\"line\":" + std::to_string(at.line) +
        path + ",\"text\":" + pulse::api::String(text) + "}");
}

inline void WarnAt(const Location& origin, const std::string& text) {
    const Location at = Resolve(origin);
    if (at.file.empty()) std::fprintf(stderr, "warning: %s\n", text.c_str());
    else std::fprintf(stderr, "warning: %s: %s\n", Where(at, at.file).c_str(), text.c_str());
    Record("warning", at, text);
}

// printf-style; a trailing newline in the format is dropped.
inline std::string Format(const char* format, va_list args) {
    va_list copy;
    va_copy(copy, args);
    const int length = std::vsnprintf(nullptr, 0, format, copy);
    va_end(copy);

    std::string text(length > 0 ? static_cast<size_t>(length) : 0, '\0');
    if (length > 0) std::vsnprintf(text.data(), text.size() + 1, format, args);
    if (!text.empty() && text.back() == '\n') text.pop_back();
    return text;
}

inline void WarnAtf(const Location& at, const char* format, ...) {
    va_list args;
    va_start(args, format);
    const std::string text = Format(format, args);
    va_end(args);
    WarnAt(at, text);
}

inline void Warnf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    const std::string text = Format(format, args);
    va_end(args);
    WarnAt(Current(), text);
}

// The fatal error's location, kept with the exact message it produced so a later
// unrelated error is never reported at a stale location.
struct Failure {
    std::string message, text;
    Location at;
};

inline Failure& LastFailure() {
    static Failure failure;
    return failure;
}

inline void NoteFailure(const std::string& message, const Location& at, const std::string& text) {
    LastFailure() = {message, text, Resolve(at)};
}

inline void RecordFatal(const std::string& message) {
    const Failure& failure = LastFailure();
    if (failure.message == message) Record("error", failure.at, failure.text);
    else Record("error", {}, message);
}

} // namespace pulse::diag

#endif
