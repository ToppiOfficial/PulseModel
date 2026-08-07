// Fatal-error footer shared by mdlcompiler and mdldecompiler: every failure exits
// through Fail(), and Install() routes a crash into the same block.
#pragma once

#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
// windows.h's min/max macros would break std::min/std::max in any TU that
// includes this header
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <conio.h>
#include <windows.h>
#endif

namespace pulse {
namespace fatal {

// What the tool is doing right now, so a crash or a stray C++ exception can name
// the stage it died in. Updated at each pipeline step.
inline const char* g_stage = "startup";

// -pause: hold the window open at exit, so a drag-and-drop run stays readable.
inline bool g_pause = false;

inline void PauseIfAsked() {
    if (!g_pause)
        return;
    g_pause = false; // a crash after the normal footer must not ask twice
    std::printf("Press any key to exit...");
    std::fflush(stdout);
#ifdef _WIN32
    (void)_getch();
#else
    (void)std::getchar();
#endif
    std::printf("\n");
}

// A spaced footer block so the reason stands out at the bottom of a long log.
inline int Fail(const char* what, const std::string& detail) {
    std::printf("\n\n");
    std::printf("----------------------------------------\n");
    std::printf("FAILED: %s (stage: %s)\n", what, g_stage);
    if (!detail.empty())
        std::printf("%s\n", detail.c_str());
    std::printf("----------------------------------------\n\n");
    return 1;
}

#ifdef _WIN32
inline const char* SehName(DWORD code) {
    switch (code) {
        case EXCEPTION_ACCESS_VIOLATION: return "access violation (bad pointer)";
        case EXCEPTION_STACK_OVERFLOW: return "stack overflow (runaway recursion)";
        case EXCEPTION_IN_PAGE_ERROR: return "in-page error (memory/file unreadable)";
        case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "array bounds exceeded";
        case EXCEPTION_INT_DIVIDE_BY_ZERO: return "integer divide by zero";
        case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "float divide by zero";
        case EXCEPTION_ILLEGAL_INSTRUCTION: return "illegal instruction";
        case EXCEPTION_PRIV_INSTRUCTION: return "privileged instruction";
        case EXCEPTION_DATATYPE_MISALIGNMENT: return "datatype misalignment";
        case 0xC0000017: return "out of memory";           // STATUS_NO_MEMORY
        case 0xC0000135: return "required DLL not found";  // STATUS_DLL_NOT_FOUND
        case 0xC0000139: return "DLL entry point not found";
        case 0xC0000142: return "DLL initialization failed";
        default: return "unhandled exception";
    }
}

// Which module (exe or dll) the faulting address lives in, plus its offset -
// enough to point at the culprit without shipping a symbol handler.
inline std::string FaultModule(void* addr) {
    HMODULE mod = nullptr;
    char path[MAX_PATH] = {};
    if (addr &&
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                               GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           static_cast<LPCSTR>(addr), &mod) &&
        GetModuleFileNameA(mod, path, MAX_PATH)) {
        const char* base = std::strrchr(path, '\\');
        char off[32];
        std::snprintf(off, sizeof off, "+0x%llx",
                      static_cast<unsigned long long>(static_cast<char*>(addr) -
                                                      reinterpret_cast<char*>(mod)));
        return std::string(base ? base + 1 : path) + off;
    }
    return "unknown module";
}

// 16 bytes at the faulting instruction. ReadProcessMemory on ourselves fails
// instead of faulting again, so an unmapped address is safe to ask about.
inline std::string HexAt(void* addr) {
    unsigned char b[16];
    SIZE_T got = 0;
    if (!addr || !ReadProcessMemory(GetCurrentProcess(), addr, b, sizeof b, &got) || got == 0)
        return "<unreadable>";
    std::string out;
    for (SIZE_T i = 0; i < got; ++i) {
        char h[4];
        std::snprintf(h, sizeof h, "%02X ", b[i]);
        out += h;
    }
    return out;
}

inline LONG WINAPI CrashFooter(EXCEPTION_POINTERS* ep) {
    const EXCEPTION_RECORD* rec = ep->ExceptionRecord;
    std::string detail = SehName(rec->ExceptionCode);
    char buf[320];
    std::snprintf(buf, sizeof buf, " (0x%08lX)\nin %s at %p",
                  static_cast<unsigned long>(rec->ExceptionCode),
                  FaultModule(rec->ExceptionAddress).c_str(), rec->ExceptionAddress);
    detail += buf;
    if (rec->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && rec->NumberParameters >= 2) {
        static const char* kOp[] = {"reading", "writing", "executing"};
        const ULONG_PTR op = rec->ExceptionInformation[0];
        std::snprintf(buf, sizeof buf, "\nwhile %s address 0x%llx",
                      op <= 8 ? kOp[op == 8 ? 2 : op] : "accessing",
                      static_cast<unsigned long long>(rec->ExceptionInformation[1]));
        detail += buf;
    }
    detail += "\ncode bytes: " + HexAt(rec->ExceptionAddress);
    // PE TimeDateStamp+SizeOfImage: the same build id a debugger matches
    // symbols on, so a pasted footer identifies which binary crashed.
    if (const HMODULE self = GetModuleHandleA(nullptr)) {
        const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(self);
        const auto* nt =
            reinterpret_cast<const IMAGE_NT_HEADERS*>(reinterpret_cast<const char*>(self) +
                                                      dos->e_lfanew);
        std::snprintf(buf, sizeof buf, "\nbuild id: %08lX%lx (base %p)",
                      static_cast<unsigned long>(nt->FileHeader.TimeDateStamp),
                      static_cast<unsigned long>(nt->OptionalHeader.SizeOfImage),
                      static_cast<void*>(self));
        detail += buf;
    }
    Fail("crashed", detail);
    // the filter terminates the process, so main never gets to pause
    PauseIfAsked();
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

inline void Install() {
#ifdef _WIN32
    SetUnhandledExceptionFilter(CrashFooter);
#endif
}

} // namespace fatal
} // namespace pulse
