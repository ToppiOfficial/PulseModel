// PulseMDL - Source engine model compiler (.mdl/.vvd/.vtx/.phy).
//
// Usage:
//   pulsemdl <file.pulseqc> [-game <dir>]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <new>
#include <string>

#include "compile.h"
#include "pulselimits.h"
#include "qcloader.h"
#include "writer.h"

#ifdef _WIN32
#include <windows.h>
#endif

// Defined by CMake from PROJECT_VERSION, same value the .exe version resource gets.
static constexpr const char* kAppVersion = PULSEMDL2_VERSION;

// What the compiler is doing right now, so a crash or a stray C++ exception can
// name the stage it died in. Updated at each pipeline step.
static const char* g_stage = "startup";

// Every failure exits through here: a spaced footer block so the reason stands
// out at the bottom of a long compile log.
static int Fail(const char* what, const std::string& detail) {
    std::printf("\n\n");
    std::printf("----------------------------------------\n");
    std::printf("FAILED: %s (stage: %s)\n", what, g_stage);
    if (!detail.empty())
        std::printf("%s\n", detail.c_str());
    std::printf("----------------------------------------\n\n");
    return 1;
}

#ifdef _WIN32
static const char* SehName(DWORD code) {
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
static std::string FaultModule(void* addr) {
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
static std::string HexAt(void* addr) {
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

static LONG WINAPI CrashFooter(EXCEPTION_POINTERS* ep) {
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
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

static void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("PulseMDL2\n");
    std::printf("version:   %s (model version %d)\n", kAppVersion, pulse::limits::kStudioVersion);
    std::printf("developer: Toppi (MIT License)\n");
    std::printf("-------------------------------\n");
}

static int Usage() {
    std::printf("usage: pulsemdl <file.pulseqc> [-game <dir>]   (.qc accepted)\n");
    std::printf("\n");
    std::printf("  -game <dir>   mod dir to install into; output goes to\n");
    std::printf("                <dir>\\models\\<modelname>.mdl (-outdir is a synonym)\n");
    std::printf("  -defvar <name> <value>\n");
    std::printf("                define a .pulseqc script variable ($name$) before the\n");
    std::printf("                script runs; repeatable. The script cannot override it\n");
    std::printf("  -vtxformat <0|1>\n");
    std::printf("                .vtx layout, overriding the script's $vtxformat.\n");
    std::printf("                0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW)\n");
    return 1;
}

static int RunCompile(int argc, char** argv) {
    if (argc < 2)
        return Usage();

    g_stage = "command line";
    const char* script = nullptr;
    std::string outdir;
    int vtxFormat = -1; // unset; otherwise wins over the script's $vtxformat
    pulse::loader::ScriptVars defvars;
    for (int i = 1; i < argc; ++i) {
        // -game is studiomdl's name for it: the mod dir the model installs
        // into. The writer already roots output at <dir>/models, so it is the
        // same value as -outdir.
        if ((std::strcmp(argv[i], "-game") == 0 || std::strcmp(argv[i], "-outdir") == 0) &&
            i + 1 < argc) {
            outdir = argv[++i];
        } else if (std::strcmp(argv[i], "-defvar") == 0) {
            // a .pulseqc $definevariable, set from the launch line and pinned
            if (i + 2 >= argc)
                return Fail("bad option", "-defvar needs two arguments: <name> <value>");
            defvars.emplace_back(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (std::strcmp(argv[i], "-vtxformat") == 0 && i + 1 < argc) {
            vtxFormat = std::atoi(argv[++i]);
            if (vtxFormat != 0 && vtxFormat != 1)
                return Fail("bad option", std::string("-vtxformat must be 0 or 1, got \"") +
                                              argv[i] + "\"");
        } else if (argv[i][0] == '-') {
            // studiomdl GUI front ends pass switches we do not have (-nop4,
            // -verbose, ...). Warn and carry on rather than refusing to run.
            std::printf("warning: ignoring unknown option: %s\n", argv[i]);
        } else if (!script || pulse::loader::IsQcScriptPath(argv[i])) {
            // an ignored option may have eaten a value we now see as
            // positional, so a real script extension always wins
            script = argv[i];
        }
    }
    if (!script)
        return Usage();

    std::printf("Compiling: %s\n", script);

    using Clock = std::chrono::steady_clock;
    const bool timing = std::getenv("PULSEMDL_TIMING") != nullptr;
    auto t0 = Clock::now();
    auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };

    // .pulseqc (keyvalues1) is the only front end. The keyvalues2 .pulsemdl
    // loader is halted indefinitely - pulseloader.cpp is out of the build.
    if (!pulse::loader::IsQcScriptPath(script))
        return Fail("unsupported script",
                    std::string(script) + ": only .pulseqc/.qc compile scripts are supported");

    std::string err;
    pulse::compile::CompileInput input;
    g_stage = "script load";
    if (!pulse::loader::LoadQcScript(script, input, &err, defvars))
        return Fail("script error", err);
    auto tLoad = Clock::now();
    if (vtxFormat >= 0)
        input.vtxArchetype = vtxFormat;

    pulse::compile::CompiledModel model;
    g_stage = "compile";
    if (!pulse::compile::Compile(input, model, &err))
        return Fail("compile error", err);

    auto tCompile = Clock::now();

    g_stage = "write";
    if (!pulse::writer::WriteModelFiles(model, outdir,
                                        /*legacyVtx=*/input.vtxArchetype == 0, &err))
        return Fail("write error", err);
    auto tWrite = Clock::now();
    g_stage = "done";

    std::printf("compile time: %.2f s\n", ms(t0, tWrite) / 1000.0);

    if (timing) {
        std::printf("[timing] load %.0f ms | compile %.0f ms | write %.0f ms | total %.0f ms\n",
                    ms(t0, tLoad), ms(tLoad, tCompile), ms(tCompile, tWrite), ms(t0, tWrite));
    }
    return 0;
}

int main(int argc, char** argv) {
    // progress lines are useless if they sit in the CRT buffer until exit -
    // MSVC has no line buffering (_IOLBF == _IOFBF), so go unbuffered
    std::setvbuf(stdout, nullptr, _IONBF, 0);

#ifdef _WIN32
    SetUnhandledExceptionFilter(CrashFooter);
#endif

    PrintHeader();

    // a leak past a stage's own error path still gets named in the footer
    try {
        return RunCompile(argc, argv);
    } catch (const std::bad_alloc&) {
        return Fail("out of memory", "an allocation failed - the model may exceed available RAM");
    } catch (const std::exception& e) {
        return Fail("internal error", e.what());
    } catch (...) {
        return Fail("internal error", "unknown C++ exception");
    }
}
