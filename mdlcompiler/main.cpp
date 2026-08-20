// mdlcompiler - Source engine model compiler (.mdl/.vvd/.vtx/.phy).
//
// Usage:
//   mdlcompiler <file.pulseqc> [-game <dir>] [-pause]

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <vector>

#include "compile.h"
#include "fatalerror.h"
#include "perf.h"
#include "pulselimits.h"
#include "qcloader.h"
#include "writer.h"

// Defined by CMake from TOOL_VERSION, same value the .exe version resource gets.
static constexpr const char* kAppVersion = PULSEMODEL_VERSION;

using pulse::fatal::Fail;
static const char*& g_stage = pulse::fatal::g_stage;

static void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("PulseModel [Model Compiler]\n");
    std::printf("version:   %s (model version %d)\n", kAppVersion, pulse::limits::kStudioVersion);
    std::printf("developer: Toppi\n");
    std::printf("-------------------------------\n");
}

static int Usage() {
    std::printf("usage: mdlcompiler <file.pulseqc> [-game <dir>]   (a .qc is imported first)\n");
    std::printf("\n");
    std::printf("  -game <dir>   mod dir to install into; output goes to\n");
    std::printf("                <dir>\\models\\<modelname>.mdl (-outdir is a synonym)\n");
    std::printf("  -defvar <name> <value>\n");
    std::printf("                define a .pulseqc script variable ($name$) before the\n");
    std::printf("                script runs; repeatable. The script cannot override it\n");
    std::printf("  -includesearchdir <dir>\n");
    std::printf("                extra fallback dir for $include, searched after any\n");
    std::printf("                $addincludesearchdir; repeatable\n");
    std::printf("  -filesearchdir <dir>\n");
    std::printf("                extra fallback dir for source files, searched after\n");
    std::printf("                any $addsearchdir; repeatable\n");
    std::printf("  -vtxformat <0|1>\n");
    std::printf("                .vtx layout, overriding the script's $vtxformat.\n");
    std::printf("                0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW)\n");
    std::printf("  -definebones  print the compiled skeleton as $definebone lines and\n");
    std::printf("                stop - no .mdl/.vvd/.vtx/.phy is written\n");
    std::printf("  -pause        wait for a keypress before exiting (drag-and-drop runs)\n");
    std::printf("  -perfmetrics  print wall time in ms for each stage of the compile\n");
    std::printf("  -dumpcommands print every accepted $command, one per line, and exit\n");
    std::printf("  -editorinfo <path>\n");
    std::printf("                write a JSON report of the compile (files read, files\n");
    std::printf("                written, errors, timing) for an editor to consume\n");
    return 1;
}

static std::string JsonEscape(const std::string& s) {
    std::string out;
    for (char ch : s) {
        if (ch == '"' || ch == '\\') { out += '\\'; out += ch; }
        else if (ch == '\n') out += "\\n";
        else if (ch == '\r') out += "\\r";
        else if (ch == '\t') out += "\\t";
        else out += ch;
    }
    return out;
}

// Split a loader error, which reads `file(line): message`. Anything else is
// reported whole with no line, so a compile/write error still comes through.
static void SplitError(const std::string& e, std::string& file, int& line, std::string& msg) {
    file.clear();
    line = 0;
    msg = e;
    const size_t sep = e.find("): ");
    const size_t open = e.rfind('(', sep);
    if (sep == std::string::npos || open == std::string::npos)
        return;
    const std::string num = e.substr(open + 1, sep - open - 1);
    if (num.empty() || num.find_first_not_of("0123456789") != std::string::npos)
        return;
    file = e.substr(0, open);
    line = std::atoi(num.c_str());
    msg = e.substr(sep + 3);
}

// -editorinfo <path>: one JSON per compile for an editor - what the compile
// read, what it wrote, why it failed, how long it took. Written from the
// destructor so every early `return Fail(...)` still reports the error.
struct EditorInfo {
    std::string path; // empty when the flag was not given - then nothing is written
    std::string error;
    double load = 0, compile = 0, write = 0, total = 0;

    ~EditorInfo() {
        if (path.empty())
            return;
        FILE* f = nullptr;
#ifdef _WIN32
        fopen_s(&f, path.c_str(), "wb");
#else
        f = fopen(path.c_str(), "wb");
#endif
        if (!f) {
            std::printf("warning: -editorinfo: cannot write %s\n", path.c_str());
            return;
        }
        auto list = [f](const char* key, const std::vector<std::string>& v, bool comma) {
            std::fprintf(f, "  \"%s\": [", key);
            for (size_t i = 0; i < v.size(); ++i)
                std::fprintf(f, "%s\n    \"%s\"", i ? "," : "", JsonEscape(v[i]).c_str());
            std::fprintf(f, "%s]%s\n", v.empty() ? "" : "\n  ", comma ? "," : "");
        };
        std::fprintf(f, "{\n");
        list("files", pulse::loader::g_openedFiles, true);
        list("output", pulse::writer::g_writtenFiles, true);
        std::fprintf(f, "  \"errors\": [");
        if (!error.empty()) {
            std::string file, msg;
            int line = 0;
            SplitError(error, file, line, msg);
            std::fprintf(f, "\n    { \"file\": \"%s\", \"line\": %d, \"msg\": \"%s\" }\n  ",
                         JsonEscape(file).c_str(), line, JsonEscape(msg).c_str());
        }
        std::fprintf(f, "],\n");
        std::fprintf(f, "  \"inactive\": [");
        {
            const auto& r = pulse::loader::g_inactiveRanges;
            for (size_t i = 0; i < r.size(); ++i)
                std::fprintf(f, "%s\n    [%d, %d]", i ? "," : "", r[i].first, r[i].second);
            std::fprintf(f, "%s],\n", r.empty() ? "" : "\n  ");
        }
        std::fprintf(f,
                     "  \"timing\": { \"load\": %.0f, \"compile\": %.0f, \"write\": %.0f, "
                     "\"total\": %.0f }\n}\n",
                     load, compile, write, total);
        std::fclose(f);
    }
};

// -definebones: dump the final bone table in $definebone form so it can be
// pasted back into the script and pin the skeleton (reference DumpDefineBones).
// Both sextets are the compiled matrices, so the second one marks every bone
// pre-aligned - a re-compile must not realign an already-realigned rig.
static void DumpDefineBones(const pulse::compile::CompiledModel& model) {
    std::printf("\n--- $definebone (%zu bones) ---------------------------------\n\n",
                model.bones.size());
    for (const pulse::compile::Bone& b : model.bones) {
        const pulse::math::Vector3 rot = pulse::math::MatrixAnglesDeg(b.rawLocal);
        const pulse::math::Vector3 realignRot = pulse::math::MatrixAnglesDeg(b.srcRealign);
        std::printf("$definebone \"%s\" \"%s\" %f %f %f %f %f %f %f %f %f %f %f %f\n",
                    b.name.c_str(),
                    b.parent >= 0 ? model.bones[b.parent].name.c_str() : "",
                    b.rawLocal.m[0][3], b.rawLocal.m[1][3], b.rawLocal.m[2][3],
                    rot.x, rot.y, rot.z,
                    b.srcRealign.m[0][3], b.srcRealign.m[1][3], b.srcRealign.m[2][3],
                    realignRot.x, realignRot.y, realignRot.z);
    }
    std::printf("\n------------------------------------------------------------\n");
}

static int RunCompile(int argc, char** argv) {
    if (argc < 2)
        return Usage();

    g_stage = "command line";
    EditorInfo info; // writes on scope exit, including every error path below
    const char* script = nullptr;
    std::string outdir;
    int vtxFormat = -1; // unset; otherwise wins over the script's $vtxformat
    bool definebones = false;
    pulse::loader::ScriptVars defvars;
    pulse::loader::SearchDirs includeDirs, fileDirs;
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
        } else if (std::strcmp(argv[i], "-includesearchdir") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-includesearchdir needs a directory");
            includeDirs.emplace_back(argv[++i]);
        } else if (std::strcmp(argv[i], "-filesearchdir") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-filesearchdir needs a directory");
            fileDirs.emplace_back(argv[++i]);
        } else if (std::strcmp(argv[i], "-vtxformat") == 0 && i + 1 < argc) {
            vtxFormat = std::atoi(argv[++i]);
            if (vtxFormat != 0 && vtxFormat != 1)
                return Fail("bad option", std::string("-vtxformat must be 0 or 1, got \"") +
                                              argv[i] + "\"");
        } else if (std::strcmp(argv[i], "-editorinfo") == 0 && i + 1 < argc) {
            info.path = argv[++i];
        } else if (std::strcmp(argv[i], "-definebones") == 0) {
            definebones = true;
        } else if (std::strcmp(argv[i], "-perfmetrics") == 0) {
            pulse::perf::g_enabled = true;
        } else if (std::strcmp(argv[i], "-pause") == 0) {
            pulse::fatal::g_pause = true;
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
    const bool timing = pulse::perf::g_enabled;
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
    if (!pulse::loader::LoadQcScript(script, input, &err, defvars, includeDirs, fileDirs)) {
        info.error = err;
        return Fail("script error", err);
    }
    auto tLoad = Clock::now();
    info.load = ms(t0, tLoad);
    if (vtxFormat >= 0)
        input.vtxArchetype = vtxFormat;

    pulse::compile::CompiledModel model;
    g_stage = "compile";
    if (!pulse::compile::Compile(input, model, &err)) {
        info.error = err;
        return Fail("compile error", err);
    }

    auto tCompile = Clock::now();
    info.compile = ms(tLoad, tCompile);

    if (definebones) {
        g_stage = "definebones";
        DumpDefineBones(model);
        return 0;
    }

    g_stage = "write";
    if (!pulse::writer::WriteModelFiles(model, outdir,
                                        /*legacyVtx=*/input.vtxArchetype == 0, &err)) {
        info.error = err;
        return Fail("write error", err);
    }
    auto tWrite = Clock::now();
    info.write = ms(tCompile, tWrite);
    info.total = ms(t0, tWrite);
    g_stage = "done";

    // lodFlag is a bit per LOD. The pool is shared, so a decimated LOD adds no
    // verts to it - only a replacemodel LOD does. Per-LOD = verts it draws.
    std::vector<size_t> lodVerts(model.scriptLods.size(), 0);
    size_t vertsPool = 0;
    for (const auto& m : model.models) {
        vertsPool += m.vertices.size();
        for (const auto& v : m.vertices)
            for (size_t l = 0; l < lodVerts.size(); l++)
                if (v.lodFlag & (1 << l)) lodVerts[l]++;
    }
    std::printf("bones: %zu | verts: %zu in .vvd\n", model.bones.size(), vertsPool);
    for (size_t l = 0; l < lodVerts.size(); l++)
        std::printf("  %s%zu: %zu verts\n",
                    model.scriptLods[l].switchValue < 0 ? "shadowlod" : "lod", l, lodVerts[l]);
    std::printf("compile time: %.2f s\n", ms(t0, tWrite) / 1000.0);

    if (timing) {
        pulse::perf::Record("total", "script load", ms(t0, tLoad));
        pulse::perf::Record("total", "compile", ms(tLoad, tCompile));
        pulse::perf::Record("total", "write", ms(tCompile, tWrite));
        pulse::perf::Record("total", "whole compile", ms(t0, tWrite));
        pulse::perf::Report();
    }
    return 0;
}

namespace pulse::perf {

bool g_enabled = false;

namespace {
struct Entry {
    const char* tag;
    const char* name;
    double ms = 0;
    int calls = 0;
};
std::vector<Entry> g_entries;
} // namespace

void Record(const char* tag, const char* name, double ms) {
    for (Entry& e : g_entries) {
        if (std::strcmp(e.tag, tag) == 0 && std::strcmp(e.name, name) == 0) {
            e.ms += ms;
            e.calls++;
            return;
        }
    }
    g_entries.push_back({tag, name, ms, 1});
}

// One table at the end, grouped by tag in the order the tags first appeared.
// A stage under 1 ms is dropped - it is noise next to the ones that matter.
void Report() {
    if (!g_enabled)
        return;
    std::printf("---------------------\n-perfmetrics\n");
    std::vector<const char*> tags;
    for (const Entry& e : g_entries) {
        bool seen = false;
        for (const char* t : tags)
            seen = seen || std::strcmp(t, e.tag) == 0;
        if (!seen)
            tags.push_back(e.tag);
    }
    for (const char* tag : tags) {
        std::printf("  [%s]\n", tag);
        for (const Entry& e : g_entries) {
            if (std::strcmp(e.tag, tag) != 0 || e.ms < 1.0)
                continue;
            if (e.calls > 1)
                std::printf("    %-30s %9.1f ms  (%d calls)\n", e.name, e.ms, e.calls);
            else
                std::printf("    %-30s %9.1f ms\n", e.name, e.ms);
        }
    }
}

} // namespace pulse::perf

int main(int argc, char** argv) {
    // progress lines are useless if they sit in the CRT buffer until exit -
    // MSVC has no line buffering (_IOLBF == _IOFBF), so go unbuffered
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    pulse::fatal::Install();

    // -dumpcommands takes no script and runs before the banner: the output is
    // read by tools, so stdout carries the command list and nothing else.
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-dumpcommands") == 0) {
            pulse::loader::PrintCommandNames();
            return 0;
        }
    }

    PrintHeader();

    // a leak past a stage's own error path still gets named in the footer
    int rc;
    try {
        rc = RunCompile(argc, argv);
    } catch (const std::bad_alloc&) {
        rc = Fail("out of memory", "an allocation failed - the model may exceed available RAM");
    } catch (const std::exception& e) {
        rc = Fail("internal error", e.what());
    } catch (...) {
        rc = Fail("internal error", "unknown C++ exception");
    }
    pulse::fatal::PauseIfAsked();
    return rc;
}
