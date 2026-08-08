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
#include <filesystem>
#include <new>
#include <string>

#include "compile.h"
#include "fatalerror.h"
#include "importqc.h"
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
    std::printf("  -vtxformat <0|1>\n");
    std::printf("                .vtx layout, overriding the script's $vtxformat.\n");
    std::printf("                0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW)\n");
    std::printf("  -definebones  print the compiled skeleton as $definebone lines and\n");
    std::printf("                stop - no .mdl/.vvd/.vtx/.phy is written\n");
    std::printf("  -studiomdl    treat the script as a stock studiomdl .qc: importqc\n");
    std::printf("                rewrites a copy as .pulseqc and that is what compiles.\n");
    std::printf("                Implied by a .qc extension\n");
    std::printf("  -pause        wait for a keypress before exiting (drag-and-drop runs)\n");
    return 1;
}

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

// A stock studiomdl script, by extension: .pulseqc is ours, a bare .qc is
// stock's and goes through importqc first.
static bool IsStockQc(const char* path) {
    std::string ext = std::filesystem::path(path).extension().string();
    for (char& c : ext)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".qc";
}

static int RunCompile(int argc, char** argv) {
    if (argc < 2)
        return Usage();

    g_stage = "command line";
    const char* script = nullptr;
    std::string outdir;
    int vtxFormat = -1; // unset; otherwise wins over the script's $vtxformat
    bool definebones = false;
    bool studiomdl = false; // -studiomdl: run the script through importqc first
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
        } else if (std::strcmp(argv[i], "-definebones") == 0) {
            definebones = true;
        } else if (std::strcmp(argv[i], "-studiomdl") == 0) {
            studiomdl = true;
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

    // A stock studiomdl .qc is rewritten into a .pulseqc beside it and the copy
    // is what compiles - the original is never written to.
    std::string converted;
    if (studiomdl || IsStockQc(script)) {
        g_stage = "qc import";
        converted = pulse::importqc::DefaultOutput(script);
        std::string importErr;
        if (!pulse::importqc::Convert(script, converted, &importErr))
            return Fail("import error", importErr);
        script = converted.c_str();
    }

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

    if (definebones) {
        g_stage = "definebones";
        DumpDefineBones(model);
        return 0;
    }

    g_stage = "write";
    if (!pulse::writer::WriteModelFiles(model, outdir,
                                        /*legacyVtx=*/input.vtxArchetype == 0, &err))
        return Fail("write error", err);
    auto tWrite = Clock::now();
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
        std::printf("[timing] load %.0f ms | compile %.0f ms | write %.0f ms | total %.0f ms\n",
                    ms(t0, tLoad), ms(tLoad, tCompile), ms(tCompile, tWrite), ms(t0, tWrite));
    }
    return 0;
}

int main(int argc, char** argv) {
    // progress lines are useless if they sit in the CRT buffer until exit -
    // MSVC has no line buffering (_IOLBF == _IOFBF), so go unbuffered
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    pulse::fatal::Install();

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
