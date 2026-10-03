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
#include <initializer_list>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

#include "compile.h"
#include "fatalerror.h"
#include "dependencies.h"
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
    std::printf("usage: mdlcompiler <file.qc|file.pulseqc> [-game <dir>]\n");
    std::printf("\n");
    std::printf("  -game <dir>   mod dir to install into; output goes to\n");
    std::printf("                <dir>\\models\\<modelname>.mdl (-outdir is a synonym)\n");
    std::printf("  -modelname <path>   overrides $modelname\n");
    std::printf("  -vtxformat <0|1>\n");
    std::printf("                .vtx layout, overriding the script's $vtxformat.\n");
    std::printf("                0 = legacy (TF2/L4D2/GMod/HL2), 1 = full (SFM/CS:GO/ASW)\n");
    std::printf("  -nodx80       skip the DirectX 8 .dx80.vtx output (also $nodx80).\n");
    std::printf("                dx80 is only written when the vtx format is 0\n");
    std::printf("  -includesearchdir <dir>\n");
    std::printf("                extra fallback dir for $include, searched after any\n");
    std::printf("                $addincludesearchdir; repeatable\n");
    std::printf("  -filesearchdir <dir>\n");
    std::printf("                extra fallback dir for source files, searched after\n");
    std::printf("                any $addsearchdir; repeatable\n");
    std::printf("  -tempcontent <dir>   synonym for -filesearchdir\n");
    std::printf("  -defvar <name> <value>\n");
    std::printf("                define a .pulseqc script variable ($name$) before the\n");
    std::printf("                script runs; repeatable. The script cannot override it\n");
    std::printf("  -striplods    ignore all $lod and $shadowlod commands\n");
    std::printf("  -forcewritevertexdata\n");
    std::printf("                save .vvd/.vtx even when the model has no geometry;\n");
    std::printf("                same as $forcewritevertexdata in the script\n");
    std::printf("  -minlod <lod> discard higher-detail LODs and promote this LOD to root;\n");
    std::printf("                overrides $minlod in the script\n");
    std::printf("  -definebones  print the compiled skeleton as $definebone lines and stop\n");
    std::printf("  -verify       compile the model without writing output files\n");
    std::printf("  -dumpmaterials print the names of materials used by the model\n");
    std::printf("  -dumpcommands print every accepted $command, one per line, and exit\n");
    std::printf("  -dumpprocedural <file.json>\n");
    std::printf("                write the final bone names and the jiggle, driver and\n");
    std::printf("                aim-at bone rules as JSON, then stop; nothing else is written\n");
    std::printf("  -dumpdependencies <file.txt>\n");
    std::printf("                after a successful compile, write every file it read, one per\n");
    std::printf("                line as <script|include|source>, a tab, then the absolute path\n");
    std::printf("  -pause        wait for a keypress before exiting (drag-and-drop runs)\n");
    std::printf("  -perfmetrics  print wall time in ms for each stage of the compile\n");
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

static void JsonString(std::FILE* f, const std::string& s) {
    std::fputc('"', f);
    for (const char c : s) {
        if (c == '"' || c == '\\')
            std::fprintf(f, "\\%c", c);
        else if (static_cast<unsigned char>(c) < 0x20)
            std::fprintf(f, "\\u%04x", c);
        else
            std::fputc(c, f);
    }
    std::fputc('"', f);
}

static void JsonFloats(std::FILE* f, std::initializer_list<float> v) {
    std::fputc('[', f);
    bool first = true;
    for (const float x : v) {
        std::fprintf(f, first ? "%.9g" : ",%.9g", static_cast<double>(x));
        first = false;
    }
    std::fputc(']', f);
}

// -dumpprocedural: the procedural bone rules with the values the .mdl stores,
// bones named rather than indexed, so a tool can lay them over a compiled model.
static bool DumpProcedural(const pulse::compile::CompiledModel& m, const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    auto boneName = [&](int i) -> std::string {
        return i >= 0 && i < static_cast<int>(m.bones.size()) ? m.bones[i].name : std::string();
    };

    std::fprintf(f, "{\"bones\":[");
    for (size_t i = 0; i < m.bones.size(); i++) {
        if (i) std::fputc(',', f);
        JsonString(f, m.bones[i].name);
    }

    std::fprintf(f, "],\n\"jiggle\":[");
    for (size_t i = 0; i < m.jigglebones.size(); i++) {
        const pulse::compile::JiggleBone& j = m.jigglebones[i];
        std::fprintf(f, i ? ",\n{\"bone\":" : "\n{\"bone\":");
        JsonString(f, boneName(j.bone));
        std::fprintf(f, ",\"flags\":%d,\"values\":", j.flags);
        JsonFloats(f, {j.length, j.tipMass, j.yawStiffness, j.yawDamping, j.pitchStiffness,
                       j.pitchDamping, j.alongStiffness, j.alongDamping, j.angleLimit, j.minYaw,
                       j.maxYaw, j.yawFriction, j.yawBounce, j.minPitch, j.maxPitch,
                       j.pitchFriction, j.pitchBounce, j.baseMass, j.baseStiffness, j.baseDamping,
                       j.baseMinLeft, j.baseMaxLeft, j.baseLeftFriction, j.baseMinUp, j.baseMaxUp,
                       j.baseUpFriction, j.baseMinForward, j.baseMaxForward, j.baseForwardFriction,
                       j.boingImpactSpeed, j.boingImpactAngle, j.boingDampingRate,
                       j.boingFrequency, j.boingAmplitude});
        std::fputc('}', f);
    }

    std::fprintf(f, "],\n\"driver\":[");
    for (size_t i = 0; i < m.proceduralbones.size(); i++) {
        const pulse::compile::ProceduralBone& p = m.proceduralbones[i];
        std::fprintf(f, i ? ",\n{\"bone\":" : "\n{\"bone\":");
        JsonString(f, boneName(p.helper));
        std::fprintf(f, ",\"driver\":");
        JsonString(f, boneName(p.driver));
        std::fprintf(f, ",\"triggers\":[");
        for (size_t t = 0; t < p.triggers.size(); t++) {
            const pulse::compile::ProceduralBoneTrigger& tr = p.triggers[t];
            // same double-then-narrow as the writer
            const float invTolerance = static_cast<float>(1.0 / static_cast<double>(tr.tolerance));
            std::fprintf(f, t ? "," : "");
            JsonFloats(f, {invTolerance, tr.trigger.x, tr.trigger.y, tr.trigger.z, tr.trigger.w,
                           tr.pos.x, tr.pos.y, tr.pos.z,
                           tr.quat.x, tr.quat.y, tr.quat.z, tr.quat.w});
        }
        std::fprintf(f, "]}");
    }

    std::fprintf(f, "],\n\"aimat\":[");
    for (size_t i = 0; i < m.aimatbones.size(); i++) {
        const pulse::compile::AimAtBone& a = m.aimatbones[i];
        std::fprintf(f, i ? ",\n{\"bone\":" : "\n{\"bone\":");
        JsonString(f, boneName(a.bone));
        std::fprintf(f, ",\"parent\":");
        JsonString(f, boneName(a.parent));
        std::fprintf(f, ",\"attachment\":%s,\"aim\":", a.aimAttach == -1 ? "false" : "true");
        JsonString(f, a.aimAttach == -1 ? boneName(a.aimBone) : a.aimname);
        std::fprintf(f, ",\"vectors\":");
        JsonFloats(f, {a.aimvector.x, a.aimvector.y, a.aimvector.z, a.upvector.x, a.upvector.y,
                       a.upvector.z, a.basepos.x, a.basepos.y, a.basepos.z});
        std::fputc('}', f);
    }
    std::fprintf(f, "]}\n");
    return std::fclose(f) == 0;
}

static int RunCompile(int argc, char** argv) {
    if (argc < 2)
        return Usage();

    g_stage = "command line";
    const char* script = nullptr;
    std::string outdir;
    std::string modelname; // -modelname: overrides the script's $modelname
    int vtxFormat = -1; // unset; otherwise wins over the script's $vtxformat
    bool noDx80 = false; // -nodx80: force-skip .dx80.vtx, overriding the script
    int launchMinLod = -1;
    bool stripLods = false;
    bool forceWriteVertexData = false;
    bool definebones = false;
    bool verify = false;
    bool dumpMaterials = false;
    std::string dumpProcedural;
    std::string dependencyFile;
    pulse::loader::ScriptVars defvars;
    pulse::loader::SearchDirs includeDirs, fileDirs;
    for (int i = 1; i < argc; ++i) {
        // -game is studiomdl's name for it: the mod dir the model installs
        // into. The writer already roots output at <dir>/models, so it is the
        // same value as -outdir.
        if ((std::strcmp(argv[i], "-game") == 0 || std::strcmp(argv[i], "-outdir") == 0) &&
            i + 1 < argc) {
            outdir = argv[++i];
        } else if (std::strcmp(argv[i], "-modelname") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-modelname needs a model path");
            modelname = argv[++i];
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
        } else if (std::strcmp(argv[i], "-filesearchdir") == 0 ||
                   std::strcmp(argv[i], "-tempcontent") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", std::string(argv[i]) + " needs a directory");
            fileDirs.emplace_back(argv[++i]);
        } else if (std::strcmp(argv[i], "-vtxformat") == 0 && i + 1 < argc) {
            vtxFormat = std::atoi(argv[++i]);
            if (vtxFormat != 0 && vtxFormat != 1)
                return Fail("bad option", std::string("-vtxformat must be 0 or 1, got \"") +
                                              argv[i] + "\"");
        } else if (std::strcmp(argv[i], "-minlod") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-minlod needs a non-negative LOD index");
            const std::string value = argv[++i];
            try {
                size_t used = 0;
                launchMinLod = std::stoi(value, &used);
                if (used != value.size() || launchMinLod < 0)
                    throw std::invalid_argument("invalid LOD index");
            } catch (const std::exception&) {
                return Fail("bad option", "-minlod needs a non-negative LOD index, got \"" +
                                              value + "\"");
            }
        } else if (std::strcmp(argv[i], "-nodx80") == 0) {
            noDx80 = true;
        } else if (std::strcmp(argv[i], "-striplods") == 0) {
            stripLods = true;
        } else if (std::strcmp(argv[i], "-forcewritevertexdata") == 0) {
            forceWriteVertexData = true;
        } else if (std::strcmp(argv[i], "-definebones") == 0) {
            definebones = true;
        } else if (std::strcmp(argv[i], "-verify") == 0) {
            verify = true;
        } else if (std::strcmp(argv[i], "-dumpprocedural") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-dumpprocedural needs an output file");
            dumpProcedural = argv[++i];
        } else if (std::strcmp(argv[i], "-dumpdependencies") == 0) {
            if (i + 1 >= argc)
                return Fail("bad option", "-dumpdependencies needs an output file");
            dependencyFile = argv[++i];
            pulse::dependencies::g_enabled = true;
        } else if (std::strcmp(argv[i], "-dumpmaterials") == 0) {
            dumpMaterials = true;
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
    // pre-seeded so a script without $modelname still loads under -modelname
    input.outname = modelname;
    input.stripLods = stripLods;
    g_stage = "script load";
    if (!pulse::loader::LoadQcScript(script, input, &err, defvars, includeDirs, fileDirs))
        return Fail("script error", err);
    auto tLoad = Clock::now();
    if (forceWriteVertexData)
        input.forceWriteVertexData = true;
    if (stripLods)
        input.minLod = 0;
    else if (launchMinLod >= 0)
        input.minLod = launchMinLod;
    if (!modelname.empty()) {
        // -modelname wins over $modelname; extension stripped the same way
        const size_t dot = modelname.find_last_of('.');
        const size_t sep = modelname.find_last_of("/\\");
        input.outname = (dot != std::string::npos && (sep == std::string::npos || dot > sep))
                            ? modelname.substr(0, dot)
                            : modelname;
    }
    if (vtxFormat >= 0)
        input.vtxArchetype = vtxFormat;
    if (noDx80)
        input.noDx80 = true;
    input.stopAfterProcedural = !dumpProcedural.empty();

    pulse::compile::CompiledModel model;
    g_stage = "compile";
    if (!pulse::compile::Compile(input, model, &err))
        return Fail("compile error", err);
    auto tCompile = Clock::now();

    if (!dumpProcedural.empty()) {
        g_stage = "dumpprocedural";
        if (!DumpProcedural(model, dumpProcedural))
            return Fail("write error", "cannot write \"" + dumpProcedural + "\"");
        std::printf("procedural bones written to %s (%.2f s)\n", dumpProcedural.c_str(),
                    ms(t0, tCompile) / 1000.0);
        return 0;
    }

    if (dumpMaterials) {
        std::printf("Used materials:\n");
        for (int texture : model.mats->materialToTexture)
            std::printf("  %s\n", model.mats->textures[texture].name.c_str());
    }

    if (definebones) {
        g_stage = "definebones";
        DumpDefineBones(model);
        return 0;
    }

    if (!verify) {
        g_stage = "write";
        // dx80 only ships with the legacy archetype; Alien Swarm+ (archetype 1)
        // dropped DirectX 8, and $nodx80 / -nodx80 opt out explicitly.
        const bool legacyVtx = input.vtxArchetype == 0;
        const bool writeDx80 = legacyVtx && !input.noDx80;
        if (!pulse::writer::WriteModelFiles(model, outdir, legacyVtx, writeDx80, &err))
            return Fail("write error", err);
    }
    if (!dependencyFile.empty() && !pulse::dependencies::Write(dependencyFile))
        return Fail("write error", "cannot write \"" + dependencyFile + "\"");
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
        pulse::perf::Record("total", "script load", ms(t0, tLoad));
        pulse::perf::Record("total", "compile", ms(tLoad, tCompile));
        pulse::perf::Record("total", "write", ms(tCompile, tWrite));
        pulse::perf::Record("total", "whole compile", ms(t0, tWrite));
        pulse::perf::Report();
    }
    return 0;
}

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
