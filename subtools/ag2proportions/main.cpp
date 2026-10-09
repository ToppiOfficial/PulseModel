// main.cpp - ag2proportions command line: each argument is a KV3 job file or
// a .vmdl, which runs with every default and is edited in place.

#include <iostream>
#include <stdexcept>

#include "ag2.h"
#include "strcompat.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace {

using ag2::fs::path;

const char* const kUsage =
    "CS2 AnimGraph2 proportion wrapper (experimental)\n\n"
    "Usage:\n"
    "  ag2proportions <model.vmdl> [...] [options]\n"
    "  ag2proportions <job.kv3> [...] [options]\n\n"
    "Options:\n"
    "  --generate-only          Write source assets without compiling them.\n"
    "  --debug                  Write debug/ files for issue reports.\n"
    "  --compiler-runner wine   Run the Windows Resource Compiler through Wine\n"
    "                           on Linux.\n"
    "  --help, -h               Show this help.\n"
    "  --version                Show the tool version.\n\n"
    "Job file (KV3):\n"
    "  Paths are relative to the job file.\n\n"
    "  vmdl           Model to process (required).\n"
    "  model_dmx      Rig DMX.\n"
    "                 Default: the VMDL's SkeletonFile import.\n"
    "  proportions    Held-pose DMX.\n"
    "                 Default: the rig's bind pose.\n"
    "  vnmskel        Compiled worldmodel.vnmskel_c.\n"
    "                 Default: read from the installed CS2 VPK.\n"
    "  write_model    Edit the VMDL in place when true (default: false).\n"
    "                 The original is kept as <model>.vmdl.bak.\n\n"
    "Model input and output:\n"
    "  A VMDL passed directly is edited in place (write_model = true).\n"
    "  Output goes beside the VMDL, in a folder named after the model.\n\n";

bool HasExtension(const path& file, const char* ext) {
    return _stricmp(file.extension().u8string().c_str(), ext) == 0;
}

void PrintHeader() {
    std::cout << "-------------------------------\n"
                 "PulseModel [AG2 Proportions]\n"
                 "version:   " << PULSEMODEL_VERSION << '\n'
              << "developer: ToppiOfficial\n"
                 "-------------------------------\n";
}

ag2::Job LoadJob(const path& file) {
    ag2::Job job;
    if (HasExtension(file, ".vmdl")) {
        job.vmdl = file;
        job.writeModel = true; // a dragged model should come out wired up
        return job;
    }
    const auto bytes = ag2::kv::ReadFile(file.u8string());
    const ag2::kv::Value doc = ag2::kv::ParseText(std::string(bytes.begin(), bytes.end()));
    if (!doc.IsObject())
        throw std::runtime_error("the job file must be a KV3 object");
    const path base = file.parent_path();
    for (const auto& [key, value] : doc.Members()) {
        auto at = [&] { return ag2::fs::weakly_canonical(base / ag2::fs::u8path(value.String())); };
        if (key == "vmdl") job.vmdl = at();
        else if (key == "model_dmx") job.modelDmx = at();
        else if (key == "proportions") job.proportions = at();
        else if (key == "vnmskel") job.vnmskel = at();
        else if (key == "write_model") job.writeModel = value.Boolean();
        else throw std::runtime_error("unknown job key \"" + key + "\"");
    }
    if (job.vmdl.empty())
        throw std::runtime_error("the job has no vmdl");
    for (const path* p : {&job.vmdl, &job.modelDmx, &job.proportions, &job.vnmskel})
        if (!p->empty() && !ag2::fs::is_regular_file(*p))
            throw std::runtime_error("not found: " + p->u8string());
    return job;
}

int Main(const std::vector<std::string>& args) {
    PrintHeader();
    ag2::Options options;
    std::vector<path> jobs;
    for (size_t i = 1; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--version" || a == "-version") {
            std::cout << "ag2proportions " << PULSEMODEL_VERSION << '\n';
            return 0;
        }
        if (a == "--help" || a == "-help" || a == "-h") {
            std::cout << kUsage;
            return 0;
        }
        if (a == "--generate-only") {
            options.generateOnly = true;
        } else if (a == "--debug") {
            options.debug = true;
        } else if (a == "--compiler-runner") {
            if (++i >= args.size() || args[i] != "wine")
                throw std::runtime_error("--compiler-runner takes \"wine\"");
            options.compilerRunner = args[i];
        } else if (a.rfind("--", 0) == 0) {
            throw std::runtime_error("unknown option " + a);
        } else {
            jobs.push_back(ag2::fs::absolute(ag2::fs::u8path(a)));
        }
    }
    if (jobs.empty()) {
        std::cout << kUsage;
        return 1;
    }

    int failed = 0;
    for (const path& file : jobs) {
        try {
            ag2::Generate(LoadJob(file), options);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: " << file.u8string() << ": " << e.what() << '\n';
            ++failed;
        }
    }
    return failed ? 1 : 0;
}

} // namespace

#ifdef _WIN32
// A drag-and-drop launch owns its console, which closes on exit; keep it open
// so the instructions and errors can be read. A terminal launch is unchanged.
void PauseIfOwnConsole() {
    DWORD processes[2];
    if (GetConsoleProcessList(processes, 2) == 1) {
        std::cout << "\nPress Enter to close.";
        std::cin.get();
    }
}

int wmain(int argc, wchar_t** argv) {
    std::vector<std::string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(std::filesystem::path(argv[i]).u8string());
    int code = 1;
    try {
        code = Main(args);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
    }
    PauseIfOwnConsole();
    return code;
}
#else
int main(int argc, char** argv) {
    try {
        return Main(std::vector<std::string>(argv, argv + argc));
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
#endif
