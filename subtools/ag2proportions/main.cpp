// main.cpp - ag2proportions command line: each argument is a KV3 job file or
// a .vmdl, which runs as a job with every default.

#include <iostream>
#include <stdexcept>

#include "ag2.h"
#include "strcompat.h"

namespace {

using ag2::fs::path;

const char* const kUsage =
    "ag2proportions - CS2 AnimGraph2 proportion wrapper (experimental)\n"
    "Usage: ag2proportions <job.kv3 | model.vmdl> [...] [options]\n"
    "  --generate-only          write source assets without compiling them\n"
    "  --debug                  also write debug/ files for issue reports\n"
    "  --compiler-runner wine   Linux: run the Windows resourcecompiler through Wine\n"
    "A job file is KV3 with these keys, paths relative to the job file:\n"
    "  vmdl         the model (required)\n"
    "  model_dmx    rig DMX (default: the VMDL's SkeletonFile import)\n"
    "  proportions  held-pose DMX (default: the rig's bind pose)\n"
    "  vnmskel      compiled worldmodel.vnmskel_c (default: from the CS2 VPK)\n"
    "  write_model  true to edit the VMDL in place (original kept as .vmdl.bak)\n"
    "Output goes to a folder beside the VMDL, named after it.\n";

bool HasExtension(const path& file, const char* ext) {
    return _stricmp(file.extension().u8string().c_str(), ext) == 0;
}

ag2::Job LoadJob(const path& file) {
    ag2::Job job;
    if (HasExtension(file, ".vmdl")) {
        job.vmdl = file;
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
int wmain(int argc, wchar_t** argv) {
    std::vector<std::string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(std::filesystem::path(argv[i]).u8string());
    try {
        return Main(args);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
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
