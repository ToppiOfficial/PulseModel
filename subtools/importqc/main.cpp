// importqc - stock studiomdl .qc -> .pulseqc (importqc.h).
//
// Usage:
//   importqc <file.qc> [-o <file.pulseqc>] [-pause]

#include <cstdio>
#include <cstring>
#include <string>

#include "importqc.h"

// Defined by CMake from TOOL_VERSION, same value the .exe version resource gets.
static constexpr const char* kAppVersion = PULSEMODEL_VERSION;

static int Usage() {
    std::printf("usage: importqc <file.qc> [-o <file.pulseqc>]\n");
    std::printf("\n");
    std::printf("  -o <file>     where to write; default is <name>.pulseqc beside the input\n");
    std::printf("  -pause        wait for a keypress before exiting (drag-and-drop runs)\n");
    return 1;
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("-------------------------------\n");
    std::printf("PulseModel [QC Importer]\n");
    std::printf("version:   %s\n", kAppVersion);
    std::printf("developer: Toppi\n");
    std::printf("-------------------------------\n");

    const char* in = nullptr;
    std::string out;
    bool pause = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc)
            out = argv[++i];
        else if (std::strcmp(argv[i], "-pause") == 0)
            pause = true;
        else if (argv[i][0] == '-')
            std::printf("warning: ignoring unknown option: %s\n", argv[i]);
        else
            in = argv[i];
    }
    if (!in)
        return Usage();
    if (out.empty())
        out = pulse::importqc::DefaultOutput(in);

    std::string err;
    const bool ok = pulse::importqc::Convert(in, out, &err);
    if (!ok)
        std::fprintf(stderr, "importqc error: %s\n", err.c_str());
    if (pause) {
        std::printf("press enter to exit...");
        std::getchar();
    }
    return ok ? 0 : 1;
}
