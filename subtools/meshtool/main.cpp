// meshtool - the mesh operations PulseWorkshop's Model Editor previews with.
//
// Both subcommands exchange binary files rather than pipes: a character is
// megabytes of positions and indices, and writing stdin while reading stdout
// deadlocks on a full buffer.

#include <cstring>
#include <iostream>
#include <string>

#include "convex_decompose.h"
#include "mesh_simplify.h"

namespace {

void print_usage() {
    std::cout <<
        "meshtool - mesh simplification and convex decomposition\n"
        "Usage:\n"
        "  simplify <request_file> <response_file>\n"
        "  decompose <request_file> <response_file>\n";
}

// The decimation $decimate and $lod ask for. See mesh_simplify.h.
int cmd_simplify(int argc, char* argv[]) {
    if (argc < 2) { print_usage(); return 1; }
    std::string err;
    if (pulse::tool::RunSimplify(argv[0], argv[1], &err)) return 0;
    std::cerr << "[meshtool] simplify: " << err << "\n";
    return 1;
}

// The convex decomposition $physicsshape asks for. See convex_decompose.h.
int cmd_decompose(int argc, char* argv[]) {
    if (argc < 2) { print_usage(); return 1; }
    std::string err;
    if (pulse::tool::RunDecompose(argv[0], argv[1], &err)) return 0;
    std::cerr << "[meshtool] decompose: " << err << "\n";
    return 1;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) { print_usage(); return 1; }

    if (std::strcmp(argv[1], "simplify") == 0)
        return cmd_simplify(argc - 2, argv + 2);

    if (std::strcmp(argv[1], "decompose") == 0)
        return cmd_decompose(argc - 2, argv + 2);

    std::cerr << "[meshtool] Unknown subcommand: " << argv[1] << "\n";
    print_usage();
    return 1;
}
