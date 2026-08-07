cmake_minimum_required(VERSION 3.22)
# No project-wide version: each tool sets TOOL_VERSION in its own CMakeLists,
# feeding both its -version printout and its Win32 version resource.
# C is enabled only for the vendored ufbx single-file reader; all of our own
# code is C++.
project(PulseMDL2 LANGUAGES CXX C)

# C++17: inline constexpr + nested namespaces (used by libs/pulselimits.h).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single portable executable; no external package manager (vcpkg/conan).
# Linux build is a near-completion goal - avoid Windows-only imports in code.
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM_DEFINES COMPILER_MSVC64 COMPILER_MSVC _AMD64_)
    else()
        message(FATAL_ERROR "PulseMDL2 targets 64-bit only.")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PLATFORM_DEFINES LINUX COMPILER_GCC POSIX _AMD64_)
endif()

add_compile_definitions(${PLATFORM_DEFINES} _CRT_SECURE_NO_WARNINGS)

# Both tools ship side by side in one PulseModel/ folder. Guarded so a -D on
# the command line (the CI workflows) still wins.
if(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/PulseModel")
endif()

# Layout (kept shallow):
#   libs/     - C++ libraries (pulselimits.h lives here), vendored OSS
#   pulsemdl2/ - the app: .pulsemdl loader, compile stage, writers, main()
#   mdldecompile/ - the reverse tool: .mdl -> .pulseqc
add_subdirectory(libs)
add_subdirectory(pulsemdl2)
add_subdirectory(mdldecompile)
