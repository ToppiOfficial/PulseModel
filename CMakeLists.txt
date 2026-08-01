cmake_minimum_required(VERSION 3.22)
# PROJECT_VERSION is the single source: it feeds both the -version printout and
# the Win32 version resource (pulsemdl2/version.rc.in).
project(PulseMDL2 VERSION 0.1.0 LANGUAGES CXX)

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

# Layout (kept shallow):
#   libs/     - C++ libraries (pulselimits.h lives here), vendored OSS
#   pulsemdl2/ - the app: .pulsemdl loader, compile stage, writers, main()
#   mdldecompile/ - the reverse tool: .mdl -> .pulseqc
add_subdirectory(libs)
add_subdirectory(pulsemdl2)
add_subdirectory(mdldecompile)
