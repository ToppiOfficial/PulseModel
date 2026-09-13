# subtools - standalone helpers that ship beside the two main executables.

# importqc (stock .qc -> .pulseqc) is deprecated and out of the build. The
# source stays for forks: re-enable by uncommenting the add_subdirectory.
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/importqc/CMakeLists.txt")
#     add_subdirectory(importqc)
# endif()

# Per-subtool build toggle (default ON). CI turns one OFF to drop it from the
# release zip; a default build is unchanged.
option(PULSE_BUILD_L4D_ANIMSWAP "Build the l4d_animswap subtool" ON)
option(PULSE_BUILD_MESHTOOL "Build the meshtool subtool" ON)

if(PULSE_BUILD_L4D_ANIMSWAP)
    add_subdirectory(l4d_animswap)
endif()
if(PULSE_BUILD_MESHTOOL)
    add_subdirectory(meshtool)
endif()
