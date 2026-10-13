# subtools - standalone helpers that ship beside the two main executables.

# Per-subtool build toggle (default ON). CI turns one OFF to drop it from the
# release zip; a default build is unchanged.
option(PULSE_BUILD_L4D_ANIMSWAP "Build the l4d_animswap subtool" ON)
option(PULSE_BUILD_VMD2VALVEBIPED "Build the vmd2valvebiped subtool" ON)

if(PULSE_BUILD_L4D_ANIMSWAP)
    add_subdirectory(l4d_animswap)
endif()
if(PULSE_BUILD_VMD2VALVEBIPED)
    add_subdirectory(vmd2valvebiped)
endif()

option(PULSE_BUILD_AG2PROPORTIONS "Build the experimental AG2 proportions subtool" ON)
if(PULSE_BUILD_AG2PROPORTIONS)
    add_subdirectory(ag2proportions)
endif()
