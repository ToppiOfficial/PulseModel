# libs - C++ libraries for PulseMDL.

# Header-only interface target (pulselimits.h and other shared headers).
add_library(pulse_libs INTERFACE)
target_include_directories(pulse_libs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# Compiler-side math (vectors, quaternions, 3x4 transforms).
add_library(pulse_math STATIC
    math/math.cpp
)
target_include_directories(pulse_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pulse_math PUBLIC pulse_libs)

# meshoptimizer (zeux, MIT) - vendored v1.1 snapshot matching the original
# PulseMDL's copy exactly (vcache order must match for byte parity).
add_library(pulse_meshopt STATIC
    meshoptimizer/vcacheoptimizer.cpp
    meshoptimizer/overdrawoptimizer.cpp
    meshoptimizer/allocator.cpp
    meshoptimizer/simplifier.cpp
)
target_include_directories(pulse_meshopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/meshoptimizer)

# V-HACD (Khaled Mamou, BSD-3-clause) - vendored single-header library. Only
# the convex hull builder is used today; the decomposition API is what auto
# concave decomposition will need later. vhacd_impl.cpp is the one translation
# unit allowed to define ENABLE_VHACD_IMPLEMENTATION.
add_library(pulse_vhacd STATIC
    vhacd/vhacd_impl.cpp
)
target_include_directories(pulse_vhacd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vhacd)
target_link_libraries(pulse_vhacd PUBLIC pulse_libs)

# minicollision - .phy collision hulls and IVP compact-surface serialization.
# Ported from the original PulseMDL; self-contained vector math
# so it stays clear of libs/math's byte-parity constraints.
add_library(pulse_phys STATIC
    minicollision/convexhull.cpp
    minicollision/physicscollision.cpp
)
target_include_directories(pulse_phys PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/minicollision)
target_link_libraries(pulse_phys PRIVATE pulse_vhacd PUBLIC pulse_libs)

# ufbx (ufbx/ufbx, MIT) - vendored single-file FBX reader, v0.23.0. Feeds the
# FBX mesh/skeleton loader; DMX stays the primary format.
add_library(pulse_ufbx STATIC
    ufbx/ufbx.c
)
target_include_directories(pulse_ufbx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ufbx)

# DMX reader (clean-room port of Datamodel.NET's format logic).
add_library(pulse_dmx STATIC
    dmx/dmx.cpp
    dmx/dmx_keyvalues2.cpp
    dmx/dmx_binary.cpp
)
target_include_directories(pulse_dmx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pulse_dmx PUBLIC pulse_libs)
