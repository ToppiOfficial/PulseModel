# subtools - standalone helpers that ship beside the two main executables.

# importqc (stock .qc -> .pulseqc) is deprecated and out of the build. The
# source stays for forks: re-enable by uncommenting the add_subdirectory.
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/importqc/CMakeLists.txt")
#     add_subdirectory(importqc)
# endif()

add_subdirectory(l4d_animswap)
add_subdirectory(meshtool)
