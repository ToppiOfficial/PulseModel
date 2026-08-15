# subtools - standalone helpers that ship beside the two main executables.

# Each subtool is optional: a checkout without its folder still builds, and
# mdlcompiler drops the feature the subtool provides.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/importqc/CMakeLists.txt")
    add_subdirectory(importqc)
endif()
