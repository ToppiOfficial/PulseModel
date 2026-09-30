# vmd2valvebiped - MMD .vmd motion -> DMX animation on a ValveBiped .mdl.

# Reads the .mdl and writes the DMX through the decompiler's own code.
set(DECOMP_DIR "${CMAKE_SOURCE_DIR}/mdldecompiler")
add_executable(vmd2valvebiped main.cpp "${DECOMP_DIR}/dmxwrite.cpp" "${DECOMP_DIR}/flexrig.cpp")
target_include_directories(vmd2valvebiped PRIVATE "${DECOMP_DIR}")
set(SUBTOOLS_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/subtools")
set_target_properties(vmd2valvebiped PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${SUBTOOLS_DIR}")
set(TOOL_VERSION 0.2.0)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(vmd2valvebiped PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")

if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(vmd2valvebiped PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
                                          "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
target_link_libraries(vmd2valvebiped PRIVATE pulse_libs pulse_math pulse_drlibs)
