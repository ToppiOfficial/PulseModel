# l4d_animswap - vanilla L4D2 survivor .mdl -> a $declaresequence swap .qci.

add_executable(l4d_animswap main.cpp)
# subtools ship one folder down from the two main executables. The per-config
# spellings stop a multi-config generator adding a Release/ under that.
set(SUBTOOLS_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/subtools")
set_target_properties(l4d_animswap PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${SUBTOOLS_DIR}")
# Each tool owns its own version; there is no project-wide one.
set(TOOL_VERSION 0.1.0)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(l4d_animswap PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")

# Explorer's Details tab (file description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(l4d_animswap PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
                                    "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
target_link_libraries(l4d_animswap PRIVATE pulse_libs)
