# importqc - .qc -> .pulseqc. Built twice over: a small static lib mdlcompiler
# links so it can convert a stock .qc in-process, and the drag-and-drop exe.

add_library(pulse_importqc STATIC importqc.cpp)
target_include_directories(pulse_importqc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(importqc main.cpp)
target_link_libraries(importqc PRIVATE pulse_importqc)
# subtools ship one folder down from the two main executables. The per-config
# spellings stop a multi-config generator adding a Release/ under that.
set(SUBTOOLS_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/subtools")
set_target_properties(importqc PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${SUBTOOLS_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${SUBTOOLS_DIR}")
# Each tool owns its own version; there is no project-wide one.
set(TOOL_VERSION 0.1.2)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(importqc PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")
# the lib stamps the same version into every file it writes
target_compile_definitions(pulse_importqc PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")

# Explorer's Details tab (file description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(importqc PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
                                    "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
