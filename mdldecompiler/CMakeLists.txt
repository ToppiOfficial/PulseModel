# mdldecompiler - .mdl -> .pulseqc, reusing libs/format and libs/math.

add_executable(mdldecompiler main.cpp dmxwrite.cpp animwrite.cpp)
target_include_directories(mdldecompiler PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# Each tool owns its own version; there is no project-wide one.
set(TOOL_VERSION 0.5.2)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(mdldecompiler PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")

# Explorer's Details tab (file description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(mdldecompiler PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
                                         "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
target_link_libraries(mdldecompiler PRIVATE pulse_libs pulse_math)
