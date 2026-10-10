add_executable(ag2proportions main.cpp workflow.cpp rig.cpp graph.cpp compiler.cpp)
target_link_libraries(ag2proportions PRIVATE pulse_libs pulse_math pulse_dmx pulse_keyvalues3)
target_include_directories(ag2proportions PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/wrapper.kv3" AG2_WRAPPER)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/assettypes.kv3" AG2_ASSET_TYPES)
configure_file(templates.h.in templates.h @ONLY)
set(SUBTOOLS_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/subtools")
set_target_properties(ag2proportions PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${SUBTOOLS_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${SUBTOOLS_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${SUBTOOLS_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${SUBTOOLS_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${SUBTOOLS_DIR}")
set(TOOL_VERSION 0.3.0)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(ag2proportions PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(ag2proportions PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc" "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
add_custom_command(TARGET ag2proportions POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:ag2proportions>/licenses"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/libs/zstd/LICENSE" "$<TARGET_FILE_DIR:ag2proportions>/licenses/zstd.txt"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/libs/keyvalues3/ValveResourceFormat.LICENSE" "$<TARGET_FILE_DIR:ag2proportions>/licenses/ValveResourceFormat.txt")
