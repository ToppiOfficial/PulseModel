# mdldecompile - .mdl -> .pulseqc, reusing libs/format and libs/math.

add_executable(mdldecompile main.cpp dmxwrite.cpp smdwrite.cpp)
target_include_directories(mdldecompile PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(mdldecompile PRIVATE PULSEMDL2_VERSION="${PROJECT_VERSION}")

# Explorer's Details tab (file description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(mdldecompile PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
endif()
target_link_libraries(mdldecompile PRIVATE pulse_libs pulse_math)
