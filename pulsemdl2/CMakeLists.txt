# pulsemdl2 - the single portable executable.

add_executable(pulsemdl2 main.cpp dmxloader.cpp dmxrig.cpp smdloader.cpp
               fbxloader.cpp meshedit.cpp compile.cpp facemarkup.cpp
               flexreg.cpp qcloader.cpp writemdl.cpp writevtx.cpp writephy.cpp)
# Each tool owns its own version; there is no project-wide one.
set(TOOL_VERSION 0.3.0)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(pulsemdl2 PRIVATE PULSEMDL2_VERSION="${TOOL_VERSION}")

# Explorer's Details tab (file description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(pulsemdl2 PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc")
endif()
# pulse_vhacd is named directly (not just via pulse_phys) because the compile
# stage calls the decomposition API for PhysicsShapeFromRender.
target_link_libraries(pulsemdl2 PRIVATE pulse_libs pulse_dmx pulse_math pulse_meshopt
                                       pulse_phys pulse_vhacd pulse_ufbx)
