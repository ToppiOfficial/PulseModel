# mdlcompiler - the single portable executable.

add_executable(mdlcompiler main.cpp api.h dmxloader.cpp dmxvalidate.cpp dmxrig.cpp smdloader.cpp
               fbxloader.cpp meshedit.cpp compile.cpp facemarkup.cpp rename.cpp
               flexreg.cpp pulsemodel.cpp qcloader.cpp qcpackage.cpp writemdl.cpp writevtx.cpp writephy.cpp)
# Each tool owns its own version; there is no project-wide one.
set(TOOL_VERSION 1.25.0)
string(REPLACE "." "," TOOL_VERSION_COMMA "${TOOL_VERSION}")
target_compile_definitions(mdlcompiler PRIVATE PULSEMODEL_VERSION="${TOOL_VERSION}")

# App icon plus Explorer's Details tab (description, product name/version, copyright).
if(WIN32)
    configure_file(version.rc.in "${CMAKE_CURRENT_BINARY_DIR}/version.rc" @ONLY)
    target_sources(mdlcompiler PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.rc"
                                       "${CMAKE_SOURCE_DIR}/utf8.manifest")
endif()
# pulse_vhacd is named directly (not just via pulse_phys) because the compile
# stage calls the decomposition API for PhysicsShapeFromRender.
find_package(Threads REQUIRED)
target_link_libraries(mdlcompiler PRIVATE pulse_miniz pulse_api pulse_libs pulse_dmx pulse_math pulse_meshopt
                                          pulse_phys pulse_vhacd pulse_ufbx Threads::Threads)
