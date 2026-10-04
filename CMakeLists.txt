cmake_minimum_required(VERSION 3.24)

project(SimStudio
    VERSION 0.3.0
    DESCRIPTION "Sim Studio — private automotive production-line simulation"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SIMSTUDIO_BUILD_TESTS "Build unit tests" ON)
option(SIMSTUDIO_BUILD_APP "Build the Qt application" ON)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCASCADE REQUIRED)

# OCCT 7.6 (Ubuntu) still ships TKSTEP*. OCCT 7.8+ (vcpkg 8.x) replaced those
# with TKDESTEP. Link a toolkit only when this machine actually has it.
set(SIM_OCCT_CANDIDATES
    TKernel TKMath
    TKG2d TKG3d TKGeomBase TKBRep
    TKGeomAlgo TKTopAlgo TKPrim TKBO TKShHealing TKBool TKMesh
    TKCDF TKLCAF TKCAF TKBinL TKXmlL TKBin TKXml TKStdL TKStd TKVCAF
    TKXSBase TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP
    TKXCAF TKXDESTEP TKXmlXCAF TKBinXCAF
    TKDE TKDESTEP
)
set(SIM_OCCT_LINK "")
foreach(lib IN LISTS SIM_OCCT_CANDIDATES)
    if(TARGET "${lib}")
        list(APPEND SIM_OCCT_LINK "${lib}")
    elseif(TARGET "OpenCASCADE::${lib}")
        list(APPEND SIM_OCCT_LINK "OpenCASCADE::${lib}")
    else()
        find_library(SIM_OCCT_LIB_${lib} NAMES ${lib} lib${lib} HINTS "${OpenCASCADE_LIBRARY_DIR}")
        if(SIM_OCCT_LIB_${lib})
            list(APPEND SIM_OCCT_LINK "${SIM_OCCT_LIB_${lib}}")
        endif()
    endif()
endforeach()
message(STATUS "Sim Studio OCCT libraries: ${SIM_OCCT_LINK}")

file(TO_CMAKE_PATH "${OpenCASCADE_RESOURCE_DIR}" SIM_OCCT_RESOURCE_PATH)

add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/physics)
add_subdirectory(src/scripting)
add_subdirectory(src/ai)

if(SIMSTUDIO_BUILD_APP)
    find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
    qt_standard_project_setup()
    set(CMAKE_AUTORCC ON)
    add_subdirectory(src/render)
    add_subdirectory(src/app)
endif()

if(SIMSTUDIO_BUILD_TESTS)
    enable_testing()
    find_package(Catch2 3 CONFIG REQUIRED)
    add_subdirectory(tests)
endif()

add_executable(simstudio_stepgen tools/generate_sample_step.cpp)
target_link_libraries(simstudio_stepgen PRIVATE sim_io)
target_include_directories(simstudio_stepgen SYSTEM PRIVATE "${OpenCASCADE_INCLUDE_DIR}")

add_executable(simstudio_demogen tools/generate_demo.cpp)
target_link_libraries(simstudio_demogen PRIVATE sim_io)

include(GNUInstallDirs)
if(TARGET simstudio)
    install(TARGETS simstudio RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(DIRECTORY samples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/simstudio/samples)
install(FILES README.md THIRD_PARTY_LICENSES.md LICENSE.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/simstudio)

set(CPACK_PACKAGE_NAME "SimStudio")
set(CPACK_PACKAGE_VENDOR "Sim Studio")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_FILE_NAME "SimStudio-${PROJECT_VERSION}-win64")
set(CPACK_GENERATOR "ZIP")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE.txt")
set(CPACK_NSIS_DISPLAY_NAME "Sim Studio")
set(CPACK_NSIS_PACKAGE_NAME "Sim Studio")
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
include(CPack)
