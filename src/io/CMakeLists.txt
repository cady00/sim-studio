add_library(sim_io
    src/OcctEnvironment.cpp
    src/ZipArchive.cpp
    src/ProjectStore.cpp
    src/StepImporter.cpp
    src/ObjImporter.cpp
)
target_include_directories(sim_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(sim_io SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_directories(sim_io PUBLIC ${OpenCASCADE_LIBRARY_DIR})
target_link_libraries(sim_io PUBLIC sim_core ${SIM_OCCT_LINK})
if(SIM_OCCT_RESOURCE_PATH)
    target_compile_definitions(sim_io PUBLIC "SIM_OCCT_RESOURCE_DIR=\"${SIM_OCCT_RESOURCE_PATH}\"")
endif()
if(UNIX)
    target_link_libraries(sim_io PUBLIC dl pthread)
endif()
