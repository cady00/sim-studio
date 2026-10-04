add_library(sim_io
    src/OcctEnvironment.cpp
    src/ZipArchive.cpp
    src/ProjectStore.cpp
    src/EditHistory.cpp
    src/GeometryKernel.cpp
    src/StepImporter.cpp
    src/ObjImporter.cpp
    src/AssetImport.cpp
    src/OcctCadImport.cpp
    src/OpcUa.cpp
    src/OpcUaCrypto.cpp
)
target_include_directories(sim_io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(sim_io SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_directories(sim_io PUBLIC ${OpenCASCADE_LIBRARY_DIR})
find_package(OpenSSL REQUIRED)
target_link_libraries(sim_io PUBLIC sim_core OpenSSL::SSL OpenSSL::Crypto ${SIM_OCCT_LINK})
if(SIM_OCCT_RESOURCE_PATH)
    target_compile_definitions(sim_io PUBLIC "SIM_OCCT_RESOURCE_DIR=\"${SIM_OCCT_RESOURCE_PATH}\"")
endif()
if(UNIX)
    target_link_libraries(sim_io PUBLIC dl pthread)
endif()
