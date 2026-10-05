add_library(sim_core
    src/Math.cpp
    src/Transform.cpp
    src/Units.cpp
    src/Mesh.cpp
    src/Mech.cpp
    src/PhysicsModel.cpp
    src/Kinematics.cpp
    src/Proximity.cpp
    src/Scene.cpp
    src/LineModel.cpp
    src/EmsXml.cpp
    src/Dxf.cpp
    src/DemoScene.cpp
    src/Weld.cpp
    src/Envelope.cpp
    src/Pick.cpp
    src/I18n.cpp
)
target_include_directories(sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_core PUBLIC nlohmann_json::nlohmann_json)
