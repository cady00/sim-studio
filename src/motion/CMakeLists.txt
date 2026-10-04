add_library(sim_motion
    src/Profile.cpp
    src/Solve.cpp
    src/Editor.cpp
    src/LogicMachine.cpp
    src/RobotPlan.cpp
    src/Demos.cpp
    src/LinkSuggest.cpp
    src/External.cpp
    src/CycleTime.cpp
    src/Production.cpp
    src/WeldPlan.cpp
    src/Envelope.cpp
    src/Olp.cpp
)
target_include_directories(sim_motion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_motion PUBLIC sim_core sim_physics sim_collision)
