add_library(sim_physics src/PhysicsEngine.cpp)
target_include_directories(sim_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
