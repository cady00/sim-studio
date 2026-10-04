add_library(sim_collision
    src/Gjk.cpp
    src/Decompose.cpp
    src/Sdf.cpp
    src/CpuQuery.cpp
)
target_include_directories(sim_collision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_collision PUBLIC sim_core)
