add_library(sim_ai
    src/Commands.cpp
    src/Provider.cpp
)
target_include_directories(sim_ai PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_ai PUBLIC sim_core sim_physics)
