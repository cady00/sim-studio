add_library(sim_scripting src/ScriptHost.cpp)
target_include_directories(sim_scripting PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
