add_library(sim_physics
    src/PhysicsFactory.cpp
    src/BuiltinPhysicsEngine.cpp
    src/ContactNarrow.cpp
    src/Simulation.cpp
)
target_include_directories(sim_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_physics PUBLIC sim_core)
# Static archives do not carry their own link line. The collision symbols used by
# the builtin narrow phase have to ride the public interface of sim_physics.
target_link_libraries(sim_physics PUBLIC sim_collision)
if(MSVC)
    target_compile_options(sim_physics PRIVATE /bigobj)
endif()

option(SIMSTUDIO_WITH_JOLT "Link the optional Jolt rigid-body backend" ON)
if(SIMSTUDIO_WITH_JOLT)
    target_sources(sim_physics PRIVATE src/JoltPhysicsEngine.cpp)
    # Windows uses the vcpkg port joltphysics (Jolt::Jolt). Linux CI has no distro
    # package at the same version, so it fetches the matching tag and builds it.
    find_package(Jolt CONFIG QUIET)
    if(TARGET Jolt::Jolt)
        target_link_libraries(sim_physics PUBLIC Jolt::Jolt)
    else()
        include(FetchContent)
        FetchContent_Declare(
            JoltPhysics
            GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
            GIT_TAG v5.5.0
            SOURCE_SUBDIR Build
        )
        set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
        set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
        set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
        set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
        set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
        set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
        set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
        set(DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
        set(GENERATE_DEBUG_SYMBOLS OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(JoltPhysics)
        if(TARGET Jolt::Jolt)
            target_link_libraries(sim_physics PUBLIC Jolt::Jolt)
        else()
            target_link_libraries(sim_physics PUBLIC Jolt)
        endif()
    endif()
else()
    target_compile_definitions(sim_physics PUBLIC SIM_PHYSICS_NO_JOLT)
endif()
