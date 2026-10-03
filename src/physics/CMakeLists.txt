add_library(sim_physics
    src/BulletPhysicsEngine.cpp
    src/Simulation.cpp
)
target_include_directories(sim_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_physics PUBLIC sim_core)

# Ubuntu's BulletConfig.cmake sets BULLET_INCLUDE_DIRS to the relative path
# "include/bullet" and also sets BULLET_FOUND, which makes a later
# pkg_check_modules(BULLET) skip its own search. Use a distinct pkg-config prefix.
find_package(Bullet CONFIG QUIET)
set(_sim_bullet_targets FALSE)
if(TARGET BulletDynamics AND TARGET BulletCollision AND TARGET LinearMath)
    get_target_property(_sim_bullet_inc BulletDynamics INTERFACE_INCLUDE_DIRECTORIES)
    if(_sim_bullet_inc AND IS_ABSOLUTE "${_sim_bullet_inc}")
        set(_sim_bullet_targets TRUE)
    endif()
endif()
if(_sim_bullet_targets)
    target_link_libraries(sim_physics PUBLIC BulletDynamics BulletCollision LinearMath)
elseif(Bullet_FOUND AND IS_ABSOLUTE "${BULLET_INCLUDE_DIRS}")
    target_include_directories(sim_physics SYSTEM PUBLIC ${BULLET_INCLUDE_DIRS})
    target_link_libraries(sim_physics PUBLIC ${BULLET_LIBRARIES})
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(PC_BULLET REQUIRED IMPORTED_TARGET bullet)
    target_link_libraries(sim_physics PUBLIC PkgConfig::PC_BULLET)
endif()
