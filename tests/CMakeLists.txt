add_executable(simstudio_tests
    test_transform.cpp
    test_scene.cpp
    test_i18n.cpp
    test_project.cpp
    test_obj.cpp
    test_step.cpp
    test_mech.cpp
    test_ai.cpp
    test_motion.cpp
    test_collision.cpp
    test_physics_model.cpp
    test_contact.cpp
    test_line.cpp
    test_assets.cpp
    test_opcua.cpp
    test_weld.cpp
    test_olp.cpp
    test_devices.cpp
    test_fuzz.cpp
)
target_link_libraries(simstudio_tests PRIVATE sim_core sim_io sim_physics sim_scripting sim_ai sim_motion sim_collision Catch2::Catch2WithMain)
target_include_directories(simstudio_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/render/include)
target_compile_definitions(simstudio_tests PRIVATE "SIM_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
add_test(NAME simstudio_tests COMMAND simstudio_tests)
