add_executable(simstudio_tests
    test_transform.cpp
    test_scene.cpp
    test_i18n.cpp
    test_project.cpp
    test_obj.cpp
    test_step.cpp
    test_mech.cpp
    test_ai.cpp
)
target_link_libraries(simstudio_tests PRIVATE sim_core sim_io sim_physics sim_scripting sim_ai Catch2::Catch2WithMain)
target_include_directories(simstudio_tests PRIVATE ${CMAKE_SOURCE_DIR}/src/render/include)
target_compile_definitions(simstudio_tests PRIVATE "SIM_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
add_test(NAME simstudio_tests COMMAND simstudio_tests)
