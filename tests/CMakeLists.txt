add_executable(simstudio_tests
    test_transform.cpp
    test_scene.cpp
    test_i18n.cpp
    test_project.cpp
    test_obj.cpp
    test_step.cpp
)
target_link_libraries(simstudio_tests PRIVATE sim_core sim_io Catch2::Catch2WithMain)
target_compile_definitions(simstudio_tests PRIVATE "SIM_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
add_test(NAME simstudio_tests COMMAND simstudio_tests)
