qt_add_executable(simstudio
    src/main.cpp
    src/MainWindow.cpp
    include/sim/app/MainWindow.hpp
    ${CMAKE_SOURCE_DIR}/resources/resources.qrc
)
set_target_properties(simstudio PROPERTIES AUTORCC ON AUTOMOC ON)
if(WIN32)
    target_sources(simstudio PRIVATE ${CMAKE_SOURCE_DIR}/resources/app_icon.rc)
endif()
target_include_directories(simstudio PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(simstudio PRIVATE
    sim_render
    sim_io
    sim_physics
    sim_scripting
    Qt6::Widgets
    Qt6::OpenGLWidgets
)
target_compile_definitions(simstudio PRIVATE SIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
