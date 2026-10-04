qt_add_executable(simstudio
    src/main.cpp
    src/MainWindow.cpp
    src/MotionPanel.cpp
    src/CollisionPanel.cpp
    src/ChromeBar.cpp
    src/CommandPalette.cpp
    src/IntentBar.cpp
    src/TaskPanels.cpp
    src/LinePanels.cpp
    src/PhysicsShelf.cpp
    include/sim/app/PhysicsShelf.hpp
    include/sim/app/MainWindow.hpp
    include/sim/app/MotionPanel.hpp
    include/sim/app/CollisionPanel.hpp
    include/sim/app/ChromeBar.hpp
    include/sim/app/CommandPalette.hpp
    include/sim/app/IntentBar.hpp
    include/sim/app/TaskPanels.hpp
    include/sim/app/LinePanels.hpp
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
    sim_motion
    sim_ai
    Qt6::Widgets
    Qt6::OpenGLWidgets
)
target_compile_definitions(simstudio PRIVATE SIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
