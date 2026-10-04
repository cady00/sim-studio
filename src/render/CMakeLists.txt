add_library(sim_render
    src/Camera.cpp
    src/OpenGLRenderer.cpp
    src/ViewportWidget.cpp
    include/sim/render/Renderer.hpp
    include/sim/render/OpenGLRenderer.hpp
    include/sim/render/ViewportWidget.hpp
)
target_include_directories(sim_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_render PUBLIC sim_core Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
