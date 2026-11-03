
set(CMAKE_INCLUDE_CURRENT_DIR ON)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/Cousine-Regular.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/Cousine-Regular.ttf COPYONLY)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/DroidSans.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/DroidSans.ttf COPYONLY)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/Karla-Regular.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/Karla-Regular.ttf COPYONLY)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/ProggyClean.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/ProggyClean.ttf COPYONLY)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/ProggyTiny.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/ProggyTiny.ttf COPYONLY)
configure_file(${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf ${CMAKE_CURRENT_SOURCE_DIR}/fonts/Roboto-Medium.ttf COPYONLY)

add_executable(renderer_example
        renderer_example.cpp
        gl_renderer.cpp
        gl_renderer.h
        glad.c
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui.h
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(renderer_example PRIVATE include ${imgui_SOURCE_DIR})
# The generated C++ wrapper (header-only): include paths only, nothing to link.
target_link_libraries(renderer_example PRIVATE dew::await)
cmrc_add_resource_library(renderer_example_shaders ALIAS renderer_example::shaders NAMESPACE shaders
        shaders/aabb.frag
        shaders/aabb.vert
        shaders/skybox.frag
        shaders/skybox.vert
        shaders/points.frag
        shaders/points.vert
        shaders/dynpoints.frag
        shaders/dynpoints_1.vert
        shaders/dynpoints_3.vert
        shaders/dynpoints_crossfade.vert
        shaders/axis_gizmo.vert
        shaders/axis_gizmo.frag
        shaders/origin_anchor.vert
        shaders/origin_anchor.frag
        shaders/environment.vert
        shaders/environment.frag
        shaders/node_bbox.vert
        shaders/node_bbox.frag
)
cmrc_add_resource_library(renderer_example_fonts ALIAS renderer_example::fonts NAMESPACE fonts
        fonts/Cousine-Regular.ttf
        fonts/DroidSans.ttf
        fonts/Karla-Regular.ttf
        fonts/ProggyClean.ttf
        fonts/ProggyTiny.ttf
        fonts/Roboto-Medium.ttf
)

cmrc_add_resource_library(renderer_example_textures ALIAS renderer_example::textures NAMESPACE textures
        textures/back.jpg
        textures/bottom.jpg
        textures/front.jpg
        textures/left.jpg
        textures/right.jpg
        textures/top.jpg
)

set(_renderer_example_deps dew_render dew_converter fmt glm renderer_example::shaders renderer_example::fonts renderer_example::textures SDL3::SDL3)
if(TARGET SDL3::SDL3main)
    list(APPEND _renderer_example_deps SDL3::SDL3main)
endif()
target_link_libraries(renderer_example PRIVATE ${_renderer_example_deps})
copy_dll_for_target(renderer_example dew_render dew_converter dew_core SDL3::SDL3)
setWarningFlagsForTarget(renderer_example)
if(NOT MSVC)
    target_compile_options(renderer_example PRIVATE -Wno-language-extension-token)
endif()
set_example_properties(renderer_example)
