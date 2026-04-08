cmake_minimum_required(VERSION 3.10)

option(TRAJLO_HEADLESS "Build headless (no GUI, no GLFW)" OFF)

add_subdirectory(ros EXCLUDE_FROM_ALL)
add_subdirectory(yaml-cpp)
add_subdirectory(eigen)

if(NOT TRAJLO_HEADLESS)
# imgui lib
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB IMGUI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp")

add_library(imgui SHARED
        ${IMGUI_SRC}
        imgui/backends/imgui_impl_glfw.cpp
        imgui/backends/imgui_impl_opengl3.cpp
        implot/implot_items.cpp
        implot/implot.cpp
        implot/implot_demo.cpp
        )
target_include_directories(imgui PUBLIC
        imgui/
        imgui/backends/
        implot/)
target_link_libraries(imgui PUBLIC
        glfw OpenGL::GL
        ${CMAKE_DL_LIBS}
        )
endif()

add_subdirectory(glm)
add_subdirectory(robin-map)

option(TBB_TEST "Enable testing" OFF)
add_subdirectory(oneTBB)


