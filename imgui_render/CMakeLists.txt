cmake_minimum_required(VERSION 3.5.1)
project(imgui_render)

add_compile_options(-fvisibility=hidden)

link_directories(../core/)
include_directories(../core/)

find_package(DART REQUIRED COMPONENTS gui collision-bullet  CONFIG)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${DART_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

file(GLOB srcs "*.h" "*.cpp")

add_executable(imgui_render ${srcs})
target_link_libraries(imgui_render ${DART_LIBRARIES} GL GLU glut glad mss glfw imgui)