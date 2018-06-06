cmake_minimum_required(VERSION 3.5)
project(BenchmarkTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp)


find_package(OpenCV REQUIRED)
#SET(OpenCV_DIR /usr/local/share/OpenCV)

set(GLFW_INCLUDE_PATH "" CACHE PATH "The directory that contains GL/glfw.h")
set(GLFW_LIBRARY "" CACHE FILEPATH "libglfw.a or glfw.lib")
#
set(GLEW_INCLUDE_PATH "" CACHE PATH "The directory that contains GL/glew.h")
set(GLFW_LIBRARY "" CACHE FILEPATH "libglew.a or glew.lib")
#
set(GLM_INCLUDE_PATH "" CACHE PATH "The directory that contains glm/glm.hpp")



include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(run_benchmark ${SOURCE_FILES})
#target_link_libraries(run_benchmark realsense2 ${OpenCV_LIBS})
target_link_libraries(run_benchmark realsense2 ${OpenCV_LIBS} ${OPENGL_LIBRARY} ${GLFW_LIBRARY} ${GLEW_LIBRARY})
