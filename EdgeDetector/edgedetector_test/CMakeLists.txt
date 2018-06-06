cmake_minimum_required(VERSION 3.10)
project(edgedetector_test)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
add_executable(edgedetector_test main.cpp)
target_link_libraries(edgedetector_test ${OpenCV_LIBS})
