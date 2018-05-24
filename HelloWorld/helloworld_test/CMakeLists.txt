cmake_minimum_required(VERSION 3.10)
project(helloworld_test)

set(CMAKE_CXX_STANDARD 11)
set(PROJECT_LINK_LIBS libhelloworld.so)
link_directories(../helloworld_shared/build)

include_directories(../helloworld_shared/include)

add_executable(helloworld_test main.cpp)
target_link_libraries(helloworld_test ${PROJECT_LINK_LIBS})