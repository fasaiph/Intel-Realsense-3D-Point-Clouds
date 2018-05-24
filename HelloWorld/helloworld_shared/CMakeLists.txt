cmake_minimum_required(VERSION 3.10)
project(helloworld_shared)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")
add_library(helloworld_shared SHARED ${SOURCES})

if (APPLE)
    set_property(TARGET helloworld_shared PROPERTY PREFIX "lib")
    set_property(TARGET helloworld_shared PROPERTY OUTPUT_NAME "helloworld.so")
    set_property(TARGET helloworld_shared PROPERTY SUFFIX "")
    #set_property(TARGET helloworld_shared PROPERTY SOVERSION "32.1.2.0")
endif()

# May add installation location