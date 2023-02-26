cmake_minimum_required(VERSION 3.1.0)

project(carmelo)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")
add_subdirectory(test)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
