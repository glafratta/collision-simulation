cmake_minimum_required(VERSION 3.1.0)

project(collision_simulation)

set (CMAKE_CXX_STANDARD 11)

find_package(Threads)

find_package(OpenCV REQUIRED)

add_executable(simulate main.cpp)

##add_executable(printMap printMap.cpp)

target_link_libraries(simulate alphabot pigpio rt a1lidarrpi Box2D)

target_link_libraries(simulate ${OpenCV_LIBS})

##target_link_libraries(printMap alphabot pigpio rt a1lidarrpi)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
