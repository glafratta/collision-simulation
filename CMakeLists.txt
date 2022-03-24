cmake_minimum_required(VERSION 3.1.0)

project(collision_simulation)

set (CMAKE_CXX_STANDARD 11)

find_package(Threads)

find_package(OpenCV REQUIRED)

add_executable(simulate main.cpp)


##target_link_libraries(simulate /usr/lib/aarch64-linux-gnu/libBox2D.a)

target_link_libraries(simulate alphabot pigpio rt a1lidarrpi box2d)

target_link_libraries(simulate ${OpenCV_LIBS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##include_directories("/usr/include/Box2D/")

##include_directories("/usr/local/include/") ##for a1lidarrpi.h

##include_directories("/usr/include/opencv4/")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
