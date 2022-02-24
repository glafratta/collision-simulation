cmake_minimum_required(VERSION 3.1.0)

project(collision_simulation)

set (CMAKE_CXX_STANDARD 11)

add_executable(simulate main.cpp)

target_link_libraries(simulate cpptimer rt)

target_link_libraries(simulate /usr/lib/x86_64-linux-gnu/libBox2D.a)

include_directories("/usr/include/opencv4/")

include_directories("/usr/include/Box2D/")

include_directories("../rplidar_rpi/")



##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
