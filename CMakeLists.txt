cmake_minimum_required(VERSION 3.1.0)

project(collision_simulation)

set (CMAKE_CXX_STANDARD 11)

add_executable(simulate draft2.cpp)

target_link_libraries(simulate cpptimer rt)

target_link_libraries(simulate /home/gula/Desktop/collision-simulation/libbox2d.a)


##this is for debugging using gdb
##set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
