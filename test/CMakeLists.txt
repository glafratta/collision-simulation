cmake_minimum_required(VERSION 3.1.0)

project(testpacito)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

add_executable(noRobot noRobot.cpp)
add_executable(turnMaps turnMaps.cpp)

add_subdirectory(../src "${CMAKE_CURRENT_BINARY_DIR}/src")

target_link_libraries(noRobot Box2D gasolina ${OpenCV_LIBS})
target_link_libraries(turnMaps ${OpenCV_LIBS})


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##add_subdirectory(unit-test)

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
