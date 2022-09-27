cmake_minimum_required(VERSION 3.1.0)

project(carmelo)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

##target_link_libraries(simulate alphabot pigpio rt a1lidarrpi Box2D ncurses)


add_executable(noRobot noRobot.cpp)
#add_executable(lineFollowing lineFollowing.cpp)
add_executable(exampleTrajectories makeTrajectories.cpp)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

target_link_libraries(noRobot Box2D gasolina ${OpenCV_LIBS})
#target_link_libraries(lineFollowing Box2D gasolina ${OpenCV_LIBS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
