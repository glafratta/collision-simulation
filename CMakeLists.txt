cmake_minimum_required(VERSION 3.1.0)

project(carmelo)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

##target_link_libraries(simulate alphabot pigpio rt a1lidarrpi Box2D ncurses)


add_executable(noRobot noRobot.cpp)
#add_executable(lineFollowingSim lineFollowingSim.cpp)
add_executable(exampleTrajectories makeTrajectories.cpp)
#add_executable(testTrajectory testTrajectory.cpp)
#add_executable(testSpeedReadings testSpeedReadings.cpp)
add_executable(testMaxSim testMaxSimulation.cpp)
add_executable(testTree testTree.cpp)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

target_link_libraries(noRobot Box2D gasolina ${OpenCV_LIBS} cpptimer rt)
#target_link_libraries(lineFollowingSim Box2D gasolina ${OpenCV_LIBS} cpptimer rt)
#target_link_libraries(testTrajectory Box2D gasolina)
target_link_libraries(testMaxSim Box2D gasolina)
#target_link_libraries(testSpeedReadings gasolina Box2D ${OpenCV_LIBS} cpptimer rt)
target_link_libraries(testTree Box2D gasolina ${OpenCV_LIBS} cpptimer rt)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
