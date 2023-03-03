cmake_minimum_required(VERSION 3.1.0)

project(carmelo)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

include_directories(../src)

add_executable(noRobot noRobot.cpp)
add_executable(testTrajectory testTrajectory.cpp)
add_executable(testEnvRep testEnvironmentRepresentation.cpp)
add_executable(SPF simplifiedPathFiniding.cpp)
add_executable(synthData generateSynthetic.cpp)

#add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

target_link_libraries(noRobot Box2D gasolina ${OpenCV_LIBS} cpptimer rt)
#target_link_libraries(lineFollowingSim Box2D gasolina ${OpenCV_LIBS} cpptimer rt)
target_link_libraries(testTrajectory Box2D gasolina)
target_link_libraries(testEnvRep Box2D gasolina)
#target_link_libraries(testSpeedReadings gasolina Box2D ${OpenCV_LIBS} cpptimer rt)
target_link_libraries(SPF Box2D gasolina ${OpenCV_LIBS} cpptimer rt)
target_link_libraries(synthData gasolina ${OpenCV_LIBS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
