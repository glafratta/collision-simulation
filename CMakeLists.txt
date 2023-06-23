cmake_minimum_required(VERSION 3.1.0)

project(HICS LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)

find_package(OpenCV REQUIRED)

find_package(Boost)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  add_executable(navigate main.cpp)
  target_link_libraries(navigate alphabot a1lidarrpi HICS pigpio)
  message("detected architecture aarch64, building navigate.exe")
else()
#	add_subdirectory(test)
	message("not building any executable")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
#set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
