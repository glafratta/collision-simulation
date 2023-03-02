cmake_minimum_required(VERSION 3.1.0)

project(carmelo)

set (CMAKE_CXX_STANDARD 17)

find_package(Threads)

find_package(OpenCV REQUIRED)

find_package(Boost)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")
add_subdirectory(test)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  add_executable(navigate main.cpp)
  target_link_libraries(navigate alphabot a1lidarrpi gasolina pigpio)
  message("detected architecture aarch64, building navigate.exe")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64") #desktop
	message("skipping programs requiring pigpio")
else()
  message("not x86_64 or aarch64 architecture")
endif()


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

##this is for debugging using gdb
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
