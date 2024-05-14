cmake_minimum_required(VERSION 3.1.0)

project(CloCK LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)
find_package(PkgConfig)
find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)



##find_package(Boost)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")

add_subdirectory(test "${CMAKE_CURRENT_BINARY_DIR}/test")

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  #pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
  add_executable(target custom.h target.cpp)
  add_executable(targetless custom.h targetless.cpp)
  add_executable(targetless_benchmark custom.h targetless_benchmark.cpp) 
  #add_executable(targetless_benchmark_alt targetless_benchmark_alt.cpp)  
  add_executable(target_benchmark custom.h target_benchmark.cpp)
  #add_executable(target_benchmark_alt target_benchmark_alt.cpp)
  add_executable(reactive custom.h reactive.cpp)
  add_executable(reactive_target custom.h reactive_target.cpp)
  target_link_libraries(reactive ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)
  target_link_libraries(target ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)
  target_link_libraries(targetless ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)
  target_link_libraries(targetless_benchmark ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)
 #	target_link_libraries(targetless_benchmark_alt box2d alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(target_benchmark ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)

 # target_link_libraries(reactive_target ${OpenCV_LIBS} box2d alphabot a1lidarrpi CloCK pigpio cam2opencv)
 # target_include_directories(target ${LIBCAMERA_INCLUDE_DIRS})
 # target_include_directories(targetless ${LIBCAMERA_INCLUDE_DIRS})
 # target_include_directories(targetless_benchmark ${LIBCAMERA_INCLUDE_DIRS})
 # target_include_directories(target_benchmark ${LIBCAMERA_INCLUDE_DIRS})
 # target_include_directories(reactive ${LIBCAMERA_INCLUDE_DIRS})
 # target_include_directories(reactive_target ${LIBCAMERA_INCLUDE_DIRS})
  message("detected architecture aarch64, building executables")
else()
#	add_subdirectory(test)
	message("not building any executable")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)
