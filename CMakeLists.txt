cmake_minimum_required(VERSION 3.1.0)

project(CloCK LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads)

find_package(OpenCV REQUIRED)

##find_package(Boost)

add_subdirectory(src "${CMAKE_CURRENT_BINARY_DIR}/src")
add_subdirectory(test "test")
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
  add_executable(target main.cpp)
  add_executable(targetless targetless.cpp)
  add_executable(targetless_benchmark targetless_benchmark.cpp) 
  add_executable(targetless_benchmark_alt targetless_benchmark_alt.cpp)  
  add_executable(target_benchmark target_benchmark.cpp)
  add_executable(target_benchmark_alt target_benchmark_alt.cpp)
  add_executable(reactive reactive.cpp)
  add_executable(reactive_target reactive_target.cpp)
  target_link_libraries(reactive alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(target alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(targetless alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(targetless_benchmark alphabot a1lidarrpi CloCK pigpio)
 	target_link_libraries(targetless_benchmark_alt alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(target_benchmark alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(target_benchmark_alt alphabot a1lidarrpi CloCK pigpio)
  target_link_libraries(reactive_target alphabot a1lidarrpi CloCK pigpio)
  message("detected architecture aarch64, building executables")
else()
#	add_subdirectory(test)
	message("not building any executable")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-Wall -pedantic)
