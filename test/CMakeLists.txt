# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)


#pkg_check_modules(clock REQUIRED IMPORTED_TARGET libCLoCK)


#include_directories(blank ${HEAD})
add_executable(blank blank.cpp)	
add_executable(target callbacks.h target.cpp)
target_link_libraries(target CloCK ${OpenCV_LIBS} box2d cpptimer rt pthread)
target_link_libraries(blank CloCK box2d ${OpenCV_LIBS})
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
	include_directories(${LIBCAMERA_INCLUDE_DIRS})
	#add_executable(noMotors noMotors.cpp)
	#add_executable(testController testController.cpp)
	#add_executable(test_optic_flow test_optic_flow.cpp)
	add_executable(task_ofs task_ofs.cpp )
	add_executable(task_avgofs task_avgofs.cpp )
	#target_link_libraries(noMotors box2d ${OpenCV_LIBS} HICS a1lidarrpi pthread pigpio)
	#target_link_libraries(testController box2d ${OpenCV_LIBS} CLoCK a1lidarrpi pthread pigpio)
	#target_link_libraries(test_optic_flow box2d ${OpenCV_LIBS} CLoCK a1lidarrpi pthread pigpio)
	#target_include_directories(task_ofs ${LIBCAMERA_INCLUDE_DIRS})
	target_link_libraries(task_ofs box2d PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
	target_link_libraries(task_avgofs box2d PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
		
message("build test cases")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
