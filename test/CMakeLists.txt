include(GNUInstallDirs)

find_package(OpenCV REQUIRED)

include_directories(../src)

find_package(box2d REQUIRED)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	add_executable(noMotors noMotors.cpp)
	target_link_libraries(noMotors HICS box2d ${OpenCV_LIBS} pthread)
	message("build test cases")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
