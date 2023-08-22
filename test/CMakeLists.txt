find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	add_executable(noMotors noMotors.cpp)
	target_link_libraries(noMotors box2d ${OpenCV_LIBS} HICS pthread)
	message("build test cases")
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
