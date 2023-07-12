include(GNUInstallDirs)

set(SRC configurator.cpp task.cpp general.cpp measurement.cpp)

set(HEAD configurator.h robot.h task.h general.h affordance.h settings.h measurement.h)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)

find_package(box2d)

add_library(HICS ${SRC} ${HEAD})

set_target_properties(HICS PROPERTIES POSITION_INDEPENDENT_CODE TRUE 
PUBLIC_HEADER "${HEAD}")

target_link_libraries(HICS box2d ${OpenCV_LIBS} pthread)

install(TARGETS HICS 
	EXPORT HICS-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message("library and archive destination " ${CMAKE_INSTALL_LIBDIR})

set(CMAKE_BUILD_TYPE Debug)
