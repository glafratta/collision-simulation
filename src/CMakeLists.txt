include(GNUInstallDirs)

set (CMAKE_CXX_STANDARD 17)

set(SRC disturbance.cpp configurator.cpp task.cpp graphTools.cpp measurement.cpp worldbuilder.cpp sensor.cpp)

set(HEAD disturbance.h configurator.h robot.h task.h graphTools.h affordance.h settings.h measurement.h worldbuilder.h sensor.h)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)

find_package(Box2D REQUIRED)

#find_package(PkgConfig)

if(Box2D_FOUND)
        target_link_libraries(CloCK Box2D ${OpenCV_LIBS} pthread)
        message("box2d installed ${BOX2D_LIBS}")
else()
       #set(BOX2D_BIN_DIR ${"../box2d/build/bin"})
       target_link_libraries(HICS ${BOX2D_BIN_DIR}/libBox2D.a ${OpenCV_LIBS} pthread)
       #message("fetching static library from ${BOX2D_BIN_DIR}")
	message("BOX2D NOT INSTALLED")
endif()

add_library(CloCK ${SRC} ${HEAD})

set_target_properties(CloCK PROPERTIES POSITION_INDEPENDENT_CODE TRUE 
PUBLIC_HEADER "${HEAD}")

install(TARGETS CloCK 
	EXPORT CloCK-targets 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

message("library and archive destination " ${CMAKE_INSTALL_LIBDIR})

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
