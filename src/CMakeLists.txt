set(SRC configurator.cpp task.cpp general.cpp)

set(HEAD configurator.h robot.h task.h general.h)

find_package(OpenCV REQUIRED)

add_library(HICS ${SRC} ${HEAD})

target_link_libraries(HICS Box2D ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Debug)
