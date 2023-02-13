set(SRC configurator.cpp state.cpp general.cpp)

set(HEAD configurator.h robot.h state.h general.h)

add_library(gasolina ${SRC} ${HEAD})

target_link_libraries(gasolina Box2D ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Debug)
