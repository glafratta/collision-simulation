set(SRC configurator.cpp primitive.cpp general.cpp)

set(HEAD configurator.h robot.h primitive.h general.h)

add_library(gasolina ${SRC} ${HEAD})

target_link_libraries(gasolina Box2D ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Debug)
