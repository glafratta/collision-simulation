set(SRC configurator.cpp state.cpp)

set(HEAD configurator.h robot.h state.h)

add_library(gasolina ${SRC} ${HEAD})

target_link_libraries(gasolina Box2D ${OpenCV_LIBS})
