# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

include(CTest)
enable_testing()

set(CTEST_OUTPUT_ON_FAILURE 1)

add_executable(split_task split_task.cpp)
target_link_libraries(split_task CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(backtrack backtrack.cpp)
target_link_libraries(backtrack CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(explorer explorer.cpp ../callbacks.h)
target_link_libraries(explorer CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(transitionMatrix ../callbacks.h transitionMatrix.cpp)
target_link_libraries(transitionMatrix CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(disturbance_dimensions ../callbacks.h disturbance_dimensions.cpp)
target_link_libraries(disturbance_dimensions CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(planner ../callbacks.h planner.cpp)
target_link_libraries(planner CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)


add_test(NAME split_task COMMAND split_task 0.6 0 0)
add_test(NAME split_task_angle COMMAND split_task 0.0 .6 1.57079633)
add_test(NAME split_task_offset COMMAND split_task 0.75 0.27 0 0.2 0.27 0)
add_test(NAME split_task_offset_angle COMMAND split_task 0.27 0.75 1.57079633 0.27 0.2 1.57079633)
add_test(NAME split_task_1_node COMMAND split_task 0.27 0 0)
add_test(NAME split_task_0_nodes COMMAND split_task 0.18 0 0)

add_test(NAME backtrack COMMAND backtrack 0.6 0 0)

add_test(NAME explorer_no COMMAND explorer)
add_test(NAME explorer_cds COMMAND explorer ../cul_de_sac/ 0)
add_test(NAME explorer_target COMMAND explorer ../251023/_TAGET/ 1)


add_test(NAME transition COMMAND transitionMatrix)
add_test(NAME transition_n_L_ok COMMAND transitionMatrix 0 l k)
add_test(NAME transition_n_un_ok COMMAND transitionMatrix 0 u)
add_test(NAME transition_n_D_crash COMMAND transitionMatrix 0 d c)
add_test(NAME transition_n_D_ok COMMAND transitionMatrix 0 d k)
add_test(NAME transition_n_D_safe COMMAND transitionMatrix 0 d s)
add_test(NAME transition_y_R_behind COMMAND transitionMatrix 1 r k -1 0 -3.14)
add_test(NAME transition_y_D_crash COMMAND transitionMatrix y d c)
add_test(NAME transition_y_D_ok COMMAND transitionMatrix y d k)
add_test(NAME transition_y_D_safe COMMAND transitionMatrix y d s)
add_test(NAME transition_y_stop_crash COMMAND transitionMatrix y s c)
add_test(NAME transition_y_stop_ok COMMAND transitionMatrix y s k)
add_test(NAME transition_y_stop_safe COMMAND transitionMatrix y s s)

add_test(NAME disturbance_dimensions_0 COMMAND disturbance_dimensions)

add_test(NAME planner_no COMMAND planner)
add_test(NAME planner_cds COMMAND planner ../cul_de_sac/ 0)
add_test(NAME planner_target COMMAND planner ../251023/_TAGET/ 1)




set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)

