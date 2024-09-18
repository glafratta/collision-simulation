# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

include(CTest)
enable_testing()

set(CTEST_OUTPUT_ON_FAILURE 1)

add_executable(processData_k ../../src/worldbuilder.h ../../src/worldbuilder.cpp processData_k.cpp)
add_executable(feature_clusters ../../src/worldbuilder.h ../../src/worldbuilder.cpp feature_clusters.cpp)
target_link_libraries(processData_k CloCK ${OpenCV_LIBS} box2d rt)
target_link_libraries(feature_clusters CloCK ${OpenCV_LIBS} box2d rt)


# add_executable(split_task split_task.cpp)
# target_link_libraries(split_task CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
# add_executable(backtrack backtrack.cpp)
# target_link_libraries(backtrack CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(explorer ../callbacks.h explorer.cpp )
# add_executable(transitionMatrix ../callbacks.h transitionMatrix.cpp)
# target_link_libraries(transitionMatrix CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
add_executable(disturbance_dimensions ../callbacks.h disturbance_dimensions.cpp)
add_executable(planner ../callbacks.h planner.cpp)
add_executable(match ../callbacks.h match.cpp)
add_executable(find_match_state ../callbacks.h find_match_s.cpp)
add_executable(state_difference ../callbacks.h state_difference.cpp)
add_executable(match_equal ../callbacks.h match_equal.cpp)

target_link_libraries(explorer PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(disturbance_dimensions PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(planner PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(match PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(find_match_state PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(state_difference PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
target_link_libraries(match_equal PUBLIC CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)




# add_executable(been_there ../callbacks.h been_there.cpp)
# target_link_libraries(been_there CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)


# add_test(NAME split_task COMMAND split_task 0.6 0 0)
# add_test(NAME split_task_angle COMMAND split_task 0.0 .6 1.57079633)
# add_test(NAME split_task_offset COMMAND split_task 0.75 0.27 0 0.2 0.27 0)
# add_test(NAME split_task_offset_angle COMMAND split_task 0.27 0.75 1.57079633 0.27 0.2 1.57079633)
# add_test(NAME split_task_1_node COMMAND split_task 0.27 0 0)
# add_test(NAME split_task_0_nodes COMMAND split_task 0.18 0 0)

# add_test(NAME backtrack COMMAND backtrack 0.6 0 0)

add_test(NAME explorer_no COMMAND explorer)
add_test(NAME explorer_cds COMMAND explorer ../cul_de_sac/ 0)
add_test(NAME explorer_target COMMAND explorer ../251023_TAGET/ 1)


# add_test(NAME transition COMMAND transitionMatrix)
# add_test(NAME transition_n_L_ok COMMAND transitionMatrix 0 l k)
# add_test(NAME transition_n_un_ok COMMAND transitionMatrix 0 u)
# add_test(NAME transition_n_D_crash COMMAND transitionMatrix 0 d c)
# add_test(NAME transition_n_D_ok COMMAND transitionMatrix 0 d k)
# add_test(NAME transition_n_D_safe COMMAND transitionMatrix 0 d s)
# add_test(NAME transition_y_R_behind COMMAND transitionMatrix 1 r k -1 0 -3.14)
# add_test(NAME transition_y_D_crash COMMAND transitionMatrix y d c)
# add_test(NAME transition_y_D_ok COMMAND transitionMatrix y d k)
# add_test(NAME transition_y_D_safe COMMAND transitionMatrix y d s)
# add_test(NAME transition_y_stop_crash COMMAND transitionMatrix y s c)
# add_test(NAME transition_y_stop_ok COMMAND transitionMatrix y s k)
# add_test(NAME transition_y_stop_safe COMMAND transitionMatrix y s s)

add_test(NAME disturbance_dimensions_0 COMMAND disturbance_dimensions)

add_test(NAME planner_no COMMAND planner)
add_test(NAME planner_cds COMMAND planner ../cul_de_sac/ 0)
add_test(NAME planner_target COMMAND planner ../251023_TAGET/ 1)

add_test(NAME match_false COMMAND match)
add_test(NAME match_true_exact COMMAND match 1)
add_test(NAME match_pose COMMAND match 3 )
add_test(NAME match_disturbance_exact COMMAND match 2 0)
add_test(NAME match_disturbance_ratio COMMAND match 2 1)
add_test(NAME match_disturbance_position COMMAND match 2 2)
set_property(TEST match_disturbance_position PROPERTY WILL_FAIL true)
add_test(NAME match_disturbance_shape COMMAND match 2 3)
set_property(TEST match_disturbance_shape PROPERTY WILL_FAIL true)

add_test(NAME find_match_state COMMAND find_match_state ../251023_TAGET/ 2 1)
add_test(NAME find_match_state_f COMMAND find_match_state ../251023_TAGET/ 3 1)
add_test(NAME find_match_11 COMMAND find_match_state ../251023_TAGET/ 7 1 29)
add_test(NAME find_match_midtask11 COMMAND find_match_state ../251023_TAGET/ 7 1 14)
add_test(NAME find_match_midtask2 COMMAND find_match_state ../251023_TAGET/ 2 1 14)
add_test(NAME find_match_quarter COMMAND find_match_state ../251023_TAGET/ 2 1 7)

set_property(TEST find_match_midtask11 PROPERTY WILL_FAIL true)
set_property(TEST find_match_state_f PROPERTY WILL_FAIL true)

add_test(NAME find_match_state_turn COMMAND find_match_state ../251023_TAGET/ 9 1 55)

add_test(NAME state_difference COMMAND state_difference ../251023_TAGET/ 1)
add_test(NAME state_difference_turn COMMAND state_difference ../251023_TAGET/ 1 55)
#add_test(NAME state_difference_turn COMMAND state_difference ../251023_TAGET/ 1 55)

add_test(NAME match_equal COMMAND match_equal)

add_test(NAME k_means_0 COMMAND processData_k ../170924_1321_badrun/ 1 )

add_test(NAME feature_clusters COMMAND feature_clusters ../170924_1321_badrun/ 0 )

#add_test(NAME been_there COMMAND been_there ../251023_TAGET/)
#add_test(NAME been_there_f COMMAND been_there ../251023_TAGET/)
#set_property(TEST been_there_f PROPERTY WILL_FAIL true)


set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)

