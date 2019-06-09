add_library(state_estimation_objs OBJECT
  car_specs.cpp
  servo_characteristic.cpp
  state_estimation_node_debug.cpp
  state_estimation.cpp
  kalman_filter/kalman_filter.cpp
  kalman_filter/linear_kalman_filter/linear_2ws/linear_2ws_kalman_filter.cpp
  kalman_filter/linear_kalman_filter/linear_2ws/linear_2ws_kalman_filter_debug.cpp
  kalman_filter/linear_kalman_filter/linear_4ws_fbs/linear_4ws_front_back_speed_kalman_filter.cpp
  kalman_filter/linear_kalman_filter/linear_4ws_fbs/linear_4ws_front_back_speed_kalman_filter_debug.cpp
  state.h
)


##Declare Node
add_executable(state_estimation
  $<TARGET_OBJECTS:state_estimation_objs>
  state_estimation_node.cpp
)

##Declare Nodelet
add_library(state_estimation_nodelet
  $<TARGET_OBJECTS:state_estimation_objs>
  state_estimation_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(state_estimation_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(state_estimation_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(state_estimation ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(state_estimation_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(state_estimation
  ${catkin_LIBRARIES}
)
target_link_libraries(state_estimation_nodelet
  ${catkin_LIBRARIES}
)
