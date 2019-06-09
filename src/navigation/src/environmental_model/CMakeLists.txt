
add_library(navigation_environmental_model_objs OBJECT
  src/environmental_model/no_passing_zone.cpp
  src/environmental_model/tracking_element.cpp
  src/environmental_model/environmental_model.cpp
  src/environmental_model/road_elements/road_closure.cpp
  src/environmental_model/road_elements/crosswalk.cpp
  src/environmental_model/road_elements/junction.cpp
  src/environmental_model/no_passing_zone.cpp
  src/environmental_model/road_elements/speed_limit.cpp
  src/environmental_model/road_elements/start_line.cpp
  src/environmental_model/road_elements/road_element.cpp
  src/environmental_model/road_elements/arrow.cpp
  src/environmental_model/road_elements/obstacle.cpp
  src/environmental_model/road_elements/obstacle_tracker/obstacle_tracker.cpp
  src/environmental_model/road_elements/obstacle_tracker/obstacle_model.cpp
  src/environmental_model/road_elements/obstacle_tracker/static_obstacle_model.cpp
  src/environmental_model/road_elements/obstacle_tracker/lane_obstacle_model.cpp
  src/environmental_model/road_elements/unidentified.cpp
  src/environmental_model/object_tracking/matching.cpp
  src/environmental_model/object_tracking/polygon_algorithm.cpp
  src/environmental_model/road_element_visitors/look_at_visitor.cpp
  src/environmental_model/road_element_visitors/look_at_result_visitor.cpp
  src/environmental_model/road_element_visitors/create_messages_visitor.cpp
  src/environmental_model/road_element_visitors/no_passing_zone_visitor.cpp
  src/environmental_model/road_element_visitors/consistency_check_visitor.cpp
)

add_dependencies(navigation_environmental_model_objs
  navigation_msgs_gencpp
)

##Declare Node
add_executable(environmental_model
  $<TARGET_OBJECTS:navigation_environmental_model_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  src/environmental_model/environmental_model_node.cpp
)

##Declare Nodelet
add_library(environmental_model_nodelet
  $<TARGET_OBJECTS:navigation_environmental_model_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  src/environmental_model/environmental_model_node.cpp
)

## set compiler definitions for nodelet
set_target_properties(environmental_model_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(environmental_model ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} navigation_driving_corridor_objs)
add_dependencies(environmental_model_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS} navigation_driving_corridor_objs)

target_link_libraries(environmental_model
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
target_link_libraries(environmental_model_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

catkin_add_gtest(environmental_model_test
  $<TARGET_OBJECTS:navigation_environmental_model_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  test/test_environmental_model.cpp
)

target_link_libraries(environmental_model_test
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
