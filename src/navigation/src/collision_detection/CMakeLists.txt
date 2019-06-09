add_library(vehicle_obj OBJECT
  src/collision_detection/vehicle.cpp
)

add_library(collision_detection_objs OBJECT
  src/collision_detection/collision_detection.cpp
  src/collision_detection/safety_margin.cpp
)

##Declare Node
add_executable(collision_detection
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_obstacle_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:collision_detection_objs>
  $<TARGET_OBJECTS:navigation_environmental_model_objs>
  $<TARGET_OBJECTS:vehicle_obj>
  src/collision_detection/collision_detection_node.cpp
)

##Declare Nodelet
add_library(collision_detection_nodelet
  $<TARGET_OBJECTS:navigation_obstacle_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:collision_detection_objs>  
  $<TARGET_OBJECTS:navigation_environmental_model_objs>
  $<TARGET_OBJECTS:collision_detection_objs>
  $<TARGET_OBJECTS:vehicle_obj>
  src/collision_detection/collision_detection_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(collision_detection_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(collision_detection_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(collision_detection ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(collision_detection_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(collision_detection
  ${catkin_LIBRARIES}
)
target_link_libraries(collision_detection_nodelet
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(collision_detection_test
    test/test_collision_detection.cpp
    $<TARGET_OBJECTS:navigation_geometry_objs>
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
    $<TARGET_OBJECTS:navigation_environmental_model_objs>
    $<TARGET_OBJECTS:collision_detection_objs>
    $<TARGET_OBJECTS:vehicle_obj>
    $<TARGET_OBJECTS:navigation_obstacle_objs>
  )
  target_link_libraries(collision_detection_test
    ${catkin_LIBRARIES}
  )
endif()
