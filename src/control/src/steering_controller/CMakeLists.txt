add_library(steering_controller_objs OBJECT
  src/steering_controller/steering_controller.cpp
)

##Declare Node
add_executable(steering_controller
  $<TARGET_OBJECTS:steering_controller_objs>
  src/steering_controller/steering_controller_node.cpp
  src/steering_controller/steering_controller_node_debug.cpp
)

##Declare Nodelet
add_library(steering_controller_nodelet
  $<TARGET_OBJECTS:steering_controller_objs>
  src/steering_controller/steering_controller_node.cpp
  src/steering_controller/steering_controller_node_debug.cpp
)

## set compiler definitions for nodelet
set_target_properties(steering_controller_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(steering_controller_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(steering_controller ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(steering_controller_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(steering_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(steering_controller_nodelet
  ${catkin_LIBRARIES}
)
