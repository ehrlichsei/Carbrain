add_library(speed_controller_objs OBJECT
  src/speed_controller/speed_controller.cpp
)

##Declare Node
add_executable(speed_controller
  $<TARGET_OBJECTS:speed_controller_objs>
  src/speed_controller/speed_controller_node.cpp
  src/speed_controller/speed_controller_node_debug.cpp
)
  
##Declare Nodelet
add_library(speed_controller_nodelet
  $<TARGET_OBJECTS:speed_controller_objs>
  src/speed_controller/speed_controller_node.cpp
  src/speed_controller/speed_controller_node_debug.cpp
)
## set compiler definitions for nodelet
set_target_properties(speed_controller_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(speed_controller_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(speed_controller ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(speed_controller_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(speed_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(speed_controller_nodelet
  ${catkin_LIBRARIES}
)
