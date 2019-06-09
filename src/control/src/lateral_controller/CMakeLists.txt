add_library(lateral_controller_objs OBJECT
  src/lateral_controller/lateral_control.cpp
)

##Declare Node
add_executable(lateral_controller
  $<TARGET_OBJECTS:lateral_controller_objs>
  src/lateral_controller/lateral_controller_node.cpp
)

##Declare Nodelet
add_library(lateral_controller_nodelet
  $<TARGET_OBJECTS:lateral_controller_objs>
  src/lateral_controller/lateral_controller_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(lateral_controller_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(lateral_controller_objs
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
add_dependencies(lateral_controller
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
add_dependencies(lateral_controller_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lateral_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(lateral_controller_nodelet
  ${catkin_LIBRARIES}
)
