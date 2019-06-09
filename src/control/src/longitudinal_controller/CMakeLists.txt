add_library(longitudinal_controller_objs OBJECT
  src/longitudinal_controller/longitudinal_control_params.cpp
  src/longitudinal_controller/concrete_longitudinal_controller.cpp
  src/longitudinal_controller/longitudinal_control.cpp
  src/longitudinal_controller/debug/longitudinal_controller_node_debug.cpp
  src/longitudinal_controller/debug/longitudinal_control_debug.cpp
  src/longitudinal_controller/debug/curvature_controller_debug.cpp
)

##Declare Node
add_executable(longitudinal_controller
  $<TARGET_OBJECTS:longitudinal_controller_objs>
  src/longitudinal_controller/longitudinal_controller_node.cpp
)

##Declare Nodelet
add_library(longitudinal_controller_nodelet
  $<TARGET_OBJECTS:longitudinal_controller_objs>
  src/longitudinal_controller/longitudinal_controller_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(longitudinal_controller_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(longitudinal_controller_objs
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
add_dependencies(longitudinal_controller
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
add_dependencies(longitudinal_controller_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(longitudinal_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(longitudinal_controller_nodelet
  ${catkin_LIBRARIES}
)
