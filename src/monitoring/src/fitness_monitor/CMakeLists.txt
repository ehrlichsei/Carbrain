add_library(fitness_monitor_objs OBJECT
  src/fitness_monitor/round_detection.cpp
  src/fitness_monitor/fitness_calculator.cpp
  src/fitness_monitor/fitness_monitor.cpp
)

##Declare Node
add_executable(fitness_monitor
  $<TARGET_OBJECTS:fitness_monitor_objs>
  src/fitness_monitor/fitness_monitor_node.cpp
)

##Declare Nodelet
add_library(fitness_monitor_nodelet
  $<TARGET_OBJECTS:fitness_monitor_objs>
  src/fitness_monitor/fitness_monitor_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(fitness_monitor_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(fitness_monitor_objs
#  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(fitness_monitor
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(fitness_monitor_nodelet
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fitness_monitor
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
target_link_libraries(fitness_monitor_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
