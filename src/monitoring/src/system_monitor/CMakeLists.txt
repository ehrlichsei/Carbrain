
##Declare Node
add_executable(system_monitor
  src/system_monitor/system_monitor.cpp
  src/system_monitor/system_monitor_node.cpp
  #src/self_tests.cpp
)

##Declare Nodelet
add_library(system_monitor_nodelet
  src/system_monitor/system_monitor.cpp
  src/system_monitor/system_monitor_node.cpp
  # src/self_tests.cpp
)
## set compiler definitions for nodelet
set_target_properties(system_monitor_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(system_monitor
  #${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(system_monitor_nodelet
  #${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(system_monitor
  ${catkin_LIBRARIES}
  boost_iostreams
)
target_link_libraries(system_monitor_nodelet
  ${catkin_LIBRARIES}
  boost_iostreams
)
