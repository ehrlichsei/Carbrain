add_library(diagnostic_monitor_objs OBJECT
  src/diagnostic_monitor/diagnostic_monitor.cpp
)

##Declare Node
add_executable(diagnostic_monitor
  $<TARGET_OBJECTS:diagnostic_monitor_objs>
  src/diagnostic_monitor/diagnostic_monitor_node.cpp
)

##Declare Nodelet
add_library(diagnostic_monitor_nodelet
  src/diagnostic_monitor/diagnostic_monitor_node.cpp
  $<TARGET_OBJECTS:diagnostic_monitor_objs>
)
## set compiler definitions for nodelet
set_target_properties(diagnostic_monitor_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(diagnostic_monitor ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(diagnostic_monitor_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(diagnostic_monitor_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(diagnostic_monitor
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
target_link_libraries(diagnostic_monitor_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(diagnostic_monitor_test
    $<TARGET_OBJECTS:diagnostic_monitor_objs>
    test/test_diagnostic_monitor.cpp
  )

  target_link_libraries(diagnostic_monitor_test
    ${catkin_LIBRARIES}
    ${common_LIBRARIES}
  )
endif()
