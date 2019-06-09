if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(path_planning_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

add_library(path_planning_objs OBJECT
  path_planner.cpp
  middle_planner.cpp
  least_squares_planner.cpp
)

##Declare Node
add_executable(path_planning
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:path_planning_objs>
  path_planning_node.cpp
)

##Declare Nodelet
add_library(path_planning_nodelet
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:path_planning_objs>
  path_planning_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(path_planning_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(path_planning_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(path_planning ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(path_planning_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_planning
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
#  ${CERES_LIBRARIES}
)
target_link_libraries(path_planning_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
#  ${CERES_LIBRARIES}
)

if(NOT path_planning_no_debug)
  add_library(path_planning_debug
    debug/path_planning_node_debug.cpp
    debug/least_squares_planner_debug.cpp
  )

  target_link_libraries(path_planning_debug
    ${catkin_LIBRARIES}
    ${common_LIBRARIES}
  )

  add_dependencies(path_planning_debug ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  target_link_libraries(path_planning
    path_planning_debug
  )
else(NOT path_planning_no_debug)
  set_target_properties(path_planning PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT path_planning_no_debug)
