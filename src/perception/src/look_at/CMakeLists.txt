if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(look_at_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

##Declare Node
add_library(look_at_algorithm
  src/look_at/look_at.cpp
  #servers
  src/look_at/servers/server_interface.cpp
  src/look_at/servers/obstacles_server.cpp
  src/look_at/servers/pedestrian_server.cpp
)

#Declare node
add_executable(look_at
  src/look_at/look_at_node.cpp
)

##Declare Nodelet
add_library(look_at_nodelet
  src/look_at/look_at_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(look_at_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(look_at_algorithm utils road_object_detection ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(look_at ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(look_at_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(look_at_algorithm
  road_object_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

target_link_libraries(look_at
  look_at_algorithm
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

target_link_libraries(look_at_nodelet
  look_at_algorithm
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

if(NOT look_at_no_debug)
  add_library(look_at_debug
  src/look_at/debug/look_at_node_debug.cpp
  src/look_at/debug/look_at_debug.cpp
  #servers
  src/look_at/debug/servers/debug_images_publisher.cpp
  src/look_at/debug/servers/obstacles_server_debug.cpp
  src/look_at/debug/servers/pedestrian_server_debug.cpp
  )

target_link_libraries(look_at_debug
  look_at_algorithm
  road_object_detection_debug
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  )

add_dependencies(look_at_debug ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(look_at
  look_at_debug
)

else(NOT look_at_no_debug)
  set_target_properties(look_at PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT look_at_no_debug)
