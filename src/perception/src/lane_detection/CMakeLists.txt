if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(lane_detection_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

add_library(lane_detection_objs OBJECT
  src/lane_detection/vehicle_point_filter.cpp
  src/lane_detection/lane_detection.cpp
  src/lane_detection/feature_extraction.cpp
)

##Declare Node
add_executable(lane_detection
  src/lane_detection/lane_detection_node.cpp
  $<TARGET_OBJECTS:lane_detection_objs>
)

##Declare Nodelet
add_library(lane_detection_nodelet
  src/lane_detection/lane_detection_node.cpp
  $<TARGET_OBJECTS:lane_detection_objs>
)

## set compiler definitions for nodelet
set_target_properties(lane_detection_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(lane_detection_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(lane_detection utils ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(lane_detection_nodelet utils ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(lane_detection
  utils
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(lane_detection_nodelet
  utils
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if (NOT lane_detection_no_debug)
  add_library(lane_detection_debug_lib STATIC
    src/lane_detection/debug/feature_extraction_debug.cpp
    src/lane_detection/debug/lane_detection_debug.cpp
    src/lane_detection/debug/lane_detection_node_debug.cpp
  )

  add_dependencies(lane_detection_debug_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  target_link_libraries(lane_detection
    lane_detection_debug_lib
  )
else(NOT lane_detection_no_debug)
  set_target_properties(lane_detection PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT lane_detection_no_debug)
