if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(road_watcher_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

add_library(road_watcher_algorithm
  # road_watcher
  src/road_watcher/classification.cpp
  src/road_watcher/road_watcher.cpp
)


##Declare Node
add_executable(road_watcher
  src/road_watcher/road_watcher_node.cpp
)

##Declare Nodelet
add_library(road_watcher_nodelet
  src/road_watcher/road_watcher_node.cpp
)

## set compiler definitions for nodelet
set_target_properties(road_watcher_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(road_watcher_algorithm utils road_object_detection ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(road_watcher ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(road_watcher_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(road_watcher_algorithm
  utils
  road_object_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)


target_link_libraries(road_watcher
  road_watcher_algorithm
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(road_watcher_nodelet
  road_watcher_algorithm
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)

if(NOT road_watcher_no_debug)
  add_library(road_watcher_debug
    # road_watcher/debug
    src/road_watcher/debug/classification_debug.cpp
    src/road_watcher/debug/road_watcher_debug.cpp
    src/road_watcher/debug/road_watcher_node_debug.cpp
  )

  target_link_libraries(road_watcher_debug
    road_watcher_algorithm
    road_object_detection_debug
    ${catkin_LIBRARIES}
    ${common_LIBRARIES}
    ${OpenCV_LIBS}
  )

  add_dependencies(road_watcher_debug ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  target_link_libraries(road_watcher
    road_watcher_debug
  )
else(NOT road_watcher_no_debug)
  set_target_properties(road_watcher PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT road_watcher_no_debug)
