if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  set(perpendicular_parking_no_debug "TRUE")
endif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")

add_library(perpendicular_parking_objs OBJECT
    src/perpendicular_parking/perpendicular_parking.cpp
    src/perpendicular_parking/parking_spot.cpp
    src/perpendicular_parking/parking_lot.cpp
    src/perpendicular_parking/parking_start_classifier.cpp
    src/perpendicular_parking/parking_end_classifier.cpp
#    src/perpendicular_parking/candidate_line.cpp
    src/perpendicular_parking/parking_line.cpp
)


##Declare Node
add_executable(perpendicular_parking
  src/perpendicular_parking/perpendicular_parking_node.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>
)

##Declare Nodelet
add_library(perpendicular_parking_nodelet
  src/perpendicular_parking/perpendicular_parking_node.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>
)

## set compiler definitions for nodelet
set_target_properties(perpendicular_parking_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(perpendicular_parking_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(perpendicular_parking ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(perpendicular_parking_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(perpendicular_parking
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  utils
)

target_link_libraries(perpendicular_parking_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  utils
)

if(NOT perpendicular_parking_no_debug)
  add_library(perpendicular_parking_debug_lib STATIC
    src/perpendicular_parking/debug/parking_end_classifier_debug.cpp
    src/perpendicular_parking/debug/perpendicular_parking_node_debug.cpp
    src/perpendicular_parking/debug/perpendicular_parking_debug.cpp
    src/perpendicular_parking/debug/parking_lot_debug.cpp
    src/perpendicular_parking/debug/parking_start_classifier_debug.cpp
  )

  add_dependencies(perpendicular_parking_debug_lib ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  target_link_libraries(perpendicular_parking
    perpendicular_parking_debug_lib
  )
else(NOT perpendicular_parking_no_debug)
  set_target_properties(perpendicular_parking PROPERTIES COMPILE_DEFINITIONS "NO_FANCY_DEBUG")
endif(NOT perpendicular_parking_no_debug)
