add_library(parkinglot_aligner_objs OBJECT
  src/parkinglot_aligner/parkinglot_aligner.cpp
)

##Declare Node
add_executable(parkinglot_aligner
  src/parkinglot_aligner/parkinglot_aligner_node.cpp
  $<TARGET_OBJECTS:parkinglot_aligner_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
)

##Declare Nodelet
add_library(parkinglot_aligner_nodelet
  src/parkinglot_aligner/parkinglot_aligner_node.cpp
  $<TARGET_OBJECTS:parkinglot_aligner_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
)
## set compiler definitions for nodelet
set_target_properties(parkinglot_aligner_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(parkinglot_aligner ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(parkinglot_aligner_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(parkinglot_aligner_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(parkinglot_aligner
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)
target_link_libraries(parkinglot_aligner_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(parkinglot_aligner_test
    test/test_parkinglot_aligner.cpp
    $<TARGET_OBJECTS:parkinglot_aligner_objs>
    $<TARGET_OBJECTS:navigation_geometry_objs>
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  )
  target_link_libraries(parkinglot_aligner_test
    ${catkin_LIBRARIES}
  )
endif()
