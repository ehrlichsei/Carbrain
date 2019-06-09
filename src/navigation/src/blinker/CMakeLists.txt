add_library(blinker_objs OBJECT
  src/blinker/blinker.cpp
)

##Declare Node
add_executable(blinker
  $<TARGET_OBJECTS:blinker_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  src/blinker/blinker_node.cpp
)

##Declare Nodelet
add_library(blinker_nodelet
  $<TARGET_OBJECTS:blinker_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  src/blinker/blinker_node.cpp
)

## set compiler definitions for nodelet
set_target_properties(blinker_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(blinker_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(blinker ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(blinker_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(blinker
  ${catkin_LIBRARIES}
)
target_link_libraries(blinker_nodelet
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(blinker_test
    $<TARGET_OBJECTS:blinker_objs>  
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
    $<TARGET_OBJECTS:navigation_geometry_objs>
    test/test_blinker.cpp
  )

  target_link_libraries(blinker_test
    ${catkin_LIBRARIES}
  )
endif()
