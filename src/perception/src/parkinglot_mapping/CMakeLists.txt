add_library(parkinglot_mapping_objs OBJECT
  src/parkinglot_mapping/parkinglot_map.cpp
  src/parkinglot_mapping/parkinglot_mapping.cpp
)

##Declare Node
add_executable(parkinglot_mapping
  $<TARGET_OBJECTS:parkinglot_mapping_objs>
  src/parkinglot_mapping/parkinglot_mapping_node.cpp
)

##Declare Nodelet
add_library(parkinglot_mapping_nodelet
  $<TARGET_OBJECTS:parkinglot_mapping_objs>
  src/parkinglot_mapping/parkinglot_mapping_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(parkinglot_mapping_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

target_link_libraries(parkinglot_mapping birds_view_transformation
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

target_link_libraries(parkinglot_mapping_nodelet birds_view_transformation
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

add_dependencies(parkinglot_mapping_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(parkinglot_mapping ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(parkinglot_mapping_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
