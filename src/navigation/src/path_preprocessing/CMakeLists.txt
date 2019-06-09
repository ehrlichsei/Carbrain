add_library(gate_utils_objs OBJECT
  gate_utils.cpp
)
add_dependencies(gate_utils_objs ${catkin_EXPORTED_TARGETS})

add_library(lane_utils_objs OBJECT
  lane_utils.cpp
)
add_dependencies(lane_utils_objs ${catkin_EXPORTED_TARGETS})

add_library(path_preprocessing_objs OBJECT
  path_preprocessing.cpp
  pathutils.cpp
  trackedlane.cpp
)

##Declare Node
add_executable(path_preprocessing
  $<TARGET_OBJECTS:gate_utils_objs>
  $<TARGET_OBJECTS:lane_utils_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:path_preprocessing_objs>
  path_preprocessing_node.cpp
)

##Declare Nodelet
add_library(path_preprocessing_nodelet
  $<TARGET_OBJECTS:gate_utils_objs>
  $<TARGET_OBJECTS:lane_utils_objs>
  $<TARGET_OBJECTS:path_preprocessing_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  path_preprocessing_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(path_preprocessing_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(path_preprocessing_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(path_preprocessing ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(path_preprocessing_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_preprocessing
  ${catkin_LIBRARIES}
)
target_link_libraries(path_preprocessing_nodelet
  ${catkin_LIBRARIES}
)

