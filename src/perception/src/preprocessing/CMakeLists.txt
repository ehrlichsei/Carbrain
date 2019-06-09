add_library(preprocessing_objs OBJECT 
  src/preprocessing/preprocessing.cpp
)

##Declare Node
add_executable(preprocessing
  src/preprocessing/preprocessing_node.cpp
  $<TARGET_OBJECTS:preprocessing_objs>
)

##Declare Nodelet
add_library(preprocessing_nodelet
  src/preprocessing/preprocessing_node.cpp
  $<TARGET_OBJECTS:preprocessing_objs>
)
## set compiler definitions for nodelet
set_target_properties(preprocessing_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(preprocessing_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(preprocessing utils ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(preprocessing_nodelet utils ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(preprocessing
  utils
  ${catkin_LIBRARIES}
)
target_link_libraries(preprocessing_nodelet
  utils
  ${catkin_LIBRARIES}
)

target_link_libraries(preprocessing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${common_LIBRARIES})
target_link_libraries(preprocessing_nodelet ${catkin_LIBRARIES} ${common_LIBRARIES} ${OpenCV_LIBRARIES})
