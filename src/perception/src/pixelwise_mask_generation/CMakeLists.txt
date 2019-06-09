add_library(pixelwise_mask_generation_objs OBJECT
  src/pixelwise_mask_generation/pixelwise_mask_generation.cpp
)

##Declare Node
add_executable(pixelwise_mask_generation
  $<TARGET_OBJECTS:pixelwise_mask_generation_objs>
  src/pixelwise_mask_generation/pixelwise_mask_generation_node.cpp
)

##Declare Nodelet
add_library(pixelwise_mask_generation_nodelet
  src/pixelwise_mask_generation/pixelwise_mask_generation_node.cpp
  src/pixelwise_mask_generation/pixelwise_mask_generation.cpp
)

## set compiler definitions for nodelet
set_target_properties(pixelwise_mask_generation_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(pixelwise_mask_generation_objs ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(pixelwise_mask_generation ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(pixelwise_mask_generation_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(pixelwise_mask_generation
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(pixelwise_mask_generation_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
