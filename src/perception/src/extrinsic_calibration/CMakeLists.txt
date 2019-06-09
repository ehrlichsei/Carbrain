
##Declare Node
add_executable(extrinsic_calibration
  src/extrinsic_calibration/extrinsic_calibration.cpp
  src/extrinsic_calibration/extrinsic_calibration_node.cpp
)

##Declare Nodelet
add_library(extrinsic_calibration_nodelet
  src/extrinsic_calibration/extrinsic_calibration.cpp
  src/extrinsic_calibration/extrinsic_calibration_node.cpp
)
## set compiler definitions for nodelet
set_target_properties(extrinsic_calibration_nodelet PROPERTIES COMPILE_DEFINITIONS "BUILD_NODELET")

add_dependencies(extrinsic_calibration ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(extrinsic_calibration_nodelet ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(extrinsic_calibration
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(extrinsic_calibration_nodelet
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)
