catkin_add_gtest(test_qr_code_detection
  ${CMAKE_CURRENT_SOURCE_DIR}/test_qr_code_detection.cpp
)

target_link_libraries(test_qr_code_detection qr_code_detection_lib ${catkin_LIBRARIES})
