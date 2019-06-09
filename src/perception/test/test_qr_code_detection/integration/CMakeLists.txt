add_executable(qr_code_detection_test EXCLUDE_FROM_ALL
  ${CMAKE_CURRENT_SOURCE_DIR}/test_qr_code_detection.cpp
)

target_link_libraries(qr_code_detection_test
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(qr_code_detection_test ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests qr_code_detection_test)
endif()

