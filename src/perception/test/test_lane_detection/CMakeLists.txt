include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_lane_detection EXCLUDE_FROM_ALL
  test_lane_detection.cpp
)

target_link_libraries(test_lane_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_lane_detection ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_lane_detection)
endif()
