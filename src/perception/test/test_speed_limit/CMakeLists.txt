include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_speed_limit EXCLUDE_FROM_ALL
  test_speed_limit.cpp
)

target_link_libraries(test_speed_limit
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_speed_limit ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_speed_limit)
endif()
