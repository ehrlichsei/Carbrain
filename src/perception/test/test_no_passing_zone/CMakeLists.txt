include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_no_passing_zone EXCLUDE_FROM_ALL
  test_no_passing_zone.cpp
)

target_link_libraries(test_no_passing_zone
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_no_passing_zone ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_no_passing_zone)
endif()
