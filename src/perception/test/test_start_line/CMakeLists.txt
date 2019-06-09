include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_start_line EXCLUDE_FROM_ALL
  test_start_line.cpp
)

target_link_libraries(test_start_line
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_start_line ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_start_line)
endif()
