include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_arrow_and_junction EXCLUDE_FROM_ALL
  test_arrow_and_junction.cpp
)

target_link_libraries(test_arrow_and_junction
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_arrow_and_junction ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_arrow_and_junction)
endif()
