include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_crosswalk EXCLUDE_FROM_ALL
  test_crosswalk.cpp
)

target_link_libraries(test_crosswalk
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_crosswalk ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_crosswalk)
endif()
