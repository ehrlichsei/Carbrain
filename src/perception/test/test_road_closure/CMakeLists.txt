include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_road_closure EXCLUDE_FROM_ALL
  test_road_closure.cpp
)

target_link_libraries(test_road_closure
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_road_closure ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_road_closure)
endif()
