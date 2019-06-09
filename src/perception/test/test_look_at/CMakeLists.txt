include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_look_at_pedestrians EXCLUDE_FROM_ALL
  test_look_at_pedestrians.cpp
)

target_link_libraries(test_look_at_pedestrians
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_look_at_pedestrians ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_look_at_pedestrians)
endif()

add_executable(test_look_at_obstacles EXCLUDE_FROM_ALL
  test_look_at_obstacles.cpp
)

target_link_libraries(test_look_at_obstacles
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(test_look_at_obstacles ${catkin_EXPORTED_TARGETS})

if(TARGET tests)
  add_dependencies(tests test_look_at_obstacles)
endif()
