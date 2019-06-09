include_directories(${GTEST_INCLUDE_DIRS})

### LIBRARY
add_library(parking_lot_generator EXCLUDE_FROM_ALL
  gtests/parking_lot_generator.cpp
)
add_dependencies(parking_lot_generator ${catkin_EXPORTED_TARGETS})



target_link_libraries(parking_lot_generator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

### ROSTESTS
add_executable(test_perpendicular_parking EXCLUDE_FROM_ALL
        test_perpendicular_parking.cpp
        )

target_link_libraries(test_perpendicular_parking
        ${catkin_LIBRARIES}
        ${common_LIBRARIES}
        ${GTEST_LIBRARIES}
        )


### GTESTS
catkin_add_gtest(perpendicular_parking_dummy_test
  gtests/test_perpendicular_parking.cpp
)
target_link_libraries(perpendicular_parking_dummy_test
  parking_lot_generator
  utils
  ${catkin_LIBRARIES})

catkin_add_gtest(perpendicular_parking_lot_test
  gtests/test_parking_lot.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>
)
target_link_libraries(perpendicular_parking_lot_test
  parking_lot_generator
  utils
  ${catkin_LIBRARIES})

catkin_add_gtest(parking_start_classifier_test
  gtests/parking_start_classifier_test.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>
)
target_link_libraries(parking_start_classifier_test
  parking_lot_generator
  utils
  ${catkin_LIBRARIES})

catkin_add_gtest(parking_end_classifier_test
  gtests/parking_end_classifier_test.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>  
)

target_link_libraries(parking_end_classifier_test 
  parking_lot_generator
  utils 
  ${common_LIBRARIES} 
  ${catkin_LIBRARIES})

catkin_add_gtest(perpendicular_parking_spot_test
  gtests/test_parking_spot.cpp
  $<TARGET_OBJECTS:perpendicular_parking_objs>
)
target_link_libraries(perpendicular_parking_spot_test
  utils 
  ${common_LIBRARIES} 
  ${catkin_LIBRARIES})

