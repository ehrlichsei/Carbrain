catkin_add_gtest(foot_finder_test
  test/test_foot_finder.cpp
)

include_directories(include ${common_INCLUDE_DIRS})


target_link_libraries(foot_finder_test road_watcher_algorithm ${catkin_LIBRARIES})

#catkin_add_gtest(hungarian_algorithm_test
#  test/hungarian_algorithm_test.cpp
#)

#target_link_libraries(hungarian_algorithm_test road_watcher_algorithm ${catkin_LIBRARIES})


#catkin_add_gtest(road_watcher_object_tracker_test
#  test/road_watcher_object_tracker_test.cpp
#)

#target_link_libraries(road_watcher_object_tracker_test
#  road_watcher_algorithm
#  road_object_detection
#  ${catkin_LIBRARIES})

catkin_add_gtest(scan_line_test
  test/test_scan_line.cpp
)

target_link_libraries(scan_line_test road_watcher_algorithm ${catkin_LIBRARIES})


catkin_add_gtest(step_detection_test
  test/step_detection_test.cpp
)

target_link_libraries(step_detection_test road_watcher_algorithm ${catkin_LIBRARIES})

catkin_add_gtest(parkinglot_map_test
  test/test_parkinglot_map.cpp
  $<TARGET_OBJECTS:parkinglot_mapping_objs>
)

target_link_libraries(parkinglot_map_test utils ${catkin_LIBRARIES})

catkin_add_gtest(distinct_colors_test
  test/distinct_colors_test.cpp
)

target_link_libraries(distinct_colors_test utils ${catkin_LIBRARIES})

catkin_add_gtest(contour_classifier_test
  test/contour_classifier_test.cpp
)

add_dependencies(contour_classifier_test ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(contour_classifier_test
  road_object_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
)

#catkin_add_gtest(base_area_classification_test
#    test/test_base_area_classification.cpp
#)

#target_link_libraries(base_area_classification_test
#    road_watcher_algorithm
#    ${catkin_LIBRARIES}
#    ${common_LIBRARIES}
#)


find_package(rostest REQUIRED)
add_subdirectory(test/test_lane_detection)
add_subdirectory(test/test_road_watcher)
add_subdirectory(test/test_road_closure)
add_subdirectory(test/test_crosswalk)
add_subdirectory(test/test_speed_limit)
add_subdirectory(test/test_perpendicular_parking)
add_subdirectory(test/test_start_line)
add_subdirectory(test/test_junction)
add_subdirectory(test/test_arrow_and_junction)
add_subdirectory(test/test_qr_code_detection)
add_subdirectory(test/test_look_at)
add_subdirectory(test/test_no_passing_zone)

set(INTEGRATION_TESTS_URL "https://integration.kitcar-team.de/")

function(download_test_data TEST NAME)
catkin_download_test_data(${NAME}_bag
	${INTEGRATION_TESTS_URL}/${TEST}/${NAME}/${NAME}.bag
  FILENAME test/${TEST}/${NAME}/${NAME}.bag)
catkin_download_test_data(${NAME}_yaml
  ${INTEGRATION_TESTS_URL}/${TEST}/${NAME}/camera.yaml
  FILENAME test/${TEST}/${NAME}/camera.yaml)
endfunction(download_test_data)

download_test_data(test_lane_detection home_strecht_left_lane)
add_rostest(launch/tests/home_strecht_left_lane.test DEPENDENCIES test_lane_detection home_strecht_left_lane_bag home_strecht_left_lane_yaml)

download_test_data(test_lane_detection home_strecht_left_outer)
add_rostest(launch/tests/home_strecht_left_outer.test DEPENDENCIES test_lane_detection home_strecht_left_outer_bag home_strecht_left_outer_yaml)

download_test_data(test_lane_detection dashed_solid_middle_lines)
add_rostest(launch/tests/dashed_solid_middle_lines.test DEPENDENCIES test_lane_detection dashed_solid_middle_lines_bag dashed_solid_middle_lines_yaml)

download_test_data(test_lane_detection double_solid_middle_lines)
add_rostest(launch/tests/double_solid_middle_lines.test DEPENDENCIES test_lane_detection double_solid_middle_lines_bag double_solid_middle_lines_yaml)

download_test_data(test_lane_detection pedestrian_island_no_crosswalk)
add_rostest(launch/tests/pedestrian_island_no_crosswalk.test DEPENDENCIES test_lane_detection pedestrian_island_no_crosswalk_bag pedestrian_island_no_crosswalk_yaml)

download_test_data(test_lane_detection pedestrian_island_crosswalk)
add_rostest(launch/tests/pedestrian_island_crosswalk.test DEPENDENCIES test_lane_detection pedestrian_island_crosswalk_bag pedestrian_island_crosswalk_yaml)

download_test_data(test_lane_detection init_turn_void)
add_rostest(launch/tests/init_turn_void.test DEPENDENCIES test_lane_detection init_turn_void_bag init_turn_void_yaml)

download_test_data(test_road_watcher frontal_obstacle)
add_rostest(launch/tests/frontal_obstacle.test DEPENDENCIES test_road_watcher frontal_obstacle_bag frontal_obstacle_yaml)

download_test_data(test_road_watcher object_detection_false_positive)
add_rostest(launch/tests/object_detection_false_positive.test DEPENDENCIES test_road_watcher object_detection_false_positive_bag object_detection_false_positive_yaml)

download_test_data(test_road_watcher obstacle_near_new)
add_rostest(launch/tests/obstacle_near.test DEPENDENCIES test_road_watcher obstacle_near_new_bag obstacle_near_new_yaml)
#add_rostest(launch/tests/look_at.test DEPENDENCIES test_road_watcher)

download_test_data(test_road_closure road_closure_front)
add_rostest(launch/tests/road_closure_front.test DEPENDENCIES test_road_closure road_closure_front_bag road_closure_front_yaml)
#add_rostest(launch/tests/road_closure_left.test DEPENDENCIES test_road_closure)

download_test_data(test_road_closure no_road_closure)
add_rostest(launch/tests/no_road_closure.test DEPENDENCIES test_road_closure  no_road_closure_bag no_road_closure_yaml)

download_test_data(test_crosswalk crosswalk_pedestrians_crossing)
add_rostest(launch/tests/crosswalk_pedestrians_crossing.test DEPENDENCIES test_crosswalk crosswalk_pedestrians_crossing_bag crosswalk_pedestrians_crossing_yaml)

download_test_data(test_crosswalk crosswalk_no_pedestrians)
add_rostest(launch/tests/crosswalk_no_pedestrians.test DEPENDENCIES test_crosswalk crosswalk_no_pedestrians_bag crosswalk_no_pedestrians_yaml)

download_test_data(test_crosswalk crosswalk_misclassification_speed_limit)
add_rostest(launch/tests/crosswalk_misclassification_speed_limit.test DEPENDENCIES test_crosswalk crosswalk_misclassification_speed_limit_bag crosswalk_misclassification_speed_limit_yaml)

download_test_data(test_speed_limit 10)
add_rostest(launch/tests/speed_limit/10.test DEPENDENCIES test_speed_limit 10_bag 10_yaml)

download_test_data(test_speed_limit 20)
add_rostest(launch/tests/speed_limit/20.test DEPENDENCIES test_speed_limit 20_bag 20_yaml)

download_test_data(test_speed_limit 30)
add_rostest(launch/tests/speed_limit/30.test DEPENDENCIES test_speed_limit 30_bag 30_yaml)

download_test_data(test_speed_limit 40)
add_rostest(launch/tests/speed_limit/40.test DEPENDENCIES test_speed_limit 40_bag 40_yaml)

download_test_data(test_speed_limit 60_2)
add_rostest(launch/tests/speed_limit/60.test DEPENDENCIES test_speed_limit 60_2_bag 60_2_yaml)

download_test_data(test_speed_limit 70)
add_rostest(launch/tests/speed_limit/70.test DEPENDENCIES test_speed_limit 70_bag 70_yaml)

download_test_data(test_speed_limit 30x)
add_rostest(launch/tests/speed_limit/30x.test DEPENDENCIES test_speed_limit 30x_bag 30x_yaml)

download_test_data(test_speed_limit 70x)
add_rostest(launch/tests/speed_limit/70x.test DEPENDENCIES test_speed_limit 70x_bag 70x_yaml)

download_test_data(test_speed_limit 80x)
add_rostest(launch/tests/speed_limit/80x.test DEPENDENCIES test_speed_limit 80x_bag 80x_yaml)

download_test_data(test_start_line no_start_line_crosswalk)
add_rostest(launch/tests/no_start_line_crosswalk.test DEPENDENCIES test_start_line no_start_line_crosswalk_bag no_start_line_crosswalk_yaml)

download_test_data(test_start_line start_line_front_2)
add_rostest(launch/tests/start_line_front.test DEPENDENCIES test_start_line start_line_front_2_bag start_line_front_2_yaml)
add_rostest(launch/tests/start_line_front_not_all_classifiers.test DEPENDENCIES test_start_line start_line_front_2_bag start_line_front_2_yaml)

download_test_data(test_start_line no_start_line_give_way_line)
add_rostest(launch/tests/no_start_line_give_way_line.test DEPENDENCIES test_start_line no_start_line_give_way_line_bag no_start_line_give_way_line_yaml)

download_test_data(test_junction junction_stop_left)
add_rostest(launch/tests/junction_stop_left.test DEPENDENCIES test_junction junction_stop_left_bag junction_stop_left_yaml)

download_test_data(test_junction junction_stop_right)
add_rostest(launch/tests/junction_stop_right.test DEPENDENCIES test_junction junction_stop_right_bag junction_stop_right_yaml)

download_test_data(test_no_passing_zone no_passing_zone_no_clusters)
add_rostest(launch/tests/no_passing_zone_no_clusters.test DEPENDENCIES test_no_passing_zone no_passing_zone_no_clusters_bag no_passing_zone_no_clusters_yaml)

download_test_data(test_arrow_and_junction arrow_left)
add_rostest(launch/tests/arrow_and_junction/arrow_left.test DEPENDENCIES test_arrow_and_junction arrow_left_bag arrow_left_yaml)

download_test_data(test_arrow_and_junction arrow_right)
add_rostest(launch/tests/arrow_and_junction/arrow_right.test DEPENDENCIES test_arrow_and_junction arrow_right_bag arrow_right_yaml)
#add_rostest(launch/tests/arrow/arrow_right.test DEPENDENCIES test_arrow)

catkin_download_test_data(qr_code_detection_bag
  ${INTEGRATION_TESTS_URL}/test_qr_code_detection/stop.bag
  FILENAME test/test_qr_code_detection/stop.bag)
add_rostest(launch/tests/qr_code_detection.test DEPENDENCIES qr_code_detection_test qr_code_detection_bag)

download_test_data(test_perpendicular_parking free_parking_lot)
add_rostest(launch/tests/free_parking_lot.test DEPENDENCIES test_perpendicular_parking free_parking_lot_bag free_parking_lot_yaml)

download_test_data(test_perpendicular_parking startline_30deg)
add_rostest(launch/tests/parking_startline_30deg.test DEPENDENCIES test_perpendicular_parking startline_30deg_bag startline_30deg_yaml)

download_test_data(test_perpendicular_parking startline_40deg)
add_rostest(launch/tests/parking_startline_40deg.test DEPENDENCIES test_perpendicular_parking startline_40deg_bag startline_40deg_yaml)

download_test_data(test_perpendicular_parking startline_70deg)
add_rostest(launch/tests/parking_startline_70deg.test DEPENDENCIES test_perpendicular_parking startline_70deg_bag startline_70deg_yaml)

download_test_data(test_perpendicular_parking startline_80deg)
add_rostest(launch/tests/parking_startline_80deg.test DEPENDENCIES test_perpendicular_parking startline_80deg_bag startline_80deg_yaml)

download_test_data(test_perpendicular_parking startline_car)
add_rostest(launch/tests/parking_startline_car.test DEPENDENCIES test_perpendicular_parking startline_car_bag startline_car_yaml)

download_test_data(test_perpendicular_parking startline_crossing)
add_rostest(launch/tests/parking_startline_crossing.test DEPENDENCIES test_perpendicular_parking startline_crossing_bag startline_crossing_yaml)

download_test_data(test_look_at pedestrian_look_at_right)
add_rostest(launch/tests/pedestrian_look_at_right.test DEPENDENCIES test_look_at_pedestrians pedestrian_look_at_right_bag pedestrian_look_at_right_yaml)

download_test_data(test_look_at obstacles_look_at_road_closure_)
add_rostest(launch/tests/obstacles_look_at_road_closure.test DEPENDENCIES test_look_at_obstacles obstacles_look_at_road_closure__bag obstacles_look_at_road_closure__yaml)



catkin_add_gtest(dbscan_clusterer_test
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  test/dbscan_clusterer_test.cpp
  test/spiral.txt
)

target_link_libraries(dbscan_clusterer_test ${catkin_LIBRARIES} ${common_LIBRARIES} utils)

add_rostest(launch/tests/nodelet_load/load_preprocessing.test)
add_rostest(launch/tests/nodelet_load/load_lane_detection.test)
add_rostest(launch/tests/nodelet_load/load_road_watcher.test)
add_rostest(launch/tests/nodelet_load/load_pixelwise_mask_generation.test)
add_rostest(launch/tests/nodelet_load/load_parkinglot_mapping.test)
add_rostest(launch/tests/nodelet_load/load_perpendicular_parking.test)
add_rostest(launch/tests/nodelet_load/load_look_at.test)
