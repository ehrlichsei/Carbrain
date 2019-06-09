add_library(road_object_detection
  src/road_object_detection/road_objects/road_object.cpp
  # road_object_detection/road_object_visitors
  src/road_object_detection/road_object_visitors/message_handler.cpp
  src/road_object_detection/road_object_visitors/world_coordinates_helper.cpp
  # road_object_detection/features
  src/road_object_detection/features/features.cpp
  src/road_object_detection/features/feature_extractor.cpp
  # road_object_detection/classifiers
  src/road_object_detection/classifiers/crosswalk_classifier.cpp
  src/road_object_detection/classifiers/road_closure_classifier.cpp
  src/road_object_detection/classifiers/contour_classifier.cpp
  src/road_object_detection/classifiers/contour_classifier/contour_template.cpp
  src/road_object_detection/classifiers/contour_classifier/arrow_contour_template.cpp
  src/road_object_detection/classifiers/contour_classifier/speed_limit_contour_template.cpp
  src/road_object_detection/classifiers/contour_classifier/unclassified_contour_template.cpp
  src/road_object_detection/classifiers/contour_classifier/polygon.cpp
  src/road_object_detection/classifiers/junction_classifier.cpp
  src/road_object_detection/classifiers/start_line_classifier.cpp
  src/road_object_detection/classifiers/obstacle_classifier_new.cpp
  src/road_object_detection/classifiers/pedestrian_classifier.cpp
  src/road_object_detection/classifiers/no_passing_zone_classifier.cpp
  )

add_dependencies(road_object_detection ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(road_object_detection
  utils
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS})

add_library(road_object_detection_debug
  src/road_object_detection/debug/debug_images.cpp
  src/road_object_detection/debug/classifier_debug.cpp
  # road_object_detection/debug/features
  src/road_object_detection/debug/features/feature_extractor_debug.cpp
  # road_object_detection/debug/classifiers
  src/road_object_detection/debug/classifiers/crosswalk_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/road_closure_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/junction_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/contour_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/start_line_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/obstacle_classifier_new_debug.cpp
  src/road_object_detection/debug/classifiers/pedestrian_classifier_debug.cpp
  src/road_object_detection/debug/classifiers/no_passing_zone_classifier_debug.cpp
)

target_link_libraries(road_object_detection_debug
  road_object_detection
  ${catkin_LIBRARIES}
  ${common_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(road_object_detection_debug ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

