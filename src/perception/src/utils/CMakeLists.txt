add_library(utils
    src/utils/ego_vehicle.cpp
    # scan line tools
    src/utils/dbscan_clusterer.cpp
    src/utils/step_detection.cpp
    src/utils/foot_finder.cpp
)

add_library(birds_view_transformation
    src/utils/roi_birds_view_transformation.cpp
)

add_library(distinct_colors
    src/utils/distinct_colors.cpp
)

add_dependencies(birds_view_transformation ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(utils ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(birds_view_transformation ${catkin_LIBRARIES} ${common_LIBRARIES})
target_link_libraries(distinct_colors ${catkin_LIBRARIES} ${common_LIBRARIES})
target_link_libraries(utils birds_view_transformation distinct_colors ${catkin_LIBRARIES} ${common_LIBRARIES})
