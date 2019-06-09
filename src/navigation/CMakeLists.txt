cmake_minimum_required(VERSION 2.8.3)
project(navigation C CXX)

find_package(Protobuf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  rospy
  std_msgs
  tf2_ros
  tf2
  tf
  common_msgs
  controller_msgs
  longitudinal_controller_msgs
  lateral_controller_msgs
  geometry_msgs
  tf2_geometry_msgs
  state_estimation_msgs
  perception_msgs
  navigation_msgs
  lateral_controller_msgs
  longitudinal_controller_msgs
  sensor_msgs
  dynamic_reconfigure
  cmake_modules
  nav_msgs
  common
  eigen_conversions
  ceres_catkin
  rostest
  tf2_eigen
)

find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

#checking launch files when compiling
find_package(roslaunch)
roslaunch_add_file_check(launch)

find_package(kitcar_cmake)
add_files_to_project()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#########################################
## Declare dynamic reconfigure options ##
#########################################

generate_dynamic_reconfigure_options(
  cfg/CollisionDetection.cfg
  cfg/PathPlanning.cfg
  cfg/Localization.cfg
  cfg/PathPreprocessing.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driving_corridor obstacle no_passing_zone
  CATKIN_DEPENDS roscpp std_msgs navigation_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
#  ${CERES_INCLUDE_DIRS}
)

###########
## Nodes ##
###########

INCLUDE(src/environmental_model/CMakeLists.txt)

INCLUDE(src/geometry/CMakeLists.txt)
INCLUDE(src/driving_corridor/CMakeLists.txt)
INCLUDE(src/obstacle/CMakeLists.txt)
INCLUDE(src/parkinglot_aligner/CMakeLists.txt)

INCLUDE(src/blinker/CMakeLists.txt)

INCLUDE(src/localization/CMakeLists.txt)

INCLUDE(src/collision_detection/CMakeLists.txt)

add_subdirectory(src/path_planning)
add_subdirectory(src/path_preprocessing)

add_library(driving_corridor
  $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  $<TARGET_OBJECTS:navigation_geometry_objs>
)

add_dependencies(driving_corridor
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(driving_corridor
  ${catkin_LIBRARIES}
)

add_library(obstacle
  src/obstacle/obstacle.cpp
)

add_dependencies(obstacle
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(obstacle
  ${catkin_LIBRARIES}
)


add_library(no_passing_zone
  src/environmental_model/no_passing_zone.cpp
)

add_dependencies(no_passing_zone
  ${catkin_EXPORTED_TARGETS}
  navigation_msgs_gencpp
)

target_link_libraries(no_passing_zone
  ${catkin_LIBRARIES}
  )

add_library(parking_objs OBJECT
  src/parking/parkingorchestrator.cpp
  src/parking/parkingcarinterface.cpp
  src/parking/parkingservice.cpp
  src/parking/rosparkingdebugutil.cpp
  src/parking/parkingpathsolver.cpp
)

add_library(find_parking_slot_objs OBJECT
  src/find_parking_slot/parkingslotfinder.cpp
  src/find_parking_slot/onedparkinglotmapper.cpp
)

## Declare a cpp executable
add_executable(parking
  src/parking/main.cpp
  $<TARGET_OBJECTS:parking_objs>
)
add_executable(find_parking_slot
  src/find_parking_slot/main.cpp
  $<TARGET_OBJECTS:find_parking_slot_objs>
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(parking_objs
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(find_parking_slot_objs
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against

target_link_libraries(find_parking_slot
  ${catkin_LIBRARIES}
)

target_link_libraries(parking
  ${catkin_LIBRARIES}
  boost_thread
)

###################
## Documentation ##
###################

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
find_program(ROSDOC_LITE rosdoc_lite)
if(DOXYGEN_FOUND AND ROSDOC_LITE)
  add_custom_target(doc-navigation #ALL
    rosdoc_lite ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
    COMMENT "Generating API documentation with rosdoc_lite" VERBATIM
    )
endif(DOXYGEN_FOUND AND ROSDOC_LITE)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS navigation navigation_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_parking_solver.cpp
    src/parking/parkingpathsolver.cpp
  )

  catkin_add_gtest(obstacle_test
    test/test_obstacle.cpp
  )

  target_link_libraries(obstacle_test
    obstacle
  )

  catkin_add_gtest(gate_test
    test/test_gate.cpp
    $<TARGET_OBJECTS:navigation_geometry_objs>
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
  )

  catkin_add_gtest(gate_utils_test
    test/test_gate_utils.cpp
    $<TARGET_OBJECTS:gate_utils_objs>
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
    $<TARGET_OBJECTS:navigation_geometry_objs>
  )

  catkin_add_gtest(driving_corridor_test
    test/test_driving_corridor.cpp
    $<TARGET_OBJECTS:navigation_driving_corridor_objs>
    $<TARGET_OBJECTS:navigation_geometry_objs>
  )
  if(TARGET driving_corridor_test)
    target_link_libraries(driving_corridor_test
      ${catkin_LIBRARIES}
  #    ${CERES_LIBRARIES}
    )
  endif()

  catkin_add_gtest(line_segment_test
    test/test_line_segment.cpp
    $<TARGET_OBJECTS:navigation_geometry_objs>
  )

  catkin_add_gtest(triangle_test
    test/test_triangle.cpp
    $<TARGET_OBJECTS:navigation_geometry_objs>
  )

  catkin_add_gtest(vehicle_test
    test/test_vehicle.cpp
    $<TARGET_OBJECTS:vehicle_obj>
    $<TARGET_OBJECTS:navigation_geometry_objs>
  )

  catkin_add_gtest(one_d_parkinglotmapper_test
    test/test_one_d_parkinglot_mapper.cpp
    src/find_parking_slot/onedparkinglotmapper.cpp
  )

  catkin_add_gtest(boundingbox_test
    test/test_boundingbox.cpp
  )

  add_rostest(launch/tests/nodelet_load/load_localization.test)
  add_rostest(launch/tests/nodelet_load/load_blinker.test)
  add_rostest(launch/tests/nodelet_load/load_path_preprocessing.test)
  add_rostest(launch/tests/nodelet_load/load_path_planning.test)
  add_rostest(launch/tests/nodelet_load/load_parkinglot_aligner.test)
  add_rostest(launch/tests/nodelet_load/load_collision_detection.test)

  # add_rostest(launch/tests/path_planning.test DEPENDENCIES test/test_node_path_planning.cpp) #proposed by jonas.walter didn't work
  add_rostest_gtest(test_path_planning
    launch/tests/path_planning.test
    test/test_node_path_planning.cpp
  )
  target_link_libraries(test_path_planning ${catkin_LIBRARIES} driving_corridor)

  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_navigation.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # catkin_add_nosetests(test)
endif(CATKIN_ENABLE_TESTING)

