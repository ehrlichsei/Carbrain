cmake_minimum_required(VERSION 2.8.3)
project(control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   lateral_controller_msgs
   longitudinal_controller_msgs
   message_generation
   tf
   tf_conversions
   tf2_ros
   tf2_eigen
   common
   common_msgs 
   state_estimation_msgs
   navigation_msgs
   longitudinal_controller_msgs
   lateral_controller_msgs
   dynamic_reconfigure
   cmake_modules
   control_toolbox
   geometry_msgs
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#checking launch files when compiling
find_package(roslaunch)
#roslaunch_add_file_check(launch) #TODO: reactivate roslaunch checks for control when https://github.com/ros/ros_comm/issues/993 is fixed

find_package(kitcar_cmake)
add_files_to_project()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#########################################
## Declare dynamic reconfigure options ##
#########################################

generate_dynamic_reconfigure_options(
  cfg/SpeedController.cfg
  cfg/SteeringController.cfg
  cfg/LongitudinalController.cfg
  cfg/LateralController.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES control
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Nodes ##
###########

INCLUDE(src/steering_controller/CMakeLists.txt)

INCLUDE(src/speed_controller/CMakeLists.txt)

INCLUDE(src/lateral_controller/CMakeLists.txt)

INCLUDE(src/longitudinal_controller/CMakeLists.txt)

###################
## Documentation ##
###################

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
find_program(ROSDOC_LITE rosdoc_lite)
if(DOXYGEN_FOUND AND ROSDOC_LITE)
  add_custom_target(doc-control #ALL
    rosdoc_lite ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
    COMMENT "Generating API documentation with rosdoc_lite" VERBATIM
    )
endif(DOXYGEN_FOUND AND ROSDOC_LITE)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS control control_node
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
  find_package(rostest REQUIRED)
  #add_rostest_gtest(test_high_level_controller launch/high_level_controller.test test/high_level_controller/test_high_level_controller.cpp)
  #target_link_libraries(test_high_level_controller ${catkin_LIBRARIES})
  #add_dependencies(test_high_level_controller ${catkin_EXPORTED_TARGETS})

  add_rostest(launch/test/load_speed_controller.launch)
  add_rostest(launch/test/load_steering_controller.launch)
  add_rostest(launch/test/load_longitudinal_controller.launch)
  add_rostest(launch/test/load_lateral_controller.launch)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_estimation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
