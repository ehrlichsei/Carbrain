cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  tf2_eigen
  tf2_ros
  smach
  mission_control_msgs
  longitudinal_controller_msgs
  navigation_msgs
  perception_msgs
  common_msgs
  cmake_modules
  smach_ros
  common
  navigation
  controller_msgs
  eigen_conversions
  boost_sml_catkin
)

find_package(OpenMP REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#checking launch files when compiling
find_package(roslaunch)
roslaunch_add_file_check(launch)

find_package(kitcar_cmake)
add_files_to_project()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartParkingDiscipline.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
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
#   INCLUDE_DIRS include
#  LIBRARIES mission_control
  CATKIN_DEPENDS roscpp std_msgs common_msgs longitudinal_controller_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${navigation_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

###########
## Nodes ##
###########

add_subdirectory(src/mission_controller)
add_subdirectory(src/pavlov)

## Declare a cpp library
# add_library(mission_control
#   src/${PROJECT_NAME}/mission_control.cpp
# )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

###################
## Documentation ##
###################

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
find_program(ROSDOC_LITE rosdoc_lite)
if(DOXYGEN_FOUND AND ROSDOC_LITE)
  add_custom_target(doc-mission-control #ALL
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
# install(TARGETS mission_control mission_controller
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
  add_rostest_gtest(test_pavlov launch/pavlov.test test/pavlov/test_pavlov.cpp)
  target_link_libraries(test_pavlov ${catkin_LIBRARIES})
  add_dependencies(test_pavlov ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} obstacle_pavlov)

  catkin_add_gtest(driving_corridor_checker_test
    src/pavlov/node_helper/driving_corridor_checker.cpp
    test/pavlov/test_driving_corridor_checker.cpp
  )
  add_dependencies(driving_corridor_checker_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(driving_corridor_checker_test ${catkin_LIBRARIES})

  catkin_add_gtest(fsm_test
    src/pavlov/fsm_utils/timeout.cpp
    test/pavlov/fsm/test_fsm.cpp
    src/pavlov/node_helper/driving_corridor_checker.cpp
    src/pavlov/obstacle_fsm/shared_data.cpp
  )
  add_dependencies(fsm_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(fsm_test ${catkin_LIBRARIES})

  catkin_add_gtest(timeout_test
    src/pavlov/fsm_utils/timeout.cpp
    test/pavlov/fsm/test_timeout.cpp
  )
  target_link_libraries(timeout_test ${catkin_LIBRARIES})

  add_rostest(launch/test/load_mission_controller.test)
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mission_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()
