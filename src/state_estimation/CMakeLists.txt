cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
            roscpp
            common
            common_msgs
            control
            controller_interface
            state_estimation_msgs
            controller_msgs
            cmake_modules
            dynamic_reconfigure
            rosbag_storage
            eigen_conversions
)


find_package(Eigen3 REQUIRED)
find_package(common REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#checking launch files when compiling
find_package(roslaunch)
roslaunch_add_file_check(launch)

find_package(kitcar_cmake)
add_files_to_project()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/StateEstimation.cfg
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
#  LIBRARIES state_estimation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
  ${controller_interface_INCLUDE_DIRS}
  ${control_INCLUDE_DIRS}
)

###########
## Nodes ##
###########

add_subdirectory(src/state_estimation)

## Declare a cpp library
# add_library(state_estimation
#   src/${PROJECT_NAME}/state_estimation.cpp
# )

## Declare a cpp executable
# add_executable(state_estimation_node src/state_estimation_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(state_estimation_node state_estimation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(state_estimation_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS state_estimation state_estimation_node
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
  set(INTEGRATION_TESTS_URL "https://integration.kitcar-team.de/")

  catkin_download_test_data(new_message_test_parking_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_parking.bag
    FILENAME test/test_state_estimation/new_message_test_parking.bag)
  
  catkin_download_test_data(new_message_test_straight_0_5_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_0_5.bag
    FILENAME test/test_state_estimation/new_message_test_straight_0_5.bag)

  catkin_download_test_data(new_message_test_straight_1_0_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_1_0.bag
    FILENAME test/test_state_estimation/new_message_test_straight_1_0.bag)

  catkin_download_test_data(new_message_test_straight_1_5_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_1_5.bag
    FILENAME test/test_state_estimation/new_message_test_straight_1_5.bag)

  catkin_download_test_data(new_message_test_straight_2_0_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_2_0.bag
    FILENAME test/test_state_estimation/new_message_test_straight_2_0.bag)

  catkin_download_test_data(new_message_test_straight_2_5_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_2_5.bag
    FILENAME test/test_state_estimation/new_message_test_straight_2_5.bag)

  catkin_download_test_data(new_message_test_straight_3_0_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_straight_3_0.bag
    FILENAME test/test_state_estimation/new_message_test_straight_3_0.bag)

  catkin_download_test_data(new_message_test_oval_slow_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_oval_slow.bag
    FILENAME test/test_state_estimation/new_message_test_oval_slow.bag)

  catkin_download_test_data(new_message_test_oval_medium_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_oval_medium.bag
    FILENAME test/test_state_estimation/new_message_test_oval_medium.bag)

  catkin_download_test_data(new_message_test_oval_fast_bag
    ${INTEGRATION_TESTS_URL}/test_state_estimation/new_message_test_oval_fast.bag
    FILENAME test/test_state_estimation/new_message_test_oval_fast.bag)

  catkin_download_test_data(test_state_estimation_car_specs
    ${INTEGRATION_TESTS_URL}/test_state_estimation/car_specs.yaml
    FILENAME test/test_state_estimation/car_specs.yaml)

  add_rostest_gtest(test_state_estimation launch/state_estimation.test test/state_estimation/test_state_estimation.cpp)
  target_link_libraries(test_state_estimation ${catkin_LIBRARIES})
  add_dependencies(test_state_estimation ${catkin_EXPORTED_TARGETS})
  add_dependencies(run_tests_state_estimation_rostest_launch_state_estimation.test
                   new_message_test_parking_bag
                   new_message_test_straight_0_5_bag
                   new_message_test_straight_1_0_bag
                   new_message_test_straight_1_5_bag
                   new_message_test_straight_2_0_bag
                   new_message_test_straight_2_5_bag
                   new_message_test_straight_3_0_bag
                   new_message_test_oval_slow_bag
                   new_message_test_oval_medium_bag
                   new_message_test_oval_fast_bag
                   test_state_estimation_car_specs
  )

  add_rostest(launch/tests/nodelet_load/load_state_estimation.test)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_state_estimation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
