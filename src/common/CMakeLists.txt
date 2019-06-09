cmake_minimum_required(VERSION 3.5)
project(common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  common_msgs
#  state_estimation_msgs
#  perception_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  dynamic_reconfigure
  pluginlib
  nodelet
  diagnostic_updater
)

find_package(Eigen3 REQUIRED)

#checking launch files when compiling
find_package(roslaunch REQUIRED)
roslaunch_add_file_check(launch)

find_package(kitcar_cmake REQUIRED)
add_files_to_project()

generate_dynamic_reconfigure_options(
  cfg/ParameterHandlerTest.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## include Eigen library

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

set(SUPPORTED_CARS "mr_fast2015;dr_drift")

if(NOT "$ENV{CAR_NAME}" IN_LIST SUPPORTED_CARS )
  message(FATAL_ERROR "Unsupported car '$ENV{CAR_NAME}' detected!")
endif()


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
  INCLUDE_DIRS include include/$ENV{CAR_NAME}
  LIBRARIES self_tests pca_eigen camera_transformation node_base realtime_controller runtime_analyzer realtime_timings bresenham
  CATKIN_DEPENDS roscpp diagnostic_updater common_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be lisclang static analyzer ted before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include/$ENV{CAR_NAME}
)

INCLUDE(src/node_base/CMakeLists.txt)

## Declare a cpp library
add_library(realtime_timings
  src/realtime_timings.cpp
)

add_library(self_tests
  src/self_tests.cpp
)
target_link_libraries(self_tests
  ${catkin_LIBRARIES}
)

add_library(pca_eigen
  src/pca_eigen.cpp
)
target_link_libraries(pca_eigen
  ${catkin_LIBRARIES}
)

add_library(realtime_controller
  src/realtimecontroller.cpp
)
target_link_libraries(realtime_controller
  ${catkin_LIBRARIES}
  boost_chrono
)

add_library(camera_transformation
    src/camera_transformation.cpp
)
target_link_libraries(camera_transformation
  ${catkin_LIBRARIES}
)

add_library(runtime_analyzer
  src/runtime_analyzer.cpp
)
target_link_libraries(runtime_analyzer
  ${catkin_LIBRARIES}
  boost_chrono
)

add_library(bresenham
    src/bresenham.cpp
)
target_link_libraries(bresenham
  ${catkin_LIBRARIES}
)


###########
## Nodes ##
###########

add_executable(static_transform_publisher
  src/static_transform_publisher.cpp
)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set_target_properties(static_transform_publisher PROPERTIES COMPILE_FLAGS "-Wno-everything")
endif()

if(CMAKE_CXX_CLANG_TIDY)
 set_target_properties(static_transform_publisher PROPERTIES CXX_CLANG_TIDY "")
endif()

target_link_libraries(static_transform_publisher
  ${catkin_LIBRARIES}
)

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
# install(TARGETS common common_node
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

  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(test_polynomial test/test_polynomial.cpp)
  catkin_add_gtest(test_normal_shift test/test_normal_shift.cpp)
  catkin_add_gtest(test_math test/test_math.cpp)
  catkin_add_gtest(test_eigen_utils test/test_eigen_utils.cpp)
  catkin_add_gtest(test_polynomial_utils test/test_polynomial_utils.cpp)
  catkin_add_gtest(test_discretisize_polynomial test/test_discretisize_polynomial.cpp)
  catkin_add_gtest(test_polynomialfit test/test_polynomialfit.cpp)
  catkin_add_gtest(test_angle_conversions test/test_angle_conversions.cpp)
  catkin_add_gtest(test_adaptors test/test_adaptors.cpp)
  catkin_add_gtest(test_eigen_adaptors test/test_eigen_adaptors.cpp)
  catkin_add_gtest(test_path_conversion test/test_path_conversion.cpp)
  catkin_add_gtest(test_pca_eigen test/test_pca_eigen.cpp)
  catkin_add_gtest(test_type_to_string test/test_type_to_string.cpp)
  catkin_add_gtest(test_containers test/test_containers.cpp)
  catkin_add_gtest(test_indirect_sort test/test_indirect_sort.cpp)
  catkin_add_gtest(test_simultaneous_sort test/test_simultaneous_sort.cpp)
  catkin_add_gtest(test_best_score test/test_best_score.cpp)
  catkin_add_gtest(test_minmax_element test/test_minmax_element.cpp)
  catkin_add_gtest(test_contains test/test_contains.cpp)
  catkin_add_gtest(test_basic_statistics test/test_basic_statistics.cpp)
  catkin_add_gtest(test_basic_statistics_eigen test/test_basic_statistics_eigen.cpp)
  catkin_add_gtest(test_dummy_parameter_handler test/test_dummy_parameter_handler.cpp)
  catkin_add_gtest(test_concurrent_queue test/test_concurrent_queue.cpp)
  catkin_add_gtest(test_camera_transformation test/test_camera_transformation.cpp)
  catkin_add_gtest(test_path test/test_path.cpp)

  target_link_libraries(test_concurrent_queue
    ${catkin_LIBRARIES}
  )

  # remove ros dependencies to avoid linker errors
  set_target_properties(test_path PROPERTIES COMPILE_DEFINITIONS "ROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_FATAL")

  target_link_libraries(test_camera_transformation
    camera_transformation
  )

  target_link_libraries(test_pca_eigen
    pca_eigen
  )

  find_package(rostest REQUIRED)
  add_rostest_gtest(test_parameter_handler launch/parameter_handler.test test/test_parameter_handler.cpp)
  target_link_libraries(test_parameter_handler ${catkin_LIBRARIES} node_base)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(test_nodelet_loader
    test/test_nodelet_loader.cpp
  )

  target_link_libraries(test_nodelet_loader
    ${catkin_LIBRARIES}
    ${common_LIBRARIES}
    ${GTEST_LIBRARIES}
  )

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif(CATKIN_ENABLE_TESTING)
