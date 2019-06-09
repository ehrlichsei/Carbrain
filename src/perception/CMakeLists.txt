cmake_minimum_required(VERSION 2.8.8)
project(perception)
## Find catkin macros, libraries and system dependencies

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  state_estimation_msgs
  perception_msgs
  navigation_msgs
  pixelwise_mask_generation_msgs
  state_estimation_msgs
  cmake_modules
  dynamic_reconfigure
  tf2_ros
  tf2_eigen
  common
  nodelet
  message_filters
  ceres_catkin
  actionlib_msgs
  actionlib
  eigen_conversions
)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video aruco)
include_directories(SYSTEM {OpenCV_INCLUDE_DIRS})

#checking launch files when compiling
find_package(roslaunch REQUIRED)
roslaunch_add_file_check(launch)

find_package(kitcar_cmake REQUIRED)
add_files_to_project()

# add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/PreprocessingRoi.cfg
  cfg/PreprocessingMedianBlur.cfg
  cfg/PreprocessingBinarisation.cfg
  cfg/PreprocessingTrapezoidMask.cfg
  cfg/PreprocessingPixelwiseMask.cfg
  cfg/PreprocessingMaskMode.cfg
  cfg/RoadWatcher.cfg
  cfg/ParkinglotMappingNode.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES birds_view_transformation distinct_colors
)

###########
## Build ##
###########

## include header files
include_directories(include)

## include catkin
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${common_INCLUDE_DIRS})

## include Eigen library
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

INCLUDE(src/utils/CMakeLists.txt)

INCLUDE(src/road_object_detection/CMakeLists.txt)

###########
## Nodes ##
###########

INCLUDE(src/qr_code_detection/CMakeLists.txt)

INCLUDE(src/perpendicular_parking/CMakeLists.txt)

INCLUDE(src/road_watcher/CMakeLists.txt)

INCLUDE(src/extrinsic_calibration/CMakeLists.txt)

INCLUDE(src/lane_detection/CMakeLists.txt)

INCLUDE(src/preprocessing/CMakeLists.txt)

INCLUDE(src/pixelwise_mask_generation/CMakeLists.txt)

INCLUDE(src/parkinglot_mapping/CMakeLists.txt)

INCLUDE(src/look_at/CMakeLists.txt)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  INCLUDE(test/CMakeLists.txt)
endif(CATKIN_ENABLE_TESTING)

###################
## Documentation ##
###################

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
find_program(ROSDOC_LITE rosdoc_lite)
if(DOXYGEN_FOUND AND ROSDOC_LITE)
  add_custom_target(doc-perception #ALL
    rosdoc_lite ${CMAKE_CURRENT_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
    COMMENT "Generating API documentation with rosdoc_lite" VERBATIM
    )
endif(DOXYGEN_FOUND AND ROSDOC_LITE)
