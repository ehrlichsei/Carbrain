cmake_minimum_required(VERSION 2.8.3)
project(controller_interface)
include(FindProtobuf)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## build protobuf sources
add_subdirectory(protobuf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  common
  control
  std_msgs
  longitudinal_controller_msgs
  lateral_controller_msgs
  state_estimation_msgs
  controller_msgs
  rostest
)
find_package(Protobuf REQUIRED)
find_package(common REQUIRED)
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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   ackermann_msgs
#   state_estimation_msgs
#)

#########################################
## Declare dynamic reconfigure options ##
#########################################

#generate_dynamic_reconfigure_options(
#  cfg/ControllerInterface.cfg
#)

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
#   LIBRARIES ackermann_msgs
#   CATKIN_DEPENDS roscpp std_msgs
#   DEPENDS ackermann_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${PROTO_GEN_DIR}                    # for generated protobuf messages
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
  ${control_INCLUDE_DIRS}
  ${state_estimation_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(controller_interface
#   src/${PROJECT_NAME}/controller_interface.cpp
# )

#add_custom_target(dummy_proto ALL)
#add_dependencies(dummy_proto protobuild)
## Declare a cpp executable

set_source_files_properties(${PROTO_GEN} PROPERTIES GENERATED TRUE )

###########
## Nodes ##
###########

INCLUDE(src/controller_interface/CMakeLists.txt)

###################
## Documentation ##
###################

# Add a target to generate API documentation with Doxygen
find_package(Doxygen)
find_program(ROSDOC_LITE rosdoc_lite)
if(DOXYGEN_FOUND AND ROSDOC_LITE)
  add_custom_target(doc-controller-interface #ALL
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
# install(TARGETS controller_interface controller_interface_node
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
  catkin_add_gtest(command_handler-test
  	$<TARGET_OBJECTS:controller_interface_proto_objs>
    test/test_command_handler.cpp
    src/controller_interface/command_handler.cpp
  	#${PROTO_GEN}
  )

  add_rostest(launch/test/load_controller_interface.test)

  add_dependencies(command_handler-test
    protobuild
    #${PROJECT_NAME}_gencfg
  )

	target_link_libraries(command_handler-test
  	${PROTOBUF_LIBRARY}
  	${catkin_LIBRARIES}
	)
endif()
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_controller_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

