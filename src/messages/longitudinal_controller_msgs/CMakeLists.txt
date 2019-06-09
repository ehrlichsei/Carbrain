cmake_minimum_required(VERSION 2.8.3)
project(longitudinal_controller_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs kitcar_cmake)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

add_files_to_project()

add_message_files(
  DIRECTORY msg
  FILES 
  DrivingSpeed.msg)

add_service_files(
  FILES
  StopAt.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost)
