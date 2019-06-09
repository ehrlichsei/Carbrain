cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

find_package(kitcar_cmake)
add_files_to_project()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActivationService.srv
  TimeSyncCheck.srv
)

add_message_files(
  FILES
  MissionMode.msg
  Float32Stamped.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost
)
