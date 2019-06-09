cmake_minimum_required(VERSION 2.8.3)
project(mission_control_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

find_package(kitcar_cmake)
add_files_to_project()

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   StateMachineState.msg
   StateMachineStates.msg
 )

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost
)
