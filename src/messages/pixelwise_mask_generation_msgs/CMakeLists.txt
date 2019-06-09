cmake_minimum_required(VERSION 2.8.3)
project(pixelwise_mask_generation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

find_package(kitcar_cmake)
add_files_to_project()

#add_message_files(
#  DIRECTORY msg
#)

add_service_files(
  DIRECTORY srv
  FILES
  TakePictures.srv
  MedianBlur.srv
  SaveMask.srv
  Dilation.srv
  Closing.srv
  AutomaticMaskGen.srv
  BilateralFilter.srv
  SetStartingPoint.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost
)
