cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs controller_msgs nav_msgs perception_msgs)

# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

find_package(kitcar_cmake)
add_files_to_project()

add_message_files(
  DIRECTORY msg
  FILES
    Obstacle.msg
    Obstacles.msg
    ProfileControlPoint.msg
    Profile.msg
    StateOfDriving.msg
    AutomaticCruiseControlUpdate.msg
    DrivingCorridor.msg
    Gate.msg
    NoPassingZone.msg
    NoPassingZones.msg
    TrackingElement.msg
    TrackingElements.msg
    Junction.msg
    Junctions.msg
    RoadClosure.msg
    RoadClosures.msg
    Crosswalk.msg
    Crosswalks.msg
    CeresSummary.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetHlControllerParams.srv
   SetVMax.srv
   SetPathPlannerParams.srv
   DriveIntoParkingSlot.srv
   PavlovBlinkerCommand.srv
   FindParkingSlot.srv
   DrivePastNextRoadClosure.srv
   SetRespectNoPassingZones.srv
   ParkPerpendicular.srv
   TurnAt.srv
   ReverseOutOfParkingSpot.srv
 )

generate_messages(DEPENDENCIES std_msgs geometry_msgs controller_msgs nav_msgs perception_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs controller_msgs nav_msgs perception_msgs
  DEPENDS Boost)

