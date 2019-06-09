cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs actionlib_msgs actionlib geometry_msgs sensor_msgs nav_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

find_package(kitcar_cmake)
add_files_to_project()

add_message_files(
  DIRECTORY msg
  FILES 
  RegionOfInterestStamped.msg
  Lane.msg
  Obstacle.msg Obstacles.msg
  Pixel.msg Pixels.msg
  ScanLine.msg ScanLines.msg
  RoadClosure.msg RoadClosures.msg
  SpeedLimitMarking.msg SpeedLimitMarkings.msg
  Crosswalk.msg Crosswalks.msg
  Junction.msg Junctions.msg
  Unidentified.msg Unidentifieds.msg
  StartLine.msg StartLines.msg
  ArrowMarking.msg ArrowMarkings.msg
  Pedestrian.msg Pedestrians.msg
  NoPassingZone.msg NoPassingZones.msg
  PerpendicularParkingSpot.msg PerpendicularParkingSpots.msg
  ParkingLotFound.msg
  PointsCluster.msg PointsClusters.msg
  QrCode.msg QrCodes.msg
  LookAtRegion.msg LookAtRegions.msg
  LookAtRegionDebug.msg LookAtRegionsDebug.msg
  RoadObjects.msg
  TrafficSign.msg
  TrafficSigns.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  LookAt.srv
  SetMiddleLine.srv
  SearchParkingSpots.srv
)

add_action_files(
  DIRECTORY action
  FILES
  LookForObstacles.action
  LookForPedestrians.action
  )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs nav_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs geometry_msgs sensor_msgs
  DEPENDS Boost
)
