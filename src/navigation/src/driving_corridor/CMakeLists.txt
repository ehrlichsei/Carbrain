add_library(navigation_driving_corridor_objs OBJECT
  src/driving_corridor/driving_corridor.cpp
  src/driving_corridor/gate.cpp
)

add_dependencies(navigation_driving_corridor_objs
  ${catkin_EXPORTED_TARGETS}
)
