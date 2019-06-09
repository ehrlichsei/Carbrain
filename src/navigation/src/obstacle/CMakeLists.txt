add_library(navigation_obstacle_objs OBJECT
  src/obstacle/obstacle.cpp
)

add_dependencies(navigation_obstacle_objs
  ${catkin_EXPORTED_TARGETS}
)
