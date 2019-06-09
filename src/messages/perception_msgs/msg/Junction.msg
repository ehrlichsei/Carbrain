int32 TYPE_STOPLINE_LEFT = 0
int32 TYPE_STOPLINE_RIGHT = 1
int32 TYPE_GIVEWAY_LEFT = 2
int32 TYPE_GIVEWAY_RIGHT = 3

Header header
int32 id
geometry_msgs/Pose pose
int32 junction_type
float32 certainty
geometry_msgs/Point[] base_hull_polygon
