# Message type for a model representation of a obstacle

#uint8 TYPE_FRONT=0

int8 VERTEX_DETECTED = 1
int8 VERTEX_ESTIMATED = 2
int8 VERTEX_INSECURE = 3


Header header
int32 id

#same as base_hull_polygon (really needed?)
geometry_msgs/Point[] vertices

#vector of same length as base_hull_polygon containing information,
#wether a vertex has actually been seen by the classifier
int8[] vertices_detected

float32 certainty

#estimation of 4 obstacle vertices in world coordinates
geometry_msgs/Point[] base_hull_polygon
