# message type for representing a QR code in an image

# The type represents the code style e.g. QR-Code, barcode, ... according to zbar::Symbol::get_type_name
std_msgs/String type
# The type represents the message that is encoded, e.g. 'STOP'
std_msgs/String data
# Corners of the code in the image
Pixels location
# Estimated pose based on camera parameters
geometry_msgs/PoseStamped pose_stamped
