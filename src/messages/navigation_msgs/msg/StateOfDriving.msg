# this message contains the state of driving (currently representing the state of intersection_handler)
std_msgs/Char state_of_driving

uint8 DRIVING=0
uint8 APPROACHING_STOPLINE=1
uint8 WAITING_AT_STOPLINE=2
