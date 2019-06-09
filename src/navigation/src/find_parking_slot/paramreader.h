#ifndef FIND_PARKING_LOT_PARAM_READER
#define FIND_PARKING_LOT_PARAM_READER
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#define PARAM_PATH_OFFSET "path_offset"
#define PATH_OFFSET_DEFAULT -0.5f

#define PARAM_V_MAX "v_max"
#define V_MAX_DEFAULT 0.48f

#define PARAM_V_DRIVE_ON "v_drive_on"
#define V_DRIVE_ON_DEFAULT 0.24f

#define PARAM_MIN_SLOT_SIZE "min_slot_size"
#define MIN_SLOT_SIZE_DEFAULT 0.5f

#define PARAM_MAX_SLOT_SIZE "max_slot_size"
#define MAX_SLOT_SIZE_DEFAULT 0.7f

#define PARAM_START_IGNORE_DISTANCE "start_ignore_distance"
#define START_IGNORE_DISTANCE_DEFAULT 0.5f

#define PARAM_MIN_OBSTACLE_LEN "min_obstacle_len"
#define MIN_OBSTACLE_LEN_DEFAULT 0.05f

#define PARAM_MAX_RAMP_Y_DIFF "max_ramp_y_diff"
#define MAX_RAMP_Y_DIFF_DEFAULT 0.02f

#define PARAM_DRIVE_ON_DISTANCE "drive_on_distance"
#define DRIVE_ON_DISTANCE_DEFAULT 0.02f

/**
 * @brief reads parking slot finding related parameters from the rosparam server
 */
class ParamReader {
 public:
  ParamReader() : node("~") {}

  /**
   * @brief returns the path offset parameter value that is stored in
   * "PARAM_PATH_OFFSET"
   */
  double getPathOffset() const {
    double path_offset;
    if (node.getParam(PARAM_PATH_OFFSET, path_offset))
      return path_offset;
    else
      return PATH_OFFSET_DEFAULT;
  }

  /**
   * @brief returns the max speed parameter value that is stored in
   * "PARAM_V_MAX"
   */
  double getVMax() const {
    double speed;
    if (node.getParam(PARAM_V_MAX, speed)) return speed;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_V_MAX, V_MAX_DEFAULT);
    return V_MAX_DEFAULT;
  }

  double getVDriveOn() const {
    double speed;
    if (node.getParam(PARAM_V_DRIVE_ON, speed)) return speed;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_V_DRIVE_ON, V_DRIVE_ON_DEFAULT);
    return V_DRIVE_ON_DEFAULT;
  }

  /**
   * @brief returns the min slot size parameter value that is stored in
   * "PARAM_MIN_SLOT_SIZE"
   */
  double getMinSlotSize() {
    double minSize;
    if (node.getParam(PARAM_MIN_SLOT_SIZE, minSize)) return minSize;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_MIN_SLOT_SIZE, MIN_SLOT_SIZE_DEFAULT);
    return MIN_SLOT_SIZE_DEFAULT;
  }

  double getMaxSlotSize() {
    double maxSize;
    if (node.getParam(PARAM_MAX_SLOT_SIZE, maxSize)) return maxSize;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_MAX_SLOT_SIZE, MAX_SLOT_SIZE_DEFAULT);
    return MAX_SLOT_SIZE_DEFAULT;
  }

  /**
   * @brief returns the ignore distance (from start) parameter value that is
   * stored in "PARAM_START_IGNORE_DISTANCE"
   */
  double getStartIgnoreDistance() {
    double start_ignore_distance;
    if (node.getParam(PARAM_START_IGNORE_DISTANCE, start_ignore_distance))
      return start_ignore_distance;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_START_IGNORE_DISTANCE, START_IGNORE_DISTANCE_DEFAULT);
    return START_IGNORE_DISTANCE_DEFAULT;
  }

  double getMinObstacleLen() {
    double minObstacleLen;
    if (node.getParam(PARAM_MIN_OBSTACLE_LEN, minObstacleLen))
      return minObstacleLen;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_MIN_OBSTACLE_LEN, MIN_OBSTACLE_LEN_DEFAULT);
    return MIN_OBSTACLE_LEN_DEFAULT;
  }

  double getMaxRampYDiff() {
    double maxRampYDiff;
    if (node.getParam(PARAM_MAX_RAMP_Y_DIFF, maxRampYDiff))
      return maxRampYDiff;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_MAX_RAMP_Y_DIFF, MAX_RAMP_Y_DIFF_DEFAULT);
    return MAX_RAMP_Y_DIFF_DEFAULT;
  }

  double getDriveOnDistance() {
    double driveOnDistance;
    if (node.getParam(PARAM_DRIVE_ON_DISTANCE, driveOnDistance))
      return driveOnDistance;
    ROS_WARN("No %s parameter was defined. Using possibly insane default (%f).",
             PARAM_DRIVE_ON_DISTANCE, DRIVE_ON_DISTANCE_DEFAULT);
    return DRIVE_ON_DISTANCE_DEFAULT;
  }

 private:
  ros::NodeHandle node;
};

#endif  // FIND_PARKING_LOT_PARAM_READER
