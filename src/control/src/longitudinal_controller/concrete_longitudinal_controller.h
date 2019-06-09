#ifndef CONCRETE_LONGITUDINAL_CONTROLLER_H
#define CONCRETE_LONGITUDINAL_CONTROLLER_H
#include <common/macros.h>
#include <boost/circular_buffer.hpp>
#include "longitudinal_control_params.h"


THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <boost/optional.hpp>
#include <memory>
#include <vector>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "common/path.h"


double absMin(double lh, double rh);

/*!
 * \brief The CurvatureController class implements a controller that limits the
 * driving speed using the curvature of the path in front of the car.
 * This controller is always active as it is the only concrete controller that
 * directly inherity from LongitudinalController.
 */
class CurvatureController {
 public:
  CurvatureController(std::shared_ptr<LongitudinalControlParams> params);

  virtual ~CurvatureController() = default;

  CurvatureController(const CurvatureController &) = default;

  virtual double calculateSpeed(double measured_speed, const ros::Duration &dur, int driving_direction);

  void calculate_backward(double last_lookahead, const common::Path<> &path);

  void calculate_forward(double last_lookahead,
                         double lookahead_distance,
                         const common::Path<> &path);

  void setDesiredSpeed(double ds);

  void setPath(const boost::optional<common::Path<>> &path);

  void setVehiclePose(const boost::optional<Eigen::Affine2d> &vehicle_pose);

  void reset();

  /*!
   * \brief The ArcLengthParameterizedSpeed struct represents the (allowed)
   * speed at a specified arc length (e.g. of the path) relative to the vehicle.
   * Because of the implicit conversion operators to double a
   * ArcLengthParameterizedSpeed can be used directly as if it were the speed it
   * represents.
   */
  struct ArcLengthParameterizedSpeed {
    ArcLengthParameterizedSpeed(double speed, double arc_length)
        : speed(speed), arc_length(arc_length) {}
    operator double() const { return speed; }
    operator double &() { return speed; }

    double speed = 0.0;
    double arc_length = 0.0;
  };

 protected:
  const std::shared_ptr<LongitudinalControlParams> params;

  double desired_speed;
  double abs_last_desired_speed;
  double last_measured_speed = 0.0;
  boost::optional<common::Path<>> path_;
  std::vector<ArcLengthParameterizedSpeed> v_backward_calculation;
  std::vector<ArcLengthParameterizedSpeed> v_forward_calculation;

  /**
   * @brief vehicle_pose to calculate the angle error between vehicle and
   * trajectory
   */
  boost::optional<Eigen::Affine2d> vehicle_pose_;
};

/*!
 * \brief The StoppingController class implements the stop_at functionality used
 * to stop at e.g. stop lines.
 * It calculates the driving speed that is required for the car to stop in
 * stopping_distance meters.
 * The stopping distance is at least once set with setStoppingDistance, after
 * that it will be decremented with decrementStoppingDistance by the distance
 * the car has driven since the last iteration.
 */
class StoppingController {
 public:
  StoppingController(const std::shared_ptr<LongitudinalControlParams> &params,
                     double stopping_distance);

  double calculateSpeed(double measured_speed, int driving_direction);

  /*!
   * \brief setStoppingDistance use this function to update the
   * stopping_distance
   * \param distance the distance.
   */
  void setStoppingDistance(double distance);

  /*!
   * \brief decrementStoppingDistance this function has to be called exactly
   * once every iteration to decrement the stopping distance by the distance
   * driven in the meantime.
   * \param delta_distance the delta distance.
   */
  void decrementStoppingDistance(double delta_distance);

  void reset();

  bool hasStoppedInLastIteration(double measured_speed);

 private:
  const std::shared_ptr<LongitudinalControlParams> params;

  double stopping_distance;
  int driving_direction_ = 1;
  bool overshot = false;
  bool arrived_at_stopping_distance = false;
  bool stopping_completed_published = false;
  bool received_stopping_command_in_time = false;
};

/*!
 * \brief The StoppingController class implements the stop_at functionality used
 * to stop at e.g. stop lines.
 * It calculates the driving speed that is required for the car to stop in
 * stopping_distance meters.
 * The stopping distance is at least once set with setStoppingDistance, after
 * that it will be decremented with decrementStoppingDistance by the distance
 * the car has driven since the last iteration.
 */
class StoppingControllers {
 public:
  StoppingControllers(const std::shared_ptr<LongitudinalControlParams> &params);

  double calculateSpeed(double measured_speed, int driving_direction);

  /*!
   * \brief setStoppingDistance use this function to update the
   * stopping_distance
   * \param distance the distance.
   */
  void setStoppingDistance(double distance, unsigned long id);

  /*!
   * \brief decrementStoppingDistance this function has to be called exactly
   * once every iteration to decrement the stopping distance by the distance
   * driven in the meantime.
   * \param delta_distance the delta distance.
   */
  void decrementStoppingDistance(double delta_distance);

  void reset();

  unsigned long addStoppingController(double distance);

  void removeStoppingController(unsigned long id);

  void getStoppedControllersIds(std::vector<unsigned long> &stopped_controllers_ids,
                                double measured_speed);


 private:
  const std::shared_ptr<LongitudinalControlParams> params;

  std::map<unsigned long, StoppingController> stopping_controllers;
  unsigned long current_id = 0;
};


/*!
 * \brief The ACC class implements the ACC controller which allows the car to
 * hold a constant distance to another vehicle driving in front of the car.
 * It requires the distance to and the speed of the other vehicle as inputs.
 */
class ACC {
 public:
  ACC(const std::shared_ptr<LongitudinalControlParams> &params);

  double calculateSpeed(double measured_speed, int driving_direction);

  void setDistanceToObstacle(double distance);

  void setSpeedOfObstacle(double speed);

  void updateTimeStamp(const ros::Time &new_stamp);

  void reset();

 private:
  std::shared_ptr<LongitudinalControlParams> params;

  ros::Time old_stamp = ros::Time(0);
  ros::Duration delta_t = ros::Duration(0, 0);
  double delta_distance = 0.0;
  double old_distance = 0.0;
  double distance_to_obstacle = std::numeric_limits<double>::infinity();
  double speed_of_obstacle = 0.0;
  double v_rel_old = 0.0;
  boost::circular_buffer<double> v_rel_buffer;

  enum SmootheningStyle {
    MEAN_FILTER = 0,
    RECURSIVE_FILTER = 1,
    MEDIAN_FILTER = 2
  };
};

#endif  // CONCRETE_LONGITUDINAL_CONTROLLER_H
