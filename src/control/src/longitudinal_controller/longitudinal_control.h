#ifndef LONGITUDINAL_CONTROL_H
#define LONGITUDINAL_CONTROL_H

#include "common/parameter_interface.h"
#include "concrete_longitudinal_controller.h"
#include "longitudinal_control_params.h"

#include <common/macros.h>
#include "common/path.h"

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/optional.hpp>
THIRD_PARTY_HEADERS_END


#include "common/parameter_interface.h"

class LongitudinalControl {
 public:
  /*!
   * \brief LongitudinalControl is the constructor. A ros independent
   * functionality containing
   * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
   * to get access to parameters.
   * \param parameters the ParameterInterface
   */
  LongitudinalControl(ParameterInterface* parameters);
  LongitudinalControl(LongitudinalControl&&) = default;
  virtual ~LongitudinalControl() = default;

  /*!
   * \brief stoppingCompleted return stopping controllers that have completed
   * stopping
   * \param stopped_controllers_ids the stopping controller ids which have been
   * completed.
   * \param measured_speed the current measured speed.
   */
  void stoppingCompleted(std::vector<unsigned long>& stopped_controllers_ids,
                         double measured_speed);

  /*!
   * \brief generateSpeed gets the speed control output from the active
   * controller and limits the acceleration on the control output
   * \param measured_speed the current mesured speed.
   * \param now the time stamp to use.
   * \return the new speed value.
   */
  double generateSpeed(const double measured_speed, const ros::Time& now);

  void reset();

  void updateParams();

  /*!
   * \brief setDesiredSpeed sets desired_speed_ to limit the control output
   * speed at runtime
   * \param desired_speed the new desired speed.
   */
  void setDesiredSpeed(const double& desired_speed);

  /*!
   * \brief setPath sets the path that constrains the speed
   * \param path the new path.
   */
  void setPath(const boost::optional<common::Path<>>& path);

  /**
   * @brief setVehiclePose
   * @param vehicle_pose_in_path_frame the new pose of the vehicle.
   */
  void setVehiclePoseInPathFrame(const boost::optional<Eigen::Affine2d>& vehicle_pose_in_path_frame);

  /*!
   * \brief setStoppingDistance sets the stopping_distance (but doesn't activate
   * the stopping controller)
   * \param distance the new distance to stop.
   * \param id the stopping controller to use.
   */
  void setStoppingDistance(double distance, unsigned long id);

  /*!
   * \brief updateACC update distance to obstacle and speed of obstacle
   * \param distance the new distance to the object.
   * \param speed the new speed.
   * \param stamp the new time stamp.
   */
  void updateACC(double distance, double speed, const ros::Time& stamp);

  /*!
   * \brief activateStoppingController sets active_controller to
   * stopping_controller (curvature controller remains active)
   */
  unsigned long activateStoppingController(double distance);

  /*!
   * \brief deactivateStoppingController deactivates stopping controller
   */
  void deactivateStoppingController(unsigned long id);

  /*!
   * \brief deactivateAllStoppingControllers deactivates all stopping
   * controllers (resets stopping_controllers)
   */
  void deactivateAllStoppingControllers();

  /*!
   * \brief activateACC sets active_controller to acc_controller (curvature
   * controller remains active)
   * \param message debug message
   * \return whether activation succeeded.
   */
  bool activateACC(std::string& message);

  /*!
   * \brief deactivateACC deactivates acc
   */
  void deactivateACC();

 protected:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* parameters_ptr_;

  int driving_direction = 1;

  std::shared_ptr<LongitudinalControlParams> params;

  double v_soll_last_;


  /*!
   * \brief curvature_controller the curvature controller limits the driving
   * speed by considering the curvature of the path in front of the vehicle.
   * This controller is always active
   */
  std::unique_ptr<CurvatureController> curvature_controller;
  /*!
   * \brief acc_controller the acc_controller keeps a constant distance to
   * another vehicle driving infront of the car
   */
  boost::optional<ACC> acc_controller;
  /*!
   * \brief stopping_controller the stopping controller chooses a driving speed
   * so that the vehicle stops at stopping_distance
   */
  StoppingControllers stopping_controllers;
};

#endif  // LONGITUDINAL_CONTROL_H
