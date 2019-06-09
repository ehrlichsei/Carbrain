#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common/best_score.h>
#include <ros/console.h>
#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <cmath>
#include <iterator>
#include <numeric>
//#include <boost/algorithm/transform_reduce.hpp>
THIRD_PARTY_HEADERS_END

#include "common/angle_conversions.h"
#include "common/basic_statistics.h"
#include "common/math.h"

#include "concrete_longitudinal_controller.h"

using common::sgn;
using common::squared;


double absMin(double lh, double rh) {
  if (std::fabs(lh) < std::fabs(rh)) {
    return lh;
  } else {
    return rh;
  }
}

CurvatureController::CurvatureController(std::shared_ptr<LongitudinalControlParams> params)
    : params(std::move(params)), desired_speed(0.0), abs_last_desired_speed(0.0) {}

double CurvatureController::calculateSpeed(double measured_speed,
                                           const ros::Duration& dur,
                                           int /*driving_direction*/) {
  if (!path_) {
    ROS_WARN_THROTTLE(4, "no path in curvature control, setting speed to 0.0");
    return 0.0;
  }
  if (std::isnan(desired_speed)) {
    ROS_ERROR_THROTTLE(4,
                       "desired speed is NaN in longitudinal controller, "
                       "setting speed to 0.0");
    return 0.0;
  }

  const common::Path<>& path = path_.get();

  //! Allow for max velocity to be adjusted directly by navigation
  const double v_dynamic_max =
      boost::algorithm::clamp(desired_speed,
                              -std::fabs(params->v_max_curvature_control_reverse),
                              std::fabs(params->v_max_curvature_control));
  const bool reverse_driving = desired_speed < 0.0;

  /*if (path.size() < 3) {
    ROS_ERROR_THROTTLE(
        4, "not enough path points for longitudinal control: only %zu points",
  path.size());
    return 0.0;
  }*/

  double v_desired_abs = std::numeric_limits<double>::infinity();


  if (!reverse_driving) {

    const double lookahead_distance =
        std::max(measured_speed * params->lookahead_cc_t, 0.);
    double last_lookahead =
        lookahead_distance - (last_measured_speed + measured_speed) / 2. * dur.toSec();
    if ((lookahead_distance - last_lookahead) < params->stepsize_forward_calculation) {
      last_lookahead = lookahead_distance - params->stepsize_forward_calculation;
    }

    calculate_backward(last_lookahead, path);
    calculate_forward(last_lookahead, lookahead_distance, path);

    if (!v_backward_calculation.empty() && !v_forward_calculation.empty()) {
      const double min_backward =
          *std::min_element(v_backward_calculation.begin(),
                            v_backward_calculation.end(),
                            [lookahead_distance](const auto& a, const auto& b) {
                              return std::fabs(a.arc_length - lookahead_distance) <
                                     std::fabs(b.arc_length - lookahead_distance);
                            });

      const double min_forward =
          *std::min_element(v_forward_calculation.begin(),
                            v_forward_calculation.end(),
                            [lookahead_distance](const auto& a, const auto& b) {
                              return std::fabs(a.arc_length - lookahead_distance) <
                                     std::fabs(b.arc_length - lookahead_distance);
                            });

      v_desired_abs = std::min(min_backward, min_forward);
    } else {
      ROS_ERROR("empty forward or backwards calculation");
      v_desired_abs = 0.0;
    }

    /*
     * @brief if start at great angle to path, we should limit the velocity
     */
    if (vehicle_pose_) {
      double psi = common::toYaw(vehicle_pose_->rotation());
      double psi_t = atan(path.firstDerivative(lookahead_distance));

      const double yaw_to_path = psi_t - psi;
      if (std::abs(yaw_to_path) > params->angle_krit) {
        v_desired_abs =
            v_desired_abs / (params->angle_krit_k * std::abs(yaw_to_path) + 1.0);
      }
    }

    if (std::isnan(v_desired_abs)) {
      ROS_ERROR_THROTTLE(
          4,
          "calculated NaN desired speed in curvature controller, setting speed "
          "to 0.0");
      abs_last_desired_speed = 0.0;
      return 0.0;
    }
  }

  // v_desired is determined by considering the desired driving direction and
  // the admissible speed interval (determined by v_max_cc, v_max_cc_reverse,
  // v_min_cc and desired_speed)
  double v_desired = 0.0;
  if (reverse_driving) {
    v_desired = boost::algorithm::clamp(
        -params->v_max_curvature_control_reverse, v_dynamic_max, 0.0);
  } else {
    v_desired = boost::algorithm::clamp(
        v_desired_abs, std::min(params->v_min_curvature_control, v_dynamic_max), v_dynamic_max);
  }

  abs_last_desired_speed = v_desired_abs;
  last_measured_speed = measured_speed;
  return v_desired;
}



void CurvatureController::calculate_backward(double last_lookahead,
                                             const common::Path<>& path) {

  // const double s_begin = path.front().arc_length;
  const double s_end = path.back().arc_length;

  // s_end is not an apex but we will always calculate back
  // from the end of the path. However the calculation is
  // different from the apexes because at the path end can be
  // longitudinal acceleration

  v_backward_calculation.clear();

  if (s_end > 0.) {
    // v_backward_calculation is the speedtrajectory calculated backwards from
    // the apex to the current position
    v_backward_calculation.reserve(static_cast<size_t>(std::fabs(s_end / params->stepsize)));

    v_backward_calculation.emplace_back(
        params->v_extrapolation / std::sqrt(std::fabs(path.curvature(s_end))), s_end);

    bool found_apexes = false;
    for (double s = s_end - params->stepsize; s >= last_lookahead; s -= params->stepsize) {
      const double abs_curvature = std::fabs(path.curvature(s));
      const double a_long = std::sqrt(std::max(
          std::pow(params->a_krit, 2) -
              std::pow(abs_curvature * std::pow(v_backward_calculation.back(), 2), 2),
          0.));
      if (a_long == 0.0) {
        found_apexes = true;
      }
      const double smaller_speed =
          std::min(std::sqrt(params->a_krit / abs_curvature),
                   a_long / v_backward_calculation.back() * params->stepsize +
                       v_backward_calculation.back());

      v_backward_calculation.emplace_back(smaller_speed, s);
    }
    if (!found_apexes) {
      ROS_WARN("v_extrapolation dominates calculation");
    }
  }
}



void CurvatureController::calculate_forward(double last_lookahead,
                                            double lookahead_distance,
                                            const common::Path<>& path) {
  v_forward_calculation.clear();
  v_forward_calculation.reserve(static_cast<size_t>(
      std::fabs(lookahead_distance / params->stepsize_forward_calculation)));
  v_forward_calculation.emplace_back(abs_last_desired_speed, last_lookahead);

  if (abs_last_desired_speed > params->v_min_abs) {
    double s = last_lookahead + params->stepsize_forward_calculation;
    do {
      const double abs_curvature = std::fabs(path.curvature(s));
      const double a_long = std::sqrt(std::max(
          std::pow(params->a_krit, 2) -
              std::pow(abs_curvature * std::pow(v_forward_calculation.back(), 2), 2),
          0.));
      const double smaller_speed =
          std::min(std::sqrt(params->a_krit / abs_curvature),
                   a_long / v_forward_calculation.back() * params->stepsize_forward_calculation +
                       v_forward_calculation.back());

      v_forward_calculation.emplace_back(smaller_speed, s);

      s += params->stepsize_forward_calculation;
    } while (s <= lookahead_distance);
  } else {
    v_forward_calculation.emplace_back(params->v_min_curvature_control,
                                       last_lookahead + params->stepsize_forward_calculation);
  }
}



void CurvatureController::setDesiredSpeed(double ds) { desired_speed = ds; }

void CurvatureController::setPath(const boost::optional<common::Path<>>& path) {
  path_ = path;
}

void CurvatureController::setVehiclePose(const boost::optional<Eigen::Affine2d>& vehicle_pose) {
  vehicle_pose_ = vehicle_pose;
}

void CurvatureController::reset() {
  desired_speed = std::numeric_limits<double>::infinity();
  abs_last_desired_speed = 0.0;
  path_ = boost::none;
  vehicle_pose_ = boost::none;
}


StoppingController::StoppingController(const std::shared_ptr<LongitudinalControlParams>& params,
                                       double stopping_distance)
    : params(params), stopping_distance(stopping_distance) {}

double StoppingController::calculateSpeed(double measured_speed, int driving_direction) {
  driving_direction_ = driving_direction;
  //überprüfe auf NAN
  if (!std::isfinite(stopping_distance)) {
    ROS_INFO(
        "StoppingController::doCalculateSpeed() was called but "
        "stopping_distance is %4.2f. Setting Speed to 0.0m/s",
        stopping_distance);
    return 0.0;
  }
  if (std::fabs(stopping_distance) < params->stopping_radius) {
    //! This is only true, if we are so close to the object
    //! where we want to stop, that we can already stop
    arrived_at_stopping_distance = true;
    return 0.0;
  }
  if (sgn(stopping_distance) != driving_direction) {
    // if we are driving away from the stopping point, brake immediately
    overshot = true;
  }
  if (overshot) {
    ROS_WARN_THROTTLE(4, "overshot stopping position");
    return 0.0;
  }

  //! Calculates the maximal velocity the car can have
  //! if we want to stop at the given distance decelerating
  //! with max_dec
  //! These formula can be easily derived from
  //! v = a * t
  //! s = 1/2 * a * t^2
  //! eliminating t and solving for v
  // double v_soll =
  // sqrt(2 * params->stopping_deceleration * std::max(0.0, stopping_distance));
  //! Possible Upgrade:
  //! It might be possible that the such calculated
  //! v_soll is already smaller than measured_speed
  //! by the time the stopping line/obstacle is first seen.
  //! This will happen only if the car is driving fast
  //! and is seeing the stopping line very late.
  //! In this case it is not possible to come to a
  //! complete stop at the given distance, using the
  //! deceleration max_dec.
  //! Idea: Implement emergency braking mode which
  //! uses higher deceleration to come to a complete stop
  //! before the stopping line. The needed deceleration
  //! can also be calculated with above formulas:
  //! a = 1/2 * v^2 / s
  //! Important: This deceleration has to stay the same
  //! throughout the whole braking process, so save it.
  //! If it is recalculated in every iteration, it will
  //! just grow bigger and bigger without stopping the car!

  //! Only if the calculated maximal velocity is smaller
  //! than the current velocity, set v_soll
  // return std::min(v_soll, std::abs(measured_speed));

  const double a_needed = 0.5 * squared(measured_speed) / std::fabs(stopping_distance);

  if (a_needed < params->stopping_deceleration) {
    // we dont need to decelerate yet
    received_stopping_command_in_time = true;
    return (driving_direction < 0.0 ? -std::numeric_limits<double>::infinity()
                                    : std::numeric_limits<double>::infinity());
  } else {
    // ROS_INFO("Decelleration of %4.2f needed!", a_needed);
    if (!received_stopping_command_in_time) {
      ROS_WARN_THROTTLE(4,
                        "received stopping command too late to brake in time\n"
                        "(a_needed > stopping_deceleration when stopping "
                        "command received)\na_needed = "
                        "%lf\nstopping_deceleration = %lf\nstopping_distance = "
                        "%lf\nspeed = %lf",
                        a_needed,
                        params->stopping_deceleration,
                        stopping_distance,
                        measured_speed);
    }
    const double lookahead_distance = 0.5 * a_needed * squared(params->lookahead_stop_t);

    const double stopping_distance_difference = std::fabs(stopping_distance) - lookahead_distance;

    if (stopping_distance_difference <= 0.0) {
      // if we are already trying to look farther than the stopping distance,
      // brake immediately
      return 0.0;
    } else {
      return sgn(stopping_distance) *
             std::sqrt(2 * params->stopping_deceleration * stopping_distance_difference);
    }
  }
}

void StoppingController::setStoppingDistance(double distance) {
  if (std::fabs(distance - stopping_distance) > 0.08) {
    ROS_WARN(
        "stopping distance set for stopping controller differs "
        "more than 8cm from previous stopping distance\n"
        "previous stopping distance: %lf\n"
        "new stopping distance: %lf",
        stopping_distance,
        distance);
  }
  stopping_distance = distance;
}

void StoppingController::decrementStoppingDistance(double delta_distance) {
  stopping_distance -= delta_distance;
}

void StoppingController::reset() {
  stopping_distance = 0.0;
  overshot = false;
  arrived_at_stopping_distance = false;
  stopping_completed_published = false;
}


bool StoppingController::hasStoppedInLastIteration(double measured_speed) {
  if (!stopping_completed_published &&
      (std::fabs(measured_speed) < params->stopping_completion_threshold_speed &&
       (arrived_at_stopping_distance || overshot))) {
    stopping_completed_published = true;
    ROS_INFO("stopping controller has stopped");
    if (overshot) {
      if (arrived_at_stopping_distance) {
        ROS_WARN("vehicle overshot the intended stopping position");
      } else {
        ROS_WARN(
            "vehicle moved away from stopping position and stopped therefore "
            "without arriving at the intended stopping position");
      }
    } else {
      ROS_INFO("vehicle has stopped at stopping position");
    }
    ROS_INFO("remaining stopping distance of %lf m", stopping_distance);
    return true;
  } else {
    return false;
  }
}


ACC::ACC(const std::shared_ptr<LongitudinalControlParams>& params)
    : params(params) {
  ACC::reset();
}

double ACC::calculateSpeed(double measured_speed, int /*driving_direction*/) {
  //! ACC implementation as in Moritz Werlings lecture, but only based on
  //! velocities, not acceleration
  //! constant_time_gap is the set time gap between the ego vehicle and the
  //! vehicle which is being
  //! followed depending on the velocity, this yields a gap distance d_set.
  //! distance_time_constant is a parameter determining how much the difference
  //! in the set distance d_set and actual distance modifies the velocity.

  double v_obstacle = 0.0;

  if (delta_t.isZero() || params->v_obstacle_param) {
    v_obstacle = params->dynamic_obstacle_velocity;
  } else {
    double v_rel = delta_distance / delta_t.toSec();

    switch (params->smoothening_style) {  // zum testen verschiedener
                                          // smoothening verfahren

      case MEAN_FILTER: {
        v_rel_buffer.push_front(v_rel);
        v_rel = common::mean(v_rel_buffer);
        break;
      }

      case RECURSIVE_FILTER: {
        v_rel = v_rel_old * params->v_rel_recursive_param +
                v_rel * (1.0 - params->v_rel_recursive_param);
        v_rel_old = v_rel;
        break;
      }

      case MEDIAN_FILTER: {
        v_rel_buffer.push_front(v_rel);
        std::vector<double> v(v_rel_buffer.size());
        std::copy(v_rel_buffer.begin(), v_rel_buffer.end(), v.begin());
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        v_rel = v[v.size() / 2];
        break;
      }

      default: {
        v_rel = 0.0;
        ROS_ERROR_THROTTLE(
            4,
            "invalid smoothening_style for obstacle speed smoothening in ACC");
        break;
      }
    }

    v_obstacle = measured_speed + boost::algorithm::clamp(v_rel, 0.0, 1.0);
    ROS_INFO_THROTTLE(
        2, "v_rel: %f v_obstacle: %f measured speed %f", v_rel, v_obstacle, measured_speed);
  }

  double d_set = params->constant_time_gap * v_obstacle + params->constant_distance_gap;

  return std::max(0.0, v_obstacle - 1 / params->distance_time_constant * (d_set - distance_to_obstacle));
}

void ACC::setDistanceToObstacle(double distance) {
  distance_to_obstacle = distance;
  delta_distance = distance - old_distance;
  old_distance = distance;
}

void ACC::setSpeedOfObstacle(double speed) { speed_of_obstacle = speed; }

void ACC::updateTimeStamp(const ros::Time& new_stamp) {
  delta_t = old_stamp.isZero() ? ros::Duration(0, 0) : new_stamp - old_stamp;
  old_stamp = new_stamp;
}

void ACC::reset() {
  distance_to_obstacle = std::numeric_limits<double>::infinity();
  speed_of_obstacle = 0.0;
  old_stamp = ros::Time(0);
  delta_t = ros::Duration(0, 0);
  delta_distance = 0.0;
  old_distance = 0.0;
  v_rel_buffer = boost::circular_buffer<double>(params->smoothening_range);
  v_rel_old = 0.0;
}


StoppingControllers::StoppingControllers(const std::shared_ptr<LongitudinalControlParams>& params)
    : params(params) {}

double StoppingControllers::calculateSpeed(double measured_speed, int driving_direction) {
  std::vector<double> speeds(stopping_controllers.size());
  std::transform(stopping_controllers.begin(),
                 stopping_controllers.end(),
                 speeds.begin(),
                 [measured_speed, driving_direction](auto& c) {
                   return c.second.calculateSpeed(measured_speed, driving_direction);
                 });
  return std::accumulate(
      speeds.begin(), speeds.end(), std::numeric_limits<double>::infinity(), &absMin);
  /* boost::transform_reduce(stopping_controllers.begin(),
                           stopping_controllers.end(),
                           &calculated_speed,
                           [measured_speed](const StoppingController& s) {
                             return s.calculateSpeed(measured_speed);
                           },
                           [](double a, double b) { return std::min(a, b); });*/
  /*return std::transform_reduce(stopping_controllers.begin(),
                        stopping_controllers.end(),std::numeric_limits<double>::infinity(),
                        [measured_speed](const StoppingController& s) {
                          return s.calculateSpeed(measured_speed);
                        },
                        [](double a, double b) { return std::min(a, b); });*/
  // return calculated_speed;
}

void StoppingControllers::setStoppingDistance(double distance, unsigned long id) {
  try {
    stopping_controllers.at(id).setStoppingDistance(distance);
  } catch (const std::out_of_range&) {
    throw std::runtime_error(
        "tried to set stopping distance for non-existent stopping controller "
        "with id " +
        std::to_string(id));
  }
}

void StoppingControllers::decrementStoppingDistance(double delta_distance) {
  std::for_each(stopping_controllers.begin(),
                stopping_controllers.end(),
                [delta_distance](auto& s) {
                  s.second.decrementStoppingDistance(delta_distance);
                });
}

void StoppingControllers::reset() { stopping_controllers.clear(); }

unsigned long StoppingControllers::addStoppingController(double distance) {
  stopping_controllers.emplace(++current_id, StoppingController(params, distance));
  return current_id;
}

void StoppingControllers::removeStoppingController(unsigned long id) {
  if (stopping_controllers.erase(id) == 0) {
    throw std::runtime_error(
        "tried to deactivate non-existant stopping controller with id " + std::to_string(id));
  }
}

void StoppingControllers::getStoppedControllersIds(std::vector<unsigned long>& stopped_controllers_ids,
                                                   double measured_speed) {
  stopped_controllers_ids.clear();
  for (auto& c : stopping_controllers) {
    if (c.second.hasStoppedInLastIteration(measured_speed)) {
      stopped_controllers_ids.push_back(c.first);
    }
  }
}
