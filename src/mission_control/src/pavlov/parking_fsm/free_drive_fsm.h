#ifndef FREE_DRIVE_FSM_H
#define FREE_DRIVE_FSM_H

#include "../fsm_utils/events.h"
#include "common/parameter_interface.h"
#include "parameters_free_drive_state_machine.h"
#include "../fsm_utils/timeout.h"
#include "../fsm_utils/diagnostics_interface.h"
#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <boost_sml_catkin/sml.hpp>
#include <functional>
#include <tf2_eigen/tf2_eigen.h>
THIRD_PARTY_HEADERS_END
#include "data_parking.h"
#include "../fsm_utils/state_machine_helper.h"
#include "../fsm_utils/qr_code_fsm.h"



namespace sml = boost::sml;
namespace SM {
struct WaitingForStartFreeDrive;
struct OutOfParkingZone;
struct SearchParkingSpot;
struct InParkingZone;
struct ParkIntoParkingSpot;
struct ReverseOutOfParkingSpot;
struct WaitInParkingSpot;

const auto getDistanceToStop = [](const EventPathMsgStopping& ev,
                                  const std::shared_ptr<DataParking> data_parking) {
  const double length_in_parking_lot =
      data_parking->getParams().distance_to_rear_bumper -
      data_parking->getParams().wheelbase +
      data_parking->getParams().rear_safety_margin_parking;
  Eigen::Affine3d parking_pose;
  tf2::fromMsg(data_parking->parking_spot_.entrance_pose_left, parking_pose);
  double distance = 0;
  Eigen::Vector3d last_v, current_v;
  tf2::fromMsg(ev.path_msg->poses[0].pose.position, last_v);
  for (const geometry_msgs::PoseStamped& pose : ev.path_msg->poses) {

    tf2::fromMsg(pose.pose.position, current_v);
    if ((parking_pose.inverse() * ev.path_pose * current_v).x() > length_in_parking_lot) {
      last_v = current_v;
      continue;
    }
    if ((ev.vehicle_pose.inverse() * ev.path_pose * current_v).x() > 0) {
      distance += (last_v - current_v).norm();
    }
    last_v = current_v;
  }
  return distance;
};

struct ParkingSM {
  ParkingSM(const std::shared_ptr<DataParking> data_parking)
      : data_parking_(data_parking) {}
  auto operator()() const {
    const auto ifTimeoutPassedParking =
        [](const std::shared_ptr<DataParking>& data_parking, const EventTimeUpdate& ev) {
          data_parking->timeout_.update(ev.current_time);
          return data_parking->timeout_.hasPassed();
        };

    const auto stopInParkingSpot = [](const EventPathMsgStopping& ev,
                                      const std::shared_ptr<DataParking> data_parking) {
      data_parking->stopping_id_ = data_parking->getCarController().stopAtDistance(
          (*getDistanceToStop)(ev, data_parking), data_parking->stopping_id_);
    };

    const auto setParkingSpot = [](const EventFoundParkingSpot& ev,
                                   const std::shared_ptr<DataParking> data_parking) {
      data_parking->parking_spot_ = ev.parking_spot;
      data_parking->set_path_ = true;
    };
    const auto updateParkingSpot = [](const EventFoundParkingSpot& ev,
                                      const std::shared_ptr<DataParking> data_parking) {
      Eigen::Affine3d old_pose, new_pose;
      tf2::fromMsg(ev.parking_spot.entrance_pose_left, new_pose);
      tf2::fromMsg(data_parking->parking_spot_.entrance_pose_left, old_pose);
      Eigen::Vector3d left_spot_position = new_pose.translation();
      const double distance = (left_spot_position - ev.car_position).norm();
      if (distance < data_parking->getParams().parking_spot_update_distance) {
        data_parking->getCarController().searchParkingSpot(false);
        return;
      }

      // avoid unnecessary updates
      const Eigen::Affine3d difference = old_pose.inverse() * new_pose;
      if (((difference * Eigen::Vector3d(0.2, 0, 0)) - Eigen::Vector3d(0.2, 0, 0))
              .norm() < 0.01) {
        return;
      }
      if (ev.parking_spot.id.data != data_parking->parking_spot_.id.data) {
        return;
      }
      data_parking->parking_spot_ = ev.parking_spot;
      data_parking->getCarController().parkPerpendicular(ev.parking_spot);
    };

    const auto setSpeedLimitSearch = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->max_speed_limitation_id_ = data_parking->getCarController().setMaxSpeedInMPerS(
          data_parking->getParams().velocity_search_parking_spots,
          data_parking->max_speed_limitation_id_);
    };

    const auto clearMaxSpeedParking = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().clearMaxSpeed(data_parking->max_speed_limitation_id_);
      data_parking->max_speed_limitation_id_ = 0;
    };
    const auto startDrivingParking = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().startDriving(data_parking->stopping_id_);
      data_parking->stopping_id_ = 0;
    };
    const auto enterParkIntoParkingSpot = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().parkPerpendicular(data_parking->parking_spot_);
      data_parking->getCarController().blinkLeftBeforeParkingSpot(true);
    };
    const auto exitParkIntoParkingSpot = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().searchParkingSpot(false);
      data_parking->getCarController().blinkLeftBeforeParkingSpot(false);
    };

    const auto resetAfterParking = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().reverseOutOfParkingSpot(nav_msgs::Path(), false);
      data_parking->getCarController().resetLaneDetection();
      data_parking->getCarController().resetPathPreprocessing();
    };
    const auto increaseParkingCounter = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->increaseNumberOfParkAttemptsCounter();
    };


    const auto changedParkingSpot = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->set_path_ = true;
      data_parking->time_changed_pose_ = ros::Time::now();
    };
    const auto ifShouldUpdatePath = [](const std::shared_ptr<DataParking> data_parking,
                                       const EventPathMsgUpdateReversePath& ev) {
      return data_parking->set_path_ &&
             data_parking->time_changed_pose_ < ev.path_msg->header.stamp;
    };

    const auto startDrivingReverse = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().reverseOutOfParkingSpot(
          data_parking->reverse_path_, true);
      data_parking->getCarController().publishReversePathToWorldTransform(
          data_parking->pose_reverse_path_, ros::Time::now());
      data_parking->max_speed_limitation_id_ = data_parking->getCarController().setMaxSpeedInMPerS(
          -std::numeric_limits<double>::infinity(), data_parking->max_speed_limitation_id_);
      data_parking->getCarController().startDriving(0);
      data_parking->stopping_id_ = data_parking->getCarController().stopAtDistance(
          -data_parking->getParams().reverse_out_of_parking_spot_distance, 0);
    };
    const auto updateReversePathToWorldTransform =
        [](const std::shared_ptr<DataParking> data_parking, const EventTimeUpdate& ev) {
          data_parking->getCarController().publishReversePathToWorldTransform(
              data_parking->pose_reverse_path_, ev.current_time);
        };

    const auto setPathReverseOutOfParkingSpot = [](
        const std::shared_ptr<DataParking> data_parking,
        const EventPathMsgUpdateReversePath& ev) {
      data_parking->reverse_path_ = *ev.path_msg;
      data_parking->reverse_path_.header.frame_id = "out_of_parking_spot_path";
      for (geometry_msgs::PoseStamped& pose : data_parking->reverse_path_.poses) {
        pose.header.frame_id = "out_of_parking_spot_path";
      }
      data_parking->pose_reverse_path_ = ev.path_pose;
      data_parking->set_path_ = false;
    };
    const auto updateAndCheckDroveDistance =
        [](const std::shared_ptr<DataParking> data_parking, const EventDroveDistance& ev) {
          data_parking->distance_behind_start_line_ += ev.distance;
          return data_parking->distance_behind_start_line_ > 20.0;
        };
    const auto blinkInParkingSpot = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().blinkInParkingSpot(true);
    };
    const auto stopBlinkInParkingSpot = [](const std::shared_ptr<DataParking> data_parking) {
      data_parking->getCarController().blinkInParkingSpot(false);
    };
    const auto acceptStartLine = [](const std::shared_ptr<DataParking> data_parking) {
      return !data_parking->reachedNumberOfParkAttempts();
    };

    const double waiting_time = data_parking_->getParams().waiting_time_in_parking_spot;
    using namespace sml;
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    // clang-format off
    return make_transition_table(
        *"parking_active"_s             + sml::on_exit<_>                                         / resetAfterParking ,

        *state<OutOfParkingZone>        + event<EventStartLineDetected>[acceptStartLine]
                                                / (std::bind(&DataParking::initValuesNewParkingAttempt, data_parking_, std::placeholders::_1),
                                                   std::bind(&ICarController::searchParkingSpot,&data_parking_->getCarController(),true))
                  = state<SearchParkingSpot>,


        state<SearchParkingSpot>        + sml::on_entry<_>                                        / setSpeedLimitSearch,
        state<SearchParkingSpot>        + sml::on_exit<_>                                         / clearMaxSpeedParking,
        state<SearchParkingSpot>        + event<EventFoundParkingSpot>                            / (setParkingSpot, changedParkingSpot)
                  = state<ParkIntoParkingSpot>,
        state<SearchParkingSpot>        + event<EventDroveDistance>[updateAndCheckDroveDistance]
                  = state<OutOfParkingZone>,


        state<ParkIntoParkingSpot>      + sml::on_entry<_>                                        / enterParkIntoParkingSpot,
        state<ParkIntoParkingSpot>      + sml::on_exit<_>                                         / exitParkIntoParkingSpot,
        state<ParkIntoParkingSpot>      + event<EventPathMsgUpdateReversePath>[ifShouldUpdatePath]/ setPathReverseOutOfParkingSpot,
        state<ParkIntoParkingSpot>      + event<EventPathMsgStopping>                             / stopInParkingSpot,
        state<ParkIntoParkingSpot>      + event<EventFoundParkingSpot>                            / (updateParkingSpot, changedParkingSpot),
        state<ParkIntoParkingSpot>      + event<EventCarHasStopped>                               / std::bind(setTimeout<DataParking>, data_parking_, waiting_time)
                  = state<WaitInParkingSpot>,


        state<WaitInParkingSpot>        + sml::on_entry<_>                                        / blinkInParkingSpot,
        state<WaitInParkingSpot>        + sml::on_exit<_>                                         / stopBlinkInParkingSpot,
        state<WaitInParkingSpot>        + event<EventTimeUpdate>[ifTimeoutPassedParking]          / startDrivingReverse
                  = state<ReverseOutOfParkingSpot>,


        state<ReverseOutOfParkingSpot>  + event<EventTimeUpdate>            / updateReversePathToWorldTransform,
        state<ReverseOutOfParkingSpot>  + event<EventCarHasStopped>         / (clearMaxSpeedParking, resetAfterParking, increaseParkingCounter, startDrivingParking)
                  = state<OutOfParkingZone>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<DataParking> data_parking_;
};


struct WaitingForStartFreeDrive;
struct FreeDriveStateMachine {
  auto operator()() const {
    using namespace sml;
    return make_transition_table(*state<QRCodeFSM<DataParking, ParkingSM>> +
                                     "not_exisitng_event"_e =
                                     state<QRCodeFSM<DataParking, ParkingSM>>);
  }
};
}

#endif
