#ifndef QR_CODE_FSM_H
#define QR_CODE_FSM_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost_sml_catkin/sml.hpp>
#include <controller_msgs/LightsCommand.h>
THIRD_PARTY_HEADERS_END
#include "timeout.h"
#include "state_machine_helper.h"
#include "events.h"
namespace SM {

  struct QRCodeStateVisualization : public VisualizationState {
    virtual std::string getBase() override { return "ignore"; }
    virtual std::string getName() override { return "ignore"; }
  };
struct Start : public QRCodeStateVisualization {};
struct End : public QRCodeStateVisualization {};
struct QRDetected : public QRCodeStateVisualization {};
struct WaitInBox : public QRCodeStateVisualization {};
struct DriveStraight : public QRCodeStateVisualization {};
struct DataQRCode {
  Timeout timeout_;
  double drove_distance = 0;
};
template <class CarControllerData, class NormalState>
struct QRCodeFSM : public QRCodeStateVisualization {
  auto operator()() const {
    const auto activateQrCodeDetection =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().activateQrCodeDetection(true);
        };
    const auto startCarAndDeactivateQrCodeDetection =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().setMaxSpeedAfterQRCode();
          shared_data->getCarController().activateQrCodeDetection(false);
        };

    const auto deactivateBlinkerInObstacleMode =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().setPavlovBlinkerCommand(
              controller_msgs::BlinkerCommand::NONE);
        };
    const auto activateBlinkerInObstacleMode =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().resetPavlovBlinkerCommand();
        };

    const auto activateHighBeam = [](const std::shared_ptr<CarControllerData> shared_data) {
      shared_data->getCarController().setHighBeam(true);
    };
    const auto deactivateHighBeam = [](const std::shared_ptr<CarControllerData> shared_data) {
      shared_data->getCarController().setHighBeam(false);
    };

    const auto deactivateStraightPath =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().straightPathOutOfStartBox(false);
        };

    const auto activateStraightPath =
        [](const std::shared_ptr<CarControllerData> shared_data) {
          shared_data->getCarController().straightPathOutOfStartBox(true);
        };

    const auto resetAll = [](const std::shared_ptr<CarControllerData> shared_data) {
      shared_data->getCarController().resetLaneDetection();
      shared_data->getCarController().resetPavlovBlinkerCommand();
      shared_data->getCarController().resetPathPreprocessing();
      shared_data->getCarController().resetEnvironmentalModel();
    };

    const auto stopDrivingStraight =
        [](const std::shared_ptr<DataQRCode> data_qr_code, const EventDroveDistance& ev) {
          data_qr_code->drove_distance += ev.distance;
          return data_qr_code->drove_distance > 0.9;
        };


    using namespace sml;
    IfTimeoutPassed<DataQRCode> ifTimeoutPassed;
    const auto setWaitingTime = [](const std::shared_ptr<DataQRCode> data_qr_code) {
      data_qr_code->timeout_.setNewDuration(ros::Time::now(), ros::Duration(1.5));
    };
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        // bypass qr_code_detection
        state<Start>         + event<EventStart>           / startCarAndDeactivateQrCodeDetection     = state<NormalState>,

       *state<Start>         + sml::on_entry<_>            / activateQrCodeDetection,
        state<Start>         + event<EventQRDetection>                                                = state<QRDetected>,

        state<QRDetected>    + sml::on_entry<_>            / (deactivateBlinkerInObstacleMode, activateHighBeam),
        state<QRDetected>    + sml::on_exit<_>             / deactivateHighBeam,
        state<QRDetected>    + event<EventNoQRCodeVisible> / setWaitingTime                           = state<WaitInBox>,

        state<WaitInBox>     + event<EventTimeUpdate>[ifTimeoutPassed]                                = state<DriveStraight>,
        state<WaitInBox>     + event<EventQRDetection>                                                = state<QRDetected>,

        state<DriveStraight> + event<EventDroveDistance>[stopDrivingStraight]                         = state<NormalState>,
        state<DriveStraight> + sml::on_entry<_>             / (resetAll, startCarAndDeactivateQrCodeDetection, activateStraightPath),
        state<DriveStraight> + sml::on_exit<_>              / (activateBlinkerInObstacleMode, deactivateStraightPath),

        state<NormalState>   + event<EventReset>                                                      = state<End>);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }
};
}

#endif  // QR_CODE_FSM_H
