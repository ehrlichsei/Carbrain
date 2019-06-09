#ifndef SPEED_LIMIT_H
#define SPEED_LIMIT_H

#include <common/macros.h>
THIRD_PARTY_HEADERS_BEGIN
#include <limits>
THIRD_PARTY_HEADERS_END
#include "hl_fsm.h"

namespace SM {
  struct SpeedLimitStateVisualization : public DrivingSubStateVisualization {
  public:
    std::string getName() override { return "speed limit"; }
  };
  struct SpeedLimitSubStateVisualization : public VisualizationState {
  public:
    std::string getBase() override { return "speed limit"; }
  };
struct SpeedLimited : public SpeedLimitSubStateVisualization {
  std::string getName() override { return "speed limited"; }
};
struct no_speed_limit : public SpeedLimitSubStateVisualization {
  std::string getName() override { return "no speed limit"; }
};
struct DataSpeedLimit {
  unsigned long speed_limitation_id = 0;
  double drove_distance = 0;
  unsigned int speed_limit_in_km_per_h = 0;
  void setDroveDistance(const double value) { drove_distance = value; }
};

struct SpeedLimitSM : public SpeedLimitStateVisualization {
  SpeedLimitSM() = default;
  SpeedLimitSM(const std::shared_ptr<SharedData>& shared_data,
               const std::shared_ptr<DataSpeedLimit>& data_speed_limit)
      : shared_data_(shared_data), data_speed_limit_(data_speed_limit) {}
  auto operator()() const {
    const auto setSpeedLimitInKmPerH =
        [](const std::shared_ptr<DataSpeedLimit> data_speed_limit,
           const EventSpeedLimitStartPassed& ev,
           const std::shared_ptr<SharedData> shared_data) {
          data_speed_limit->speed_limit_in_km_per_h = ev.speed_limit_in_km_per_h;
          data_speed_limit->speed_limitation_id =
              shared_data->getCarController().setMaxSpeedInModelSizeKmPerH(
                  data_speed_limit->speed_limit_in_km_per_h,
                  data_speed_limit->speed_limitation_id);
        };
    const auto clearMaxSpeed = [](const std::shared_ptr<DataSpeedLimit> data_speed_limit,
                                  const std::shared_ptr<SharedData> shared_data) {
      shared_data->getCarController().clearMaxSpeed(data_speed_limit->speed_limitation_id);
      data_speed_limit->speed_limitation_id = 0;
    };

    const auto updateAndCheckDroveDistance =
        [](const std::shared_ptr<DataSpeedLimit> data_speed_limit,
           const EventDroveDistance& ev,
           const std::shared_ptr<SharedData> shared_data) {
          data_speed_limit->drove_distance += ev.distance;
          return data_speed_limit->drove_distance >
                 shared_data->getSpeedLimitResetDistance(data_speed_limit->speed_limit_in_km_per_h);
        };
    const auto setDroveDistanceZero =
        std::bind(&DataSpeedLimit::setDroveDistance, data_speed_limit_, 0.0);
    using namespace sml;
    // clang-format off
    DISABLE_USED_BUT_MARKED_UNUSED_WARNING
    return make_transition_table(
        * state<no_speed_limit>  + event<EventSpeedLimitStartPassed> / defer                                          = state<SpeedLimited>,
          state<SpeedLimited> + event<EventSpeedLimitStartPassed> / (setDroveDistanceZero, setSpeedLimitInKmPerH),
          state<SpeedLimited> + event<EventDroveDistance>[updateAndCheckDroveDistance]                             = state<no_speed_limit>,
          state<SpeedLimited> + event<EventSpeedLimitEndPassed>                                                    = state<no_speed_limit>,
          state<SpeedLimited> + sml::on_exit<_> / clearMaxSpeed);
    // clang-format on
    ENABLE_USED_BUT_MARKED_UNUSED_WARNING
  }

 private:
  std::shared_ptr<SharedData> shared_data_;
  std::shared_ptr<DataSpeedLimit> data_speed_limit_;
};
}

#endif
