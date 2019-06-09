#include "command_handler.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include "controller_msgs/BlinkerCommand.h"
THIRD_PARTY_HEADERS_END

#include "protobuf_msg_version.h"

CommandHandler::CommandHandler(const bool use_back_steering)
    : use_steering_control_back(use_back_steering),
      steering_control_front_(CHANNEL_ID_STEERING_SERVO_OUTPUT),
      steering_control_back_(CHANNEL_ID_STEERING_BACK_SERVO_OUTPUT),
      engine_power_(CHANNEL_ID_ENGINE_POWER),
      engine_brake_(CHANNEL_ID_ENGINE_BRAKE),
      brake_lights_(CHANNEL_ID_BRAKE_LIGHTS),
      blinker_mutex(),
      blinker_mutex_attr(),
      mission_mode_mutex(),
      mission_mode_mutex_attr(),
      activate_brake_lights_by_service_call(false) {
  engine_power_.write(0.0);
  engine_brake_.write(0.0);
  brake_lights_.write(false);
  steering_control_front_.write(0.5);
  steering_control_back_.write(0.5);

  pthread_mutexattr_init(&blinker_mutex_attr);
  pthread_mutexattr_setprotocol(&blinker_mutex_attr, PTHREAD_PRIO_INHERIT);
  pthread_mutex_init(&blinker_mutex, &blinker_mutex_attr);

  pthread_mutexattr_init(&mission_mode_mutex_attr);
  pthread_mutexattr_setprotocol(&mission_mode_mutex_attr, PTHREAD_PRIO_INHERIT);
  pthread_mutex_init(&mission_mode_mutex, &mission_mode_mutex_attr);
}

CommandHandler::~CommandHandler() {
  pthread_mutexattr_destroy(&blinker_mutex_attr);
  pthread_mutex_destroy(&blinker_mutex);

  pthread_mutexattr_destroy(&mission_mode_mutex_attr);
  pthread_mutex_destroy(&mission_mode_mutex);
}

void CommandHandler::createRequestCommand(const bool with_proto_msg_version,
                                          const unsigned int request_id,
                                          kitcar::Command *protobuf_command) {
  protobuf_command->set_request_measures(true);
  protobuf_command->set_request_id(request_id);

  if (with_proto_msg_version) {
    protobuf_command->set_msg_version(PROTOBUF_MSG_VERSION);
  }
}

void CommandHandler::createMissionMode(kitcar::Command *protobuf_command) {
  pthread_mutex_lock(&mission_mode_mutex);
  if (mission_mode_has_been_updated) {
    mission_mode_has_been_updated = false;
    protobuf_command->set_mode(missionModeToMode(mission_mode));
  }
  pthread_mutex_unlock(&mission_mode_mutex);
}

void CommandHandler::createSteeringCommand(kitcar::Command *protobuf_command) {
  protobuf_command->mutable_steering_control_front()->set_steering_value(
      *steering_control_front_.get());
  if (use_steering_control_back) {
    protobuf_command->mutable_steering_control_back()->set_steering_value(
        *steering_control_back_.get());
  }
}

void CommandHandler::createSpeedCommand(kitcar::Command *protobuf_command) {
  kitcar::Command::SpeedControl *speed_control =
      protobuf_command->mutable_speed_control();
  const float speed_intensity = *engine_power_.get();
  const float brake_intensity = *engine_brake_.get();
  speed_control->set_speed_value(speed_intensity);
  speed_control->set_brake_value(brake_intensity);
  kitcar::Command::LightsControl *lights_control =
      protobuf_command->mutable_lights_control();
  if (*brake_lights_.get() || activate_brake_lights_by_service_call) {
    lights_control->set_brake_lights(kitcar::Command::OnOff::Command_OnOff_ON);
  } else {
    lights_control->set_brake_lights(kitcar::Command::OnOff::Command_OnOff_OFF);
  }
}

void CommandHandler::createLightsCommand(kitcar::Command *protobuf_command) {
  pthread_mutex_lock(&blinker_mutex);
  if (lights_command_has_been_updated) {
    lights_command_has_been_updated = false;
    kitcar::Command::LightsControl *lights_control =
        protobuf_command->mutable_lights_control();
    kitcar::Command::BlinkerControl blinker_control =
        blinkerCommandToBlinkerControl(lights_command.blinker);
    lights_control->set_blinker_control(blinker_control);
    lights_control->set_high_beam(boolMsgToOnOff(lights_command.high_beam));
    lights_control->set_debug_light_color(
        debugLightColorCommandToControl(debug_light_color_command));
  }
  pthread_mutex_unlock(&blinker_mutex);
}

kitcar::Command::BlinkerControl CommandHandler::blinkerCommandToBlinkerControl(
    const controller_msgs::BlinkerCommand &command) {
  switch (command.command) {
    case controller_msgs::BlinkerCommand::NONE:
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_NONE;
    case controller_msgs::BlinkerCommand::LEFT:
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_LEFT;
    case controller_msgs::BlinkerCommand::RIGHT:
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_RIGHT;
    case controller_msgs::BlinkerCommand::BOTH:
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_BOTH;
    case controller_msgs::BlinkerCommand::BOTH_THREE_TIMES:
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_BOTH_THREE_TIMES;
    default:
      ROS_ERROR_THROTTLE(1,
                         "Unhandled blinker command. Enabling BOTH blinkers!");
      return kitcar::Command::BlinkerControl::Command_BlinkerControl_BOTH;
  }
}

kitcar::Command::Mode CommandHandler::missionModeToMode(const common_msgs::MissionMode &mission_mode) {
  switch (mission_mode.mission_mode) {
    case common_msgs::MissionMode::IDLE:
      return kitcar::Command::Mode::Command_Mode_IDLE;
    case common_msgs::MissionMode::FREE_RIDE:
      return kitcar::Command::Mode::Command_Mode_ROUND_TRIP;
    case common_msgs::MissionMode::OBSTACLE:
      return kitcar::Command::Mode::Command_Mode_ROUND_TRIP_WITH_OBSTACLES;
    case common_msgs::MissionMode::PARKING:
      return kitcar::Command::Mode::Command_Mode_PARKING;
    default:
      ROS_ERROR_THROTTLE(1, "Unknown Mission command: sending IDLE");
      return kitcar::Command::Mode::Command_Mode_IDLE;
  }
}

void CommandHandler::setLightsCommand(controller_msgs::LightsCommand lights_command) {
  pthread_mutex_lock(&blinker_mutex);
  lights_command_has_been_updated =
      lights_command.blinker.command != this->lights_command.blinker.command ||
      lights_command.high_beam.data != this->lights_command.high_beam.data ||
      lights_command_has_been_updated;
  this->lights_command = lights_command;
  pthread_mutex_unlock(&blinker_mutex);
}

void CommandHandler::setBrakeLightsCommand(bool activate_brake_lights) {
  activate_brake_lights_by_service_call = activate_brake_lights;
}

void CommandHandler::setMissionMode(const common_msgs::MissionMode &mission_mode) {
  pthread_mutex_lock(&mission_mode_mutex);
  if ((mission_mode.header.stamp > this->mission_mode.header.stamp ||
       mission_mode.mission_mode != this->mission_mode.mission_mode)) {
    mission_mode_has_been_updated = true;
    this->mission_mode = mission_mode;
  }
  pthread_mutex_unlock(&mission_mode_mutex);
}

bool operator==(const std_msgs::ColorRGBA &lhs, const std_msgs::ColorRGBA &rhs) {
  return (lhs.r == rhs.r) && (lhs.g == rhs.g) && (lhs.b == rhs.b);
}

void CommandHandler::setDebugLightColorCommand(const std_msgs::ColorRGBA &color_command) {
  lights_command_has_been_updated = true;
  debug_light_color_command = color_command;
}

kitcar::Command::OnOff CommandHandler::boolMsgToOnOff(std_msgs::Bool msg) {
  if (msg.data > 0) {
    return kitcar::Command::OnOff::Command_OnOff_ON;
  } else {
    return kitcar::Command::OnOff::Command_OnOff_OFF;
  }
}

uint32_t CommandHandler::debugLightColorCommandToControl(std_msgs::ColorRGBA command) {
  float red_f = boost::algorithm::clamp<float>(command.r, 0.0, 1.0);
  float green_f = boost::algorithm::clamp<float>(command.g, 0.0, 1.0);
  float blue_f = boost::algorithm::clamp<float>(command.b, 0.0, 1.0);
  uint8_t red = static_cast<uint8_t>(red_f * 255);
  uint8_t green = static_cast<uint8_t>(green_f * 255);
  uint8_t blue = static_cast<uint8_t>(blue_f * 255);
  uint32_t control = (red << 0) | (green << 8) | (blue << 16);
  return control;
}
