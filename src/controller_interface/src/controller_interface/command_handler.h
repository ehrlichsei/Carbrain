#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include "command.pb.h"
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/algorithm/clamp.hpp>
#include "controller_msgs/LightsCommand.h"
#include "common_msgs/MissionMode.h"
#include <boost/optional.hpp>
THIRD_PARTY_HEADERS_END

#include <common/realtimeipc.h>
#include <common/realtime_channel_ids.h>

class CommandHandler {
 public:
  CommandHandler(const bool use_back_steering = false);
  ~CommandHandler();

  void createMissionMode(kitcar::Command *protobuf_command);
  void createRequestCommand(const bool with_proto_msg_version,
                            const unsigned int request_id,
                            kitcar::Command* protobuf_command);
  void createSteeringCommand(kitcar::Command* protobuf_command);
  void createSpeedCommand(kitcar::Command* protobuf_command);
  void createLightsCommand(kitcar::Command* protobuf_command);

  void setLightsCommand(controller_msgs::LightsCommand lights_command);
  void setBrakeLightsCommand(bool activate_brake_lights);
  void setMissionMode(const common_msgs::MissionMode& mission_mode);
	void setDebugLightColorCommand(const std_msgs::ColorRGBA& color_command);
 
  const bool use_steering_control_back;
  common::RealtimeIPC<float> steering_control_front_;
  common::RealtimeIPC<float> steering_control_back_;
  common::RealtimeIPC<float> engine_power_;
  common::RealtimeIPC<float> engine_brake_;
  common::RealtimeIPC<bool> brake_lights_;

 private:
  bool lights_command_has_been_updated = false;
  controller_msgs::LightsCommand lights_command;
  mutable pthread_mutex_t blinker_mutex;
  pthread_mutexattr_t blinker_mutex_attr;

  bool mission_mode_has_been_updated = false;
  common_msgs::MissionMode mission_mode;
  mutable pthread_mutex_t mission_mode_mutex;
  pthread_mutexattr_t mission_mode_mutex_attr;
  std_msgs::ColorRGBA debug_light_color_command;

  bool activate_brake_lights_by_service_call;

  kitcar::Command::OnOff boolMsgToOnOff(std_msgs::Bool msg);  
  kitcar::Command::BlinkerControl blinkerCommandToBlinkerControl(const controller_msgs::BlinkerCommand& command);
  kitcar::Command::Mode missionModeToMode(const common_msgs::MissionMode& mission_mode);
	uint32_t debugLightColorCommandToControl(std_msgs::ColorRGBA command);	

};


#endif  // COMMAND_HANDLER_H
