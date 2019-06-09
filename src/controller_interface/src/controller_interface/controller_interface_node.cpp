#include "controller_interface_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <std_msgs/Empty.h>
THIRD_PARTY_HEADERS_END

#include "common/node_creation_makros.h"

#include "protobuf_msg_version.h"

template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& name) {
  T v;
  if (!nh.getParam(name, v)) {
    throw ros::InvalidParameterException("Parameter " + name + " is not set!");
  }
  return v;
}

ControllerInterfaceNode::ControllerInterfaceNode(ros::NodeHandle& node_handle)
    : NodeBase(node_handle),
      use_back_steering_(getParam<bool>(node_handle, "use_back_steering")),
      command_handler_(use_back_steering_),
      controller_interface_(&parameter_handler_, &measures_handler_, &command_handler_) {
  // always open communication to Arduino and newer close ist
  controller_interface_.startControlThread();

  // always print logging messages to see if communikation no works
  controller_interface_logging_thread_ = boost::thread(boost::bind(
      &ControllerInterfaceNode::controllerInterfaceLoggingLoop, this));
  // always publish mission mode
  mission_mode_publisher_ =
      node_handle_.advertise<common_msgs::MissionMode>("mission_mode", 10);

  controller_interface_mission_mode_publisher_thread_ = boost::thread(boost::bind(
      &ControllerInterfaceNode::controllerInterfaceMissionModePublisherLoop, this));

  sendIdleMode();
}

// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
ControllerInterfaceNode::~ControllerInterfaceNode() {
  sendIdleMode();
  // give the control thread enough time to send the idle mode
  ros::Duration(0.01).sleep();
  controller_interface_publisher_thread_.interrupt();
  controller_interface_snaphost_publisher_thread_.interrupt();
  controller_interface_mission_mode_publisher_thread_.interrupt();
  controller_interface_logging_thread_.interrupt();
  controller_interface_distance_sensors_thread_.interrupt();

  controller_interface_publisher_thread_.join();
  controller_interface_snaphost_publisher_thread_.join();
  controller_interface_distance_sensors_thread_.join();
  controller_interface_logging_thread_.join();
  controller_interface_mission_mode_publisher_thread_.join();
}

void ControllerInterfaceNode::startModule() {
  state_measure_publisher_ =
      node_handle_.advertise<controller_msgs::StateMeasure>("state_measure", 10);
  manual_mode_publisher_ = node_handle_.advertise<std_msgs::Bool>("manual_mode", 10);
  snapshot_request_publisher_ =
      node_handle_.advertise<std_msgs::Empty>("snapshot_trigger", 10);
  auto_reset_publisher_ = node_handle_.advertise<std_msgs::Empty>("auto_reset", 1);
  front_ir_sensor_publisher_ =
      node_handle_.advertise<sensor_msgs::Range>("infrared_sensor_front", 1);
  back_ir_sensor_publisher_ =
      node_handle_.advertise<sensor_msgs::Range>("infrared_sensor_back", 1);
  front_us_sensor_publisher_ =
      node_handle_.advertise<sensor_msgs::Range>("ultrasonic_sensor_front", 1);
  back_us_sensor_publisher_ =
      node_handle_.advertise<sensor_msgs::Range>("ultrasonic_sensor_back", 1);
  controller_interface_publisher_thread_ = boost::thread(boost::bind(
      &ControllerInterfaceNode::controllerInterfacePublisherLoop, this));
  controller_interface_snaphost_publisher_thread_ = boost::thread(boost::bind(
      &ControllerInterfaceNode::controllerInterfaceSnapshotLoop, this));
  controller_interface_distance_sensors_thread_ = boost::thread(boost::bind(
      &ControllerInterfaceNode::controllerInterfaceDistanceSensorsLoop, this));

  engine_power_subscriber_ = node_handle_.subscribe(
      "engine_power", 1, &ControllerInterfaceNode::onEnginePowerReceived, this);
  steering_control_subscriber_ = node_handle_.subscribe(
      "steering_control", 1, &ControllerInterfaceNode::onSteeringControlReceived, this);
  if (use_back_steering_) {
    steering_control_back_subscriber_ = node_handle_.subscribe(
        "steering_control_back", 1, &ControllerInterfaceNode::onBackSteeringControlReceived, this);
  }
  blinker_command_subscriber_ = node_handle_.subscribe(
      "blinker_command", 1, &ControllerInterfaceNode::onLightsCommandReceived, this);
  engine_brake_subscriber_ = node_handle_.subscribe(
      "engine_brake", 1, &ControllerInterfaceNode::onEngineBrakeReceived, this);
  mission_mode_ = node_handle_.subscribe(
      "mission_mode", 1, &ControllerInterfaceNode::onMissionMode, this);
  brake_lights_service_server_ = node_handle_.advertiseService(
      "brake_lights", &ControllerInterfaceNode::onBrakeLightCommandReceived, this);
  debug_light_command_subscriber_ = node_handle_.subscribe(
      "debug_light", 1, &ControllerInterfaceNode::onDebugLightCommandReceived, this);
}

void ControllerInterfaceNode::stopModule() {
  controller_interface_publisher_thread_.interrupt();
  controller_interface_snaphost_publisher_thread_.interrupt();
  controller_interface_distance_sensors_thread_.interrupt();

  controller_interface_publisher_thread_.join();
  controller_interface_snaphost_publisher_thread_.join();
  controller_interface_distance_sensors_thread_.join();

  state_measure_publisher_.shutdown();
  front_ir_sensor_publisher_.shutdown();
  back_ir_sensor_publisher_.shutdown();
  front_us_sensor_publisher_.shutdown();
  back_us_sensor_publisher_.shutdown();
  manual_mode_publisher_.shutdown();
  snapshot_request_publisher_.shutdown();
  auto_reset_publisher_.shutdown();

  blinker_command_subscriber_.shutdown();
  debug_light_command_subscriber_.shutdown();
  brake_lights_service_server_.shutdown();
  engine_power_subscriber_.shutdown();
  steering_control_subscriber_.shutdown();
  steering_control_back_subscriber_.shutdown();
  engine_brake_subscriber_.shutdown();
  mission_mode_.shutdown();
}

const std::string ControllerInterfaceNode::getName() {
  return std::string("controller_interface");
}

void ControllerInterfaceNode::controllerInterfaceLoggingLoop() {
  while (!boost::this_thread::interruption_requested()) {
    LoggingMessage message;
    if (!controller_interface_.getLoggingMessage(&message)) {
      continue;
    }
    switch (message) {
      case COBS_DECODING_ERROR:
        ROS_ERROR_THROTTLE(1, "cobs decoding failed");
        break;
      case FAILED_TO_OPEN_SERIAL_PORT:
        ROS_ERROR_THROTTLE(2, "failed to open serial port");
        break;
      case VALID_PROTOBUF_MESSAGE_PARSED:
        // ROS_DEBUG_THROTTLE(2, "valid packet received");
        break;
      case SERIAL_PORT_OPENED:
        ROS_INFO("Serial port opened");
        break;
      case TIMEOUT:
        ROS_WARN_THROTTLE(2, "Timer for waiting on serial port expired.");
        break;
      case PROTOBUF_PARSING_FAILED:
        ROS_ERROR_THROTTLE(2, "protobuf parsing failed");
        break;
      case ASIO_WRITE_EXCEPTION:
        ROS_ERROR_THROTTLE(
            2, "Writing on serial port failed! Maybe Arduino is not connected");
        break;
      case ASIO_READ_EXCEPTION:
        ROS_ERROR_THROTTLE(
            2, "Reading on serial port failed! Maybe Arduino is not connected");
        break;
      case MSG_VERSION_MATCH:
        ROS_INFO_STREAM_THROTTLE(
            2, "Protobuf versions match (0x" << std::hex << PROTOBUF_MSG_VERSION << ")");
        break;
      case WRONG_MSG_VERSION:
        ROS_FATAL_STREAM_THROTTLE(
            2,
            "Wrong version in protobuf message, we have 0x"
                << std::hex << PROTOBUF_MSG_VERSION << ", but controller has 0x"
                << std::hex << controller_interface_.controllers_protobuf_version_);
        break;
      case NO_MSG_VERSION:
        ROS_WARN_THROTTLE(2, "No protobuf message version received!");
        break;
      case WRONG_RESPONSE_ID:
        ROS_ERROR("Wrong response_id in protobuf measure");
        break;
      case MEASURE_SYNCHRONIZED_AGAIN:
        ROS_INFO("response_id in protobuf is synchronized again");
        break;
      case IMUS_AND_YAWRATE_ERROR:
        ROS_ERROR(
            "Measure has yaw_rate/acceleration and imus at the same time!");
        break;
      case SERIAL_PORT_DOES_NOT_EXIST:
        ROS_FATAL_THROTTLE(2, "Serial port not found!");
        break;
      default:
        ROS_WARN("unknown message appeared");
    }
  }
}

void ControllerInterfaceNode::controllerInterfacePublisherLoop() {
  while (!boost::this_thread::interruption_requested()) {

    controller_msgs::StateMeasure state_measure;
    measures_handler_.getSensorMessage(state_measure);
    state_measure_publisher_.publish(state_measure);

    // manual mode
    std_msgs::Bool manual_mode;
    if (measures_handler_.getManualMode(manual_mode)) {
      manual_mode_publisher_.publish(manual_mode);

      if (leftManualMode(manual_mode)) {
        std_msgs::Empty empty_msg;
        auto_reset_publisher_.publish(empty_msg);
      }
    }
  }
}

void ControllerInterfaceNode::controllerInterfaceSnapshotLoop() {
  while (!boost::this_thread::interruption_requested()) {
    // this has to live in a separate thread since the thread is sleeping
    // while waiting for new snapshot request messages
    if (measures_handler_.getSnapshotRequest()) {
      std_msgs::Empty empty_msg;
      snapshot_request_publisher_.publish(empty_msg);
    }
  }
}

void ControllerInterfaceNode::controllerInterfaceMissionModePublisherLoop() {
  while (!boost::this_thread::interruption_requested()) {
    common_msgs::MissionMode mission_mode;
    if (measures_handler_.getMissionMode(mission_mode)) {
      mission_mode.header.stamp = ros::Time::now();
      mission_mode_publisher_.publish(mission_mode);
    }
  }
}

void ControllerInterfaceNode::controllerInterfaceDistanceSensorsLoop() {
  while (!boost::this_thread::interruption_requested()) {
    measures_handler_.publishDistanceSensores(front_ir_sensor_publisher_,
                                              back_ir_sensor_publisher_,
                                              front_us_sensor_publisher_,
                                              back_us_sensor_publisher_);
  }
}

bool ControllerInterfaceNode::leftManualMode(const std_msgs::Bool& current_manual_mode_state) {
  const bool result = !current_manual_mode_state.data && last_manual_mode_;
  last_manual_mode_ = current_manual_mode_state.data;
  return result;
}

void ControllerInterfaceNode::sendIdleMode() {
  common_msgs::MissionMode idle_mode_msg;
  idle_mode_msg.mission_mode = common_msgs::MissionMode::IDLE;
  idle_mode_msg.header.stamp = ros::Time::now();
  command_handler_.setMissionMode(idle_mode_msg);
}


void ControllerInterfaceNode::onEnginePowerReceived(const common_msgs::Float32Stamped::ConstPtr& msg) {
  ROS_WARN_THROTTLE(5,
                    "Using engine_power from ROS topic! Make sure "
                    "speed_controller is not running!");
  command_handler_.engine_power_.write(msg->data);
}

void ControllerInterfaceNode::onSteeringControlReceived(const common_msgs::Float32Stamped::ConstPtr& msg) {
  ROS_WARN_THROTTLE(5,
                    "Using steering_control from ROS topic! Make sure "
                    "steering_controller is not running!");
  command_handler_.steering_control_front_.write(msg->data);
}

void ControllerInterfaceNode::onBackSteeringControlReceived(
    const common_msgs::Float32Stamped::ConstPtr& msg) {
  if (!command_handler_.use_steering_control_back) {
    ROS_ERROR(
        "ControllerInterfaceNode: Can not set sterring_control_back because "
        "IPC does not exist.");
    return;
  }

  ROS_WARN_THROTTLE(5,
                    "Using steering_control from ROS topic! Make sure "
                    "steering_controller is not running!");
  command_handler_.steering_control_back_.write(msg->data);
}

void ControllerInterfaceNode::onEngineBrakeReceived(const common_msgs::Float32Stamped::ConstPtr& msg) {
  ROS_WARN_THROTTLE(5,
                    "Using engine_brake from ROS topic! Make sure "
                    "speed_controller is not running!");
  command_handler_.engine_brake_.write(msg->data);
}

void ControllerInterfaceNode::onMissionMode(const common_msgs::MissionMode::ConstPtr& msg) {
  command_handler_.setMissionMode(*msg);
}

void ControllerInterfaceNode::onLightsCommandReceived(const controller_msgs::LightsCommandConstPtr& msg) {
  command_handler_.setLightsCommand(*msg);
}

bool ControllerInterfaceNode::onBrakeLightCommandReceived(std_srvs::SetBool::Request& request,
                                                          std_srvs::SetBool::Response& response) {
  command_handler_.setBrakeLightsCommand(request.data);
  response.success = true;
  response.message = "";
  return true;
}

void ControllerInterfaceNode::onDebugLightCommandReceived(const std_msgs::ColorRGBA::ConstPtr& msg) {
  command_handler_.setDebugLightColorCommand(*msg);
}

CREATE_NODE(ControllerInterfaceNode)
