#include "state_estimation_node.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <common/node_creation_makros.h>
#include <state_estimation_msgs/State.h>
#include "state_estimation/StateEstimationConfig.h"
THIRD_PARTY_HEADERS_END

#include "state.h"
#include "state_estimation_node_debug.h"

StateEstimationNode::StateEstimationNode(ros::NodeHandle &node_handle)
    : NodeBase(node_handle),
      state_estimation_(std::make_unique<StateEstimationRealtime>(&parameter_handler_)) {
  init();
}

StateEstimationNode::StateEstimationNode(ros::NodeHandle &node_handle,
                                         std::unique_ptr<StateEstimation> state_estimation_)
    : NodeBase(node_handle), state_estimation_(std::move(state_estimation_)) {
  init();
}
// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
StateEstimationNode::~StateEstimationNode() { StateEstimationNode::stopModule(); }

void StateEstimationNode::init() {
  if (!isInTestMode()) {
    /*parameter_handler_.addDynamicReconfigureServer<state_estimation::StateEstimationConfig>(
        node_handle_);*/
  }
}

void StateEstimationNode::startModule() {
  if (isInTestMode()) {
    ROS_INFO("Starting in integration test mode");
    test_service_server = node_handle_.advertiseService(
        "process_sensor_data", &StateEstimationNode::processSensorData, this);
    if (!test_service_server) {
      ROS_ERROR("Can not start service process_sensor_data");
    }
    return;
  }
  state_estimation_publisher_ =
      node_handle_.advertise<state_estimation_msgs::State>("state_estimation", 5, true);
  state_estimation_publisher_thread_ = boost::thread(
      boost::bind(&StateEstimationNode::stateEstimationPublisherLoop, this));

  state_estimation_->start();
}

void StateEstimationNode::stopModule() {
  state_estimation_publisher_thread_.interrupt();
  state_estimation_publisher_thread_.join();
  state_estimation_->stop();
  state_estimation_publisher_.shutdown();
}


const std::string StateEstimationNode::getName() {
  return std::string("state_estimation");
}

void StateEstimationNode::stateEstimationPublisherLoop() {
  while (!boost::this_thread::interruption_requested()) {
    VehicleState estimation;
    if (state_estimation_->getNextEstimatedState(&estimation)) {
      state_estimation_msgs::State msg;
      msg.header.stamp = estimation.stamp;
      msg.header.frame_id = "vehicle";
      msg.speed_x = estimation.speed_x;
      msg.speed_y = estimation.speed_y;
      msg.yaw_rate = estimation.yaw_rate;
      msg.acceleration = estimation.acceleration;
      msg.steering_angle_back = estimation.steering_angle_back;
      msg.steering_angle_front = estimation.steering_angle_front;
      state_estimation_publisher_.publish(msg);
    }
  }
}

SensorMeasurements fromMsg(const controller_msgs::StateMeasure &msg) {
  return {
      .stamp = msg.header.stamp,
      .left_IMU = {.angular_velocity = {.x = msg.imu_left.angular_velocity.x,
                                        .y = msg.imu_left.angular_velocity.y,
                                        .z = msg.imu_left.angular_velocity.z},
                   .acceleration = {.x = msg.imu_left.linear_acceleration.x,
                                    .y = msg.imu_left.linear_acceleration.y,
                                    .z = msg.imu_left.linear_acceleration.z}},
      .right_IMU = {.angular_velocity = {.x = msg.imu_right.angular_velocity.x,
                                         .y = msg.imu_right.angular_velocity.y,
                                         .z = msg.imu_right.angular_velocity.z},
                    .acceleration = {.x = msg.imu_right.linear_acceleration.x,
                                     .y = msg.imu_right.linear_acceleration.y,
                                     .z = msg.imu_right.linear_acceleration.z}},
      .angular_wheel_speeds = {.front = {.left = msg.wheel_speeds.front.left,
                                         .right = msg.wheel_speeds.front.right},
                               .back = {.left = msg.wheel_speeds.back.left,
                                        .right = msg.wheel_speeds.back.right}},
      .steering_angles = {.front = {.left = msg.steering_angles.front.left,
                                    .right = msg.steering_angles.front.right},
                          .back = {.left = msg.steering_angles.back.left,
                                   .right = msg.steering_angles.back.right}}};
}

// only relevant for integration tests: callback of service process_sensor_data
bool StateEstimationNode::processSensorData(
    state_estimation_msgs::ProcessSensorDataRequest &request,
    state_estimation_msgs::ProcessSensorDataResponse &response) {

  state_estimation_->updateParameters();

  std::vector<std::tuple<SensorMeasurements, float, float, float>> input_data;
  input_data.reserve(request.measures.size());

  const auto &fs_commands = request.front_steering_servo_commands;
  const auto &bs_commands = request.back_steering_servo_commands;
  const auto &eng_commands = request.engine_commands;

  auto fs_it = fs_commands.begin();
  auto bs_it = bs_commands.begin();
  auto eng_it = bs_commands.begin();

  for (const auto &measure : request.measures) {
    auto getCommandForMeasure = [&measure](const auto &commands, auto &it) {
      it = std::find_if(it, commands.end(), [&measure](const auto &c) {
        return c.header.stamp >= measure.header.stamp;
      });
      if (commands.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
      } else if (it == commands.end()) {
        return commands.back().data;
      } else {
        return it->data;
      }
    };

    const float fs_command = getCommandForMeasure(fs_commands, fs_it);
    const float bs_command = getCommandForMeasure(bs_commands, bs_it);
    const float eng_command = getCommandForMeasure(eng_commands, eng_it);

    input_data.emplace_back(fromMsg(measure), fs_command, bs_command, eng_command);
  }

  std::vector<VehicleState> intermediate_states =
      state_estimation_->processSensorData(input_data);

  if (state_estimation_->getUpdateRate() != request.rate) {
    ROS_ERROR(
        "state_estimation was compiled with an update rate of %d hz. The "
        "service was called with a sensor rate of %d hz.",
        state_estimation_->getUpdateRate(),
        request.rate);
    return false;
  }

  response.states =
      std::vector<state_estimation_msgs::State>(intermediate_states.size());
  ros::Time time = ros::Time::now();
  for (size_t i = 0; i < intermediate_states.size(); i++) {
    VehicleState vehicle_state = intermediate_states[i];
    state_estimation_msgs::State state;
    state.acceleration = vehicle_state.acceleration;
    state.speed_x = vehicle_state.speed_x;
    state.speed_y = vehicle_state.speed_y;
    state.steering_angle_back = vehicle_state.steering_angle_back;
    state.steering_angle_front = vehicle_state.steering_angle_front;
    state.yaw_rate = vehicle_state.yaw_rate;
    state.header.stamp = time + ros::Duration(i * (1.0 / request.rate));
    response.states[i] = (state);
  }
  return true;
}

bool StateEstimationNode::isInTestMode() const {
  bool test_mode;
  node_handle_.param("integration_test_mode", test_mode, false);
  return test_mode;
}


CREATE_NODE_WITH_FANCY_DEBUG(StateEstimationNode, StateEstimationNodeDebug)
