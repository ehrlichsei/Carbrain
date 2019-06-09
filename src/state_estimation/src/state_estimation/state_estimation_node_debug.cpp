#include "state_estimation_node_debug.h"
#include "state_estimation_msgs/State.h"
#include "state_estimation.h"
#include "state_estimation_node.h"
#include <state_estimation_msgs/ProcessSensorData.h>

#include "common/best_score.h"

constexpr int StateEstimationNodeDebug::accumulation_window;

StateEstimationNodeDebug::StateEstimationNodeDebug(ros::NodeHandle& node_handle)
    : StateEstimationNode(node_handle, std::unique_ptr<StateEstimation>()),
      time_of_last_measurement(0, 0),
      kalman_update_rate_check_acc(acc::tag::rolling_window::window_size = accumulation_window),
      meas_check("measurement"),
      front_servo_check("front servo command"),
      back_servo_check("back servo command"),
      engine_check("engine power command") {
  state_estimation_ =
      std::make_unique<StateEstimationDebug>(&parameter_handler_, &node_handle_);
}

void StateEstimationNodeDebug::startModule() {
  StateEstimationNode::startModule();
  node_handle_.param("debug_check_seq_numbers", check_seq_numbers, false);
  node_handle_.param("message_sync_rate_multiplier", message_sync_rate_multiplier, 4.0);

  state_measure_sub = node_handle_.subscribe(
      "state_measure", 40, &StateEstimationNodeDebug::stateMeasureCallback, this);
  front_servo_cmd_sub = node_handle_.subscribe(
      "front_steering_servo_command", 40, &StateEstimationNodeDebug::frontServoCallback, this);
  back_servo_cmd_sub = node_handle_.subscribe(
      "back_steering_servo_command", 40, &StateEstimationNodeDebug::backServoCallback, this);
  engine_cmd_sub = node_handle_.subscribe(
      "engine_command", 40, &StateEstimationNodeDebug::engineCallback, this);

  sync_timer = node_handle_.createTimer(
      ros::Duration(1. / (message_sync_rate_multiplier * state_estimation_->getUpdateRate())),
      &StateEstimationNodeDebug::syncTimerCallback,
      this);
}

void StateEstimationNodeDebug::stopModule() {
  StateEstimationNode::stopModule();
  state_measure_sub.shutdown();
  front_servo_cmd_sub.shutdown();
  back_servo_cmd_sub.shutdown();
  engine_cmd_sub.shutdown();
}

void StateEstimationNodeDebug::resetMsgCaches() {
  state_measure_cache.clear();
  front_servo_cache.clear();
  back_servo_cache.clear();
  engine_cache.clear();
}

void StateEstimationNodeDebug::handleMessages(
    const controller_msgs::StateMeasure& meas,
    const boost::optional<common_msgs::Float32Stamped>& fservo,
    const boost::optional<common_msgs::Float32Stamped>& rservo,
    const boost::optional<common_msgs::Float32Stamped>& engine) {
  state_estimation_->updateParameters();

  if (!time_of_last_measurement.isZero()) {
    time_since_last_measurement = meas.header.stamp - time_of_last_measurement;
    if (time_since_last_measurement.toSec() < 0.0) {
      ROS_WARN("detected jump back in time - reset kalman filter");
      state_estimation_->resetKalmanFilter();
    } else if (time_since_last_measurement.toSec() >
               2.0 * 1.0 / state_estimation_->getUpdateRate()) {
      ROS_WARN(
          "last two measurements were %f seconds apart - this is more than two "
          "times the expected duration - your rosbag might be missing a few "
          "measurements",
          time_since_last_measurement.toSec());
    } else {
      kalman_update_rate_check_acc(time_since_last_measurement.toSec());
      if (boost::accumulators::rolling_count(kalman_update_rate_check_acc) >= accumulation_window) {
        const double mean_update_duration =
            boost::accumulators::rolling_mean(kalman_update_rate_check_acc);
        const double mean_update_rate = 1.0 / mean_update_duration;
        const double update_rate_error =
            std::fabs(mean_update_rate - state_estimation_->getUpdateRate());
        constexpr double update_duration_threshold = 0.8;
        if (update_rate_error > update_duration_threshold) {
          ROS_ERROR_THROTTLE(4,
                             "the debug_measurement_rate specified at startup "
                             "(%dHz) doesn't seem to "
                             "be consistent with the actual measurement "
                             "publishing rate (approx. %.2fHz) - please start "
                             "the state estimation debug node with the correct "
                             "debug_measurement_rate, results will be "
                             "inaccurate otherwise",
                             state_estimation_->getUpdateRate(),
                             mean_update_rate);
        }
      }
      seqNumberPostFilteringCheck(meas, fservo, rservo, engine);
    }
  }
  time_of_last_measurement = meas.header.stamp;

  auto opt_msg_to_float = [](const auto& msg) {
    if (msg) {
      return msg->data;
    } else {
      return std::numeric_limits<float>::quiet_NaN();
    }
  };

  const SensorMeasurements sensor_measurements = fromMsg(meas);
  const float front_servo_command = opt_msg_to_float(fservo);
  const float back_servo_command = opt_msg_to_float(rservo);
  const float engine_command = opt_msg_to_float(engine);

  VehicleState estimated_state = state_estimation_->performKalmanIteration(
      sensor_measurements, front_servo_command, back_servo_command, engine_command);

  state_estimation_->estimated_state_results_queue_.push(estimated_state);
}


void StateEstimationNodeDebug::stateMeasureCallback(const controller_msgs::StateMeasure& msg) {
  if (!meas_check.seqNumberPreFilteringCheck(msg.header.seq)) {
    ROS_INFO(
        "resetting message caches for all command topics and state measure "
        "topic");
    resetMsgCaches();
  }
  state_measure_cache.push_back(msg);
}

void StateEstimationNodeDebug::frontServoCallback(const common_msgs::Float32Stamped& msg) {
  if (!front_servo_check.seqNumberPreFilteringCheck(msg.header.seq)) {
    ROS_INFO("resetting message cache for front servo command topic");
    front_servo_cache.clear();
  }
  front_servo_cache.push_back(msg);
}

void StateEstimationNodeDebug::backServoCallback(const common_msgs::Float32Stamped& msg) {
  if (!back_servo_check.seqNumberPreFilteringCheck(msg.header.seq)) {
    ROS_INFO("resetting message cache for back servo command topic");
    back_servo_cache.clear();
  }
  back_servo_cache.push_back(msg);
}

void StateEstimationNodeDebug::engineCallback(const common_msgs::Float32Stamped& msg) {
  if (!engine_check.seqNumberPreFilteringCheck(msg.header.seq)) {
    ROS_INFO("resetting message cache for engine command topic");
    engine_cache.clear();
  }
  engine_cache.push_back(msg);
}

void StateEstimationNodeDebug::syncTimerCallback(const ros::TimerEvent&) {
  if (state_measure_cache.empty()) {
    return;
  }
  const double delta_t = 1. / state_estimation_->getUpdateRate();
  controller_msgs::StateMeasure meas = state_measure_cache.front();

  using CmdIt = std::deque<common_msgs::Float32Stamped>::const_iterator;

  const auto check_for_msgs = [meas, delta_t, this](
      boost::optional<CmdIt>& it, const ros::Subscriber sub, const auto& cache) {
    if (sub.getNumPublishers() == 0 && cache.empty()) {
      // ignore topic, if there aren't any publishers and no messages left in
      // the cache
      it = boost::none;
    } else {
      // find command topic message with best matching time stamp
      it = common::min_score(
          cache,
          [meas](const auto& e) {
            return std::fabs((e.header.stamp - meas.header.stamp).toSec());
          });
      if (it == cache.end() ||
          (it == cache.end() - 1 && (*it)->header.stamp < meas.header.stamp)) {
        // if the last command topic message is older than the state measure
        // message, skip this iteration to wait for the next message, in case
        // that command message has a time stamp closer to the state measure
        // message
        return true;
      } else if (meas.header.stamp < (*it)->header.stamp - ros::Duration(delta_t)) {
        ROS_WARN("skipped too old state measure");
        state_measure_cache.pop_front();
        return true;
      }
    }
    return false;
  };

  boost::optional<CmdIt> fs_it;
  if (check_for_msgs(fs_it, front_servo_cmd_sub, front_servo_cache)) {
    return;
  }

  boost::optional<CmdIt> bs_it;
  if (check_for_msgs(bs_it, back_servo_cmd_sub, back_servo_cache)) {
    return;
  }

  boost::optional<CmdIt> eng_it;
  if (check_for_msgs(eng_it, engine_cmd_sub, engine_cache)) {
    return;
  }

  boost::optional<common_msgs::Float32Stamped> fservo;
  if (fs_it) {
    fservo = **fs_it;
    front_servo_cache.erase(front_servo_cache.begin(), (*fs_it) + 1);
  }

  boost::optional<common_msgs::Float32Stamped> rservo;
  if (bs_it) {
    rservo = **bs_it;
    back_servo_cache.erase(back_servo_cache.begin(), (*bs_it) + 1);
  }

  boost::optional<common_msgs::Float32Stamped> engine;
  if (eng_it) {
    engine = **eng_it;
    engine_cache.erase(engine_cache.begin(), (*eng_it) + 1);
  }

  state_measure_cache.pop_front();
  handleMessages(meas, fservo, rservo, engine);
}

void StateEstimationNodeDebug::seqNumberPostFilteringCheck(
    const controller_msgs::StateMeasure& meas,
    const boost::optional<common_msgs::Float32Stamped>& fservo,
    const boost::optional<common_msgs::Float32Stamped>& rservo,
    const boost::optional<common_msgs::Float32Stamped>& engine) {
  if (!meas_check.initialized()) {
    meas_check.resetSeqCounter(meas.header.seq);
  }
  if (!front_servo_check.initialized() && fservo) {
    front_servo_check.resetSeqCounter(fservo->header.seq);
  }
  if (!back_servo_check.initialized() && rservo) {
    back_servo_check.resetSeqCounter(rservo->header.seq);
  }
  if (!engine_check.initialized() && engine) {
    engine_check.resetSeqCounter(engine->header.seq);
  }

  if (check_seq_numbers) {
    meas_check.seqNumberPostFilteringCheck(meas.header.seq);
    if (fservo) {
      front_servo_check.seqNumberPostFilteringCheck(fservo->header.seq);
    }
    if (rservo) {
      back_servo_check.seqNumberPostFilteringCheck(rservo->header.seq);
    }
    if (engine) {
      engine_check.seqNumberPostFilteringCheck(engine->header.seq);
    }
  }
}

void StateEstimationNodeDebug::SeqNumberCheck::seqNumberPostFilteringCheck(uint32_t input_seq) {
  const long long msg_err =
      static_cast<long long>(input_seq) - static_cast<long long>(msg_seq);
  if (msg_err > 0) {
    acc_msg_drop_err += msg_err;
    ROS_WARN(
        "%s messages coming in are not sequential:"
        " dropped %lli message(s) now, that's %.2f%% in total on this topic",
        topic_name.c_str(),
        msg_err,
        acc_msg_drop_err * 100.f / (input_seq - init_msg_seq));
  } else if (msg_err < 0) {
    acc_msg_reused_err++;
    ROS_WARN("reused %i %s message(s) in total", acc_msg_reused_err, topic_name.c_str());
  }
  msg_seq = input_seq + 1;
}

bool StateEstimationNodeDebug::SeqNumberCheck::seqNumberPreFilteringCheck(uint32_t seq) {
  const long long diff = static_cast<long long>(seq) - static_cast<long long>(last_seq);
  last_seq = seq;
  if (diff > 1) {
    ROS_WARN(
        "message sequence number skipped on topic %s: lost message at input",
        topic_name.c_str());
    return true;
  } else if (diff < 0) {
    ROS_WARN("sequence number not in sequence on topic %s", topic_name.c_str());
    resetSeqCounter(seq);
    return false;
  }
  return true;
}

void StateEstimationNodeDebug::SeqNumberCheck::resetSeqCounter(uint32_t seq) {
  acc_msg_drop_err = 0;
  acc_msg_reused_err = 0;
  init_msg_seq = msg_seq = last_seq = seq;
  init = true;
  ROS_INFO("reset seq counter");
}
