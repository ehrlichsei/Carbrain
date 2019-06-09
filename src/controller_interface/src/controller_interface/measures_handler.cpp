#include "measures_handler.h"

MeasuresHandler::MeasuresHandler()
    : sensor_measurements_ipc_(CHANNEL_ID_SENSOR_MEASUREMENTS),
      manual_mode_for_control_(CHANNEL_ID_MANUAL_MODE) {}

SensorMeasurements::IMU fromProtobuf(const kitcar::Measure_IMU& proto) {
  return {.angular_velocity = {.x = proto.angular_velocity_x(),
                               .y = proto.angular_velocity_y(),
                               .z = proto.angular_velocity_z()},
          .acceleration = {.x = proto.acceleration_x(),
                           .y = proto.acceleration_y(),
                           .z = proto.acceleration_z()}};
}

SensorMeasurements::Axle fromProtobuf(const kitcar::Measure_AngularWheelSpeeds& proto) {
  return {.left = proto.left(), .right = proto.right()};
}

void MeasuresHandler::handleReceivedMeasures(const kitcar::Measure& protobuf_measure,
                                             const ros::Time& stamp) {

  if ((protobuf_measure.has_yaw_rate() || protobuf_measure.has_acceleration()) &&
      (protobuf_measure.has_imu_left() || protobuf_measure.has_imu_right())) {
    log_->push(IMUS_AND_YAWRATE_ERROR);
  }

  // handles measures
  if (((protobuf_measure.has_yaw_rate() && protobuf_measure.has_acceleration()) ||
       (protobuf_measure.has_imu_left() && protobuf_measure.has_imu_right())) &&
      (protobuf_measure.has_speed_back() || protobuf_measure.has_speed_front()) &&
      protobuf_measure.has_manual_mode()) {
    SensorMeasurements sensor_measurements;
    sensor_measurements.stamp = stamp;
    if (protobuf_measure.has_yaw_rate() && protobuf_measure.has_acceleration()) {
      sensor_measurements.left_IMU.angular_velocity.z = protobuf_measure.yaw_rate();
      sensor_measurements.right_IMU.angular_velocity.z = protobuf_measure.yaw_rate();
      sensor_measurements.left_IMU.acceleration.x = protobuf_measure.acceleration();
      sensor_measurements.right_IMU.acceleration.x = protobuf_measure.acceleration();
    } else {
      sensor_measurements.left_IMU = fromProtobuf(protobuf_measure.imu_left());
      sensor_measurements.right_IMU = fromProtobuf(protobuf_measure.imu_right());
    }

    sensor_measurements.angular_wheel_speeds.back =
        fromProtobuf(protobuf_measure.speed_back());

    if (protobuf_measure.has_speed_front()) {
      sensor_measurements.angular_wheel_speeds.front =
          fromProtobuf(protobuf_measure.speed_front());
    }

    if (protobuf_measure.has_servo_front()) {
      sensor_measurements.steering_angles.front.left = protobuf_measure.servo_front();
      sensor_measurements.steering_angles.front.right = protobuf_measure.servo_front();
    }

    if (protobuf_measure.has_servo_back()) {
      sensor_measurements.steering_angles.back.left = protobuf_measure.servo_back();
      sensor_measurements.steering_angles.back.right = protobuf_measure.servo_back();
    }

    sensor_measurements.manual_mode = protobuf_measure.manual_mode();

    if (write_to_shared_memory_) {
      // write data to shared memory for realtime processes
      sensor_measurements_ipc_.write(sensor_measurements);
      manual_mode_for_control_.write(sensor_measurements.manual_mode);
    }
    // this will cause publishing the date via ros
    sensor_measurements_queue_.push(sensor_measurements);
    // TODO add stamp here ?
    manual_mode_queue_.push(protobuf_measure.manual_mode());
  }

  // TODO add stamp here
  // handle distance sensors
  distance_sensors_queue_.push(protobuf_measure.distance_sensors());

  // handle mission mode
  if (protobuf_measure.has_mode()) {
    // TODO add stamp here ?
    mode_queue_.push(protobuf_measure.mode());
  }

  // handle snapshot request
  if(protobuf_measure.has_snapshot_request()){
    snapshot_request_queue_.push(protobuf_measure.snapshot_request());
  }
}

void MeasuresHandler::setWriteToSharedMemory(bool write_to_shared_memory) {
  write_to_shared_memory_ = write_to_shared_memory;
}

void MeasuresHandler::setLog(LoggingQueue* log) { log_ = log; }

sensor_msgs::Imu fromStruct(const SensorMeasurements::IMU& imu_struct,
                            const ros::Time& stamp) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.orientation.w = 1;
  imu_msg.angular_velocity.x = imu_struct.angular_velocity.x;
  imu_msg.angular_velocity.y = imu_struct.angular_velocity.y;
  imu_msg.angular_velocity.z = imu_struct.angular_velocity.z;
  imu_msg.linear_acceleration.x = imu_struct.acceleration.x;
  imu_msg.linear_acceleration.y = imu_struct.acceleration.y;
  imu_msg.linear_acceleration.z = imu_struct.acceleration.z;
  imu_msg.orientation_covariance.assign(-1);
  imu_msg.angular_velocity_covariance.assign(-1);
  imu_msg.linear_acceleration_covariance.assign(-1);
  return imu_msg;
}

controller_msgs::VectorLR fromStruct(const SensorMeasurements::Axle& axle) {
  controller_msgs::VectorLR msg;
  msg.left = static_cast<float>(axle.left);
  msg.right = static_cast<float>(axle.right);
  return msg;
}

controller_msgs::WheelSpeeds fromStruct(const SensorMeasurements::AngularWheelSpeeds& angular_wheel_speeds) {
  controller_msgs::WheelSpeeds msg;
  msg.front = fromStruct(angular_wheel_speeds.front);
  msg.back = fromStruct(angular_wheel_speeds.back);
  return msg;
}

controller_msgs::SteeringAngles fromStruct(const SensorMeasurements::SteeringAngles& steering_angles) {
  controller_msgs::SteeringAngles msg;
  msg.front = fromStruct(steering_angles.front);
  msg.back = fromStruct(steering_angles.back);
  return msg;
}

void MeasuresHandler::getSensorMessage(controller_msgs::StateMeasure& state_measure) {
  SensorMeasurements sensor_measurements;
  if (sensor_measurements_queue_.pop(&sensor_measurements)) {
    const ros::Time& stamp = sensor_measurements.stamp;
    state_measure.header.stamp = stamp;

    state_measure.imu_left = fromStruct(sensor_measurements.left_IMU, stamp);
    state_measure.imu_right = fromStruct(sensor_measurements.right_IMU, stamp);
    state_measure.wheel_speeds = fromStruct(sensor_measurements.angular_wheel_speeds);
    state_measure.steering_angles = fromStruct(sensor_measurements.steering_angles);
  }
}

bool MeasuresHandler::getManualMode(std_msgs::Bool& manual_mode) {
  bool mode;
  if (manual_mode_queue_.pop(&mode)) {
    manual_mode.data = mode;
    return true;
  } else {
    return false;
  }
}

bool MeasuresHandler::getSnapshotRequest(){
  bool request;
  // the actual value of the snapshot field is ignored
  // as the field is only available when a request was made
  return snapshot_request_queue_.pop(&request);
}

bool MeasuresHandler::getMissionMode(common_msgs::MissionMode& mission_mode) {
  kitcar::Measure_Mode mode;
  if (mode_queue_.pop(&mode)) {
    switch (mode) {
      case kitcar::Measure_Mode_ROUND_TRIP:
        mission_mode.mission_mode = common_msgs::MissionMode::FREE_RIDE;
        break;
      case kitcar::Measure_Mode_ROUND_TRIP_WITH_OBSTACLES:
        mission_mode.mission_mode = common_msgs::MissionMode::OBSTACLE;
        break;
      case kitcar::Measure_Mode_PARKING:
        mission_mode.mission_mode = common_msgs::MissionMode::PARKING;
        break;
      case kitcar::Measure_Mode_IDLE:
        mission_mode.mission_mode = common_msgs::MissionMode::IDLE;
        break;
    }
    return true;
  } else {
    return false;
  }
}

void MeasuresHandler::publishDistanceSensores(ros::Publisher& front_ir_sensor_publisher_,
                                              ros::Publisher& back_ir_sensor_publisher_,
                                              ros::Publisher& front_us_sensor_publisher_,
                                              ros::Publisher& back_us_sensor_publisher_) {
  google::protobuf::RepeatedPtrField<kitcar::Measure_DistanceMeasure> distance_sensors;
  if (!distance_sensors_queue_.pop(&distance_sensors)) {
    return;
  }
  for (int i = 0; i < distance_sensors.size(); i++) {
    if (distance_sensors.Get(i).distance() >= 0) {
      sensor_msgs::Range range;

      range.range = distance_sensors.Get(i).distance();
      if (range.range < 0.0) {
        range.range = INFINITY;
      }

      range.header.stamp = ros::Time::now();

      switch (distance_sensors.Get(i).sensor_id()) {
        case kitcar::Measure_DistanceSensorId_RIGHT_FRONT_IR:
          range.radiation_type = sensor_msgs::Range::INFRARED;

          //! @todo add this sensor parameters (enables correct visualization)
          range.field_of_view = static_cast<float>(M_PI_4);
          range.min_range = 0.01f;
          range.max_range = 2.0f;

          range.header.frame_id = "ir_front";

          front_ir_sensor_publisher_.publish(range);

          break;
        case kitcar::Measure_DistanceSensorId_RIGHT_BACK_IR:
          range.radiation_type = sensor_msgs::Range::INFRARED;

          //! @todo add this sensor parameters (enables correct visualization)
          range.field_of_view = static_cast<float>(M_PI_4);
          range.min_range = 0.01f;
          range.max_range = 2.0f;

          range.header.frame_id = "ir_back";

          back_ir_sensor_publisher_.publish(range);

          break;
        case kitcar::Measure_DistanceSensorId_FRONT_US:
          range.radiation_type = sensor_msgs::Range::ULTRASOUND;

          //! @todo add this sensor parameters (enables correct visualization)
          range.field_of_view = 0.52f;
          range.min_range = 0.01f;
          range.max_range = 2.0f;

          range.header.frame_id = "ir_ahead";

          front_us_sensor_publisher_.publish(range);

          break;
        case kitcar::Measure_DistanceSensorId_BACK_US:
          range.radiation_type = sensor_msgs::Range::ULTRASOUND;

          //! @todo add this sensor parameters (enables correct visualization)
          range.field_of_view = 0.52f;
          range.min_range = 0.01f;
          range.max_range = 2.0f;

          range.header.frame_id = "ir_middle";

          back_us_sensor_publisher_.publish(range);

          break;
        default:  // NOLINT
          ROS_ERROR("controller send weird distance-sensor-id: %d",
                    distance_sensors.Get(i).sensor_id());
          break;
      }

      ROS_DEBUG("update for distance sensor %d published", i);
    } else {
      ROS_DEBUG("no update for distance sensor %d", i);
    }
  }
}
