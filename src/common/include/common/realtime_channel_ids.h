#pragma once

// These channels are used between the controller interface and the state
// estimation
#define CHANNEL_ID_SENSOR_MEASUREMENTS 0x40001

// These channels are used between the state estimation and the speed controller
#define CHANNEL_ID_SPEED_MEASURE 0x50001
#define CHANNEL_ID_ENGINE_POWER 0x50002
#define CHANNEL_ID_ENGINE_BRAKE 0x50003
#define CHANNEL_ID_BRAKE_LIGHTS 0x50004
#define CHANNEL_ID_ACCELERATION_MEASURE 0x50005

// These channels are used between the state estimation and the steering
// controller
#define CHANNEL_ID_YAW_RATE_MEASURE 0x60001
#define CHANNEL_ID_STEERING_SERVO_OUTPUT 0x60002
#define CHANNEL_ID_STEERING_BACK_SERVO_OUTPUT 0x60003
#define CHANNEL_ID_STEERING_ANGLE_MEASURE_FRONT 0x60004
#define CHANNEL_ID_STEERING_ANGLE_MEASURE_BACK 0x60005

// These channels are used between the controller interface and the speed and
// steering controller
#define CHANNEL_ID_MANUAL_MODE 0x70001
