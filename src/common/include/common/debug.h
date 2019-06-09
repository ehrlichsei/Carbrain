#pragma once

#include <iostream>
#include <ros/debug.h>

#define DUMP_F(x) ROS_DEBUG("%s = %f", #x, static_cast<float>(x))
#define DUMP_D(x) ROS_DEBUG("%s = %f", #x, static_cast<double>(x))
#define DUMP_I(x) ROS_DEBUG("%s = %d", #x, static_cast<int>(x))
#define DUMP_S(x) ROS_DEBUG("%s = %s", #x, x)
#define DUMP(x)   ROS_DEBUG_STREAM(#x << " = " << x << std::endl)
