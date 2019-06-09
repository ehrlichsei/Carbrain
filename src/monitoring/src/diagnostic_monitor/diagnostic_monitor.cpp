#include "diagnostic_monitor.h"

#include <cstdlib>
#include <string>
#include "boost/optional.hpp"
#include "common/led_colors.h"
#include "std_msgs/Bool.h"

static ParameterString<double> THROTTLE_TIME_VOICE("throttle_time_voice");

DiagnosticMonitor::DiagnosticMonitor(ParameterInterface* parameters)
    : parameters_ptr_(parameters),
      led(boost::none),
      voice(boost::none),
      old_voice(boost::none),
      espeak_available_(true) {
  parameters_ptr_->registerParam(THROTTLE_TIME_VOICE);
}

void DiagnosticMonitor::onDiagnosticArray(const diagnostic_msgs::DiagnosticArray array) {
  for (const auto& status : array.status) {
    for (const auto& entry : status.values) {
      if (entry.key == "voice") {
        if (voice == boost::none || (*voice).level < status.level) {
          voice = status;
        }
      } else if (entry.key == "led_r") {
        if (led == boost::none || (*led).level < status.level) {
          led = status;
        }
      }
    }
  }
}

void DiagnosticMonitor::onEmergencyStopReceived(const std_msgs::Bool::ConstPtr& msg,
                                                ros::Publisher& led_publisher_) {
  const std_msgs::ColorRGBA color = msg->data ? led_color::RED : led_color::NONE;
  led_publisher_.publish(color);
}

boost::optional<std_msgs::ColorRGBA> DiagnosticMonitor::draw(const ros::TimerEvent& /*event*/) {
  if (!led) {
    return boost::none;
  }
  std_msgs::ColorRGBA color;
  for (const auto& entry : (*led).values) {
    if (entry.key == "led_r") {
      color.r = std::stof(entry.value);
    } else if (entry.key == "led_g") {
      color.g = std::stof(entry.value);
    } else if (entry.key == "led_b") {
      color.b = std::stof(entry.value);
    }
  }
  led = boost::none;
  return color;
}

bool DiagnosticMonitor::equal(const OptionalDiagnosticStatus& first,
                              const OptionalDiagnosticStatus& second) {
  return (first and second) and (first->message == second->message);
}

void DiagnosticMonitor::speak(const ros::TimerEvent& /* event*/) {
  if (!espeak_available_ || !voice) {
    return;
  }
  const double waiting_time = parameters_ptr_->getParam(THROTTLE_TIME_VOICE);
  if (equal(voice, old_voice) && (ros::Time::now() - old_voice_stamp).toSec() < waiting_time) {
    return;
  }
  std::replace(voice->message.begin(), voice->message.end(), '"', ' ');
  int return_value = system(("espeak \"" + voice->message + "\"").c_str());
  old_voice = voice;
  voice = boost::none;
  old_voice_stamp = ros::Time::now();
  if (return_value != 0) {
    ROS_WARN_THROTTLE(10, "can't call espeak");
    espeak_available_ = false;
  } else {
    ROS_DEBUG_THROTTLE(10, "can call espeak");
  }
}
