#pragma once

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "common_msgs/ActivationService.h"

class TesteeModule {
 public:
  TesteeModule(ros::NodeHandle& nh, const std::string& module_name, const double timeout = 1.0)
      : timeout_(timeout),
        module_(nh.serviceClient<common_msgs::ActivationService>(
            module_name + "/activate_module")) {
    common_msgs::ActivationService activation_service;
    activation_service.request.moduleActive = true;
    EXPECT_TRUE(module_.waitForExistence(timeout_))
        << " could not call '" << module_.getService() << "' for activation.";
    EXPECT_TRUE(module_.call(activation_service))
        << "activation call failed: " << module_.getService();
  }

  ~TesteeModule() {
    common_msgs::ActivationService activation_service;
    activation_service.request.moduleActive = false;
    EXPECT_TRUE(module_.waitForExistence(timeout_))
        << " could not call '" << module_.getService() << "' for deactivation.";
    EXPECT_TRUE(module_.call(activation_service)) << "deactivation call failed: "
                                                  << module_.getService();
  }

 private:
  const ros::Duration timeout_;
  ros::ServiceClient module_;
};
