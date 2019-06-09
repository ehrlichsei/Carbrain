#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletLoad.h>
#include <bondcpp/bond.h>
THIRD_PARTY_HEADERS_END

#include <common/parameter_handler.h>

DISABLE_SUGGEST_OVERRIDE_WARNING

/*!
 * \brief The TestLoadNodelet class
 * This test replaces a nodelet manager. It makes it possible to test, if a
 * nodelet can be loaded/initialized without errors. It is designed to be
 * compatible with the usual nodlet loading service call, so it can be used to
 * check, if a launchfile, which takes an nodelet manager as an argument, can
 * load the nodelet correctly.
 */
class TestLoadNodelet : public ::testing::Test {
 public:
  bool serviceUpload(nodelet::NodeletLoad::Request &req, nodelet::NodeletLoad::Response &res) {
    // build map of remaps (mapping old -> new name)
    nodelet::M_string remappings;
    if (req.remap_source_args.size() != req.remap_target_args.size()) {
      ROS_ERROR(
          "Bad remapppings provided, target and source of different length");
    } else {
      for (size_t i = 0; i < req.remap_source_args.size(); ++i) {
        remappings[ros::names::resolve(req.remap_source_args[i])] =
            ros::names::resolve(req.remap_target_args[i]);
        ROS_DEBUG("%s:%s\n",
                  ros::names::resolve(req.remap_source_args[i]).c_str(),
                  remappings[ros::names::resolve(req.remap_source_args[i])].c_str());
      }
    }

    pluginlib::ClassLoader<nodelet::Nodelet> loader("nodelet",
                                                    "nodelet::Nodelet");
    try {
      loader.createInstance(req.type)->init(req.name, remappings, req.my_argv);
      SUCCEED();
      std::cerr << "Succeeded loaded " << req.name << std::endl;
      ROS_INFO_STREAM("Succeeded loading: " + req.name);
    } catch (const std::exception &e) {
      ADD_FAILURE() << "loading nodelet threw exception: " << e.what() << std::endl;
    }
    res.success = true;
    finished = true;
    return true;
  }
  bool finished = false;
};



TEST_F(TestLoadNodelet, testLoadNodelet) {  // NOLINT
  ros::NodeHandle nh("~");
  ros::ServiceServer s = nh.advertiseService<TestLoadNodelet>(
      "load_nodelet", &TestLoadNodelet::serviceUpload, this);

  while (ros::ok() && !finished) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_TRUE(finished);
  std::cerr << "test_nodelet_loader: finished!" << std::endl;
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nodelet_loader");
  const auto result = RUN_ALL_TESTS();
  ros::Duration(0.1).sleep();
  return result;
}
