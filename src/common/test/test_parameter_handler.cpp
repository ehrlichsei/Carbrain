#include <common/parameter_handler.h>
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
#include <gtest/gtest.h>

#include "common/ParameterHandlerTestConfig.h"
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using common::node_base::ParameterHandler;

TEST(TestParameterHandler, testRegisterParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> bool_param("bool_test_param");
  ParameterString<int> int_param("int_test_param");
  ParameterString<double> double_param("double_test_param");
  ParameterString<bool> undefined_bool_param("bool_test_param_not_defined");

  EXPECT_NO_THROW(parameter_handler.registerParam(bool_param););
  EXPECT_ANY_THROW(parameter_handler.registerParam(undefined_bool_param););
  EXPECT_NO_THROW(parameter_handler.registerParam(int_param);
                  parameter_handler.registerParam(double_param););
}

TEST(TestParameterHandler, testLoadDefaultParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> bool_param("bool_test_param");
  ParameterString<int> int_param("int_test_param");
  ParameterString<double> double_param("double_test_param");
  ParameterString<std::string> string_param("string_test_param");

  parameter_handler.registerParam(bool_param);
  EXPECT_TRUE(parameter_handler.getParam(bool_param));
  EXPECT_EQ(parameter_handler.getParam(bool_param),
            parameter_handler.getParam(bool_param));

  parameter_handler.registerParam(int_param);
  EXPECT_EQ(parameter_handler.getParam(int_param), 1);

  parameter_handler.registerParam(double_param);
  EXPECT_EQ(parameter_handler.getParam(double_param), 0.3);

  parameter_handler.registerParam(string_param);
  EXPECT_EQ(parameter_handler.getParam(string_param), "testest");
}

TEST(TestParameterHandler, testChangeNamespace) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<double> double_param("double_test_param");
  ParameterString<int> int_param("int_test_param");

  parameter_handler.registerParam(double_param);
  EXPECT_EQ(parameter_handler.getParam(double_param), 0.3);

  parameter_handler.registerParam(int_param);
  EXPECT_EQ(parameter_handler.getParam(int_param), 1);

  parameter_handler.changeNamespace("free_ride_mode");
  EXPECT_EQ(parameter_handler.getParam(double_param), -5.5);
  EXPECT_EQ(parameter_handler.getParam(int_param), 1);

  parameter_handler.changeNamespace("default");
  EXPECT_EQ(parameter_handler.getParam(double_param), 0.3);
  EXPECT_EQ(parameter_handler.getParam(int_param), 1);
}

TEST(TestParameterHandler, testGlobalParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<double> global_double_param("/global/test/name_space/global_double_param");
  parameter_handler.registerParam(global_double_param);
  EXPECT_EQ(parameter_handler.getParam(global_double_param), 1.337);

  ParameterString<double> global_int_param("/global/test/name_space/global_int_param");
  parameter_handler.registerParam(global_int_param);
  EXPECT_EQ(parameter_handler.getParam(global_int_param), 42);

  EXPECT_ANY_THROW(parameter_handler.registerParam(
                       ParameterString<bool>("/global/test/name_space/global_bool_param")));

  parameter_handler.changeNamespace("parking_mode");
  EXPECT_EQ(parameter_handler.getParam(global_double_param), 7.331);
  EXPECT_EQ(parameter_handler.getParam(global_int_param), 47);
}

TEST(TestParameterHandler, testPackageParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<double> global_double_param("/common/double_param");
  parameter_handler.registerParam(global_double_param);
  EXPECT_EQ(parameter_handler.getParam(global_double_param), 13.37);

  ParameterString<double> global_int_param("/common/int_param");
  parameter_handler.registerParam(global_int_param);
  EXPECT_EQ(parameter_handler.getParam(global_int_param), 420);

  EXPECT_ANY_THROW(parameter_handler.registerParam(
                       ParameterString<bool>("/common/bool_param")));

  parameter_handler.changeNamespace("parking_mode");
  EXPECT_EQ(parameter_handler.getParam(global_double_param), 73.31);
  EXPECT_EQ(parameter_handler.getParam(global_int_param), 470);
}

TEST(TestParameterHandler, testVectorParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<std::vector<bool> > vector_bool_param("vector_bool_param");
  ParameterString<std::vector<int> > vector_int_param("vector_int_param");
  ParameterString<std::vector<double> > vector_double_param("vector_double_param");
  ParameterString<std::vector<std::string> > vector_string_param("vector_string_param");
  parameter_handler.registerParam(vector_bool_param);
  parameter_handler.registerParam(vector_int_param);
  parameter_handler.registerParam(vector_double_param);
  parameter_handler.registerParam(vector_string_param);
  std::vector<bool> vector_bool = parameter_handler.getParam(vector_bool_param);
  std::vector<int> vector_int = parameter_handler.getParam(vector_int_param);
  std::vector<double> vector_double = parameter_handler.getParam(vector_double_param);
  std::vector<std::string> vector_string = parameter_handler.getParam(vector_string_param);
  EXPECT_EQ(vector_bool[0], false);
  EXPECT_EQ(vector_bool[1], true);
  EXPECT_EQ(vector_int[0], 31);
  EXPECT_EQ(vector_int[1], -5);
  EXPECT_EQ(vector_double[0], 3.1);
  EXPECT_EQ(vector_double[1], 0.5);
  EXPECT_EQ(vector_string[0], "strange");
  EXPECT_EQ(vector_string[1], "string test");


  parameter_handler.changeNamespace("parking_mode");
  vector_bool = parameter_handler.getParam(vector_bool_param);
  vector_int = parameter_handler.getParam(vector_int_param);
  vector_double = parameter_handler.getParam(vector_double_param);
  vector_string = parameter_handler.getParam(vector_string_param);
  EXPECT_EQ(vector_bool[0], true);
  EXPECT_EQ(vector_bool.size(), 1);
  EXPECT_EQ(vector_int[0], 15);
  EXPECT_EQ(vector_int.size(), 1);
  EXPECT_EQ(vector_double[0], 1.5);
  EXPECT_EQ(vector_double.size(), 1);
  EXPECT_EQ(vector_string[0], "parking string");
  EXPECT_EQ(vector_string.size(), 1);
}

TEST(TestParameterHandler, testMapParameters) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<std::map<std::string, bool> > map_bool_param("map_bool_param");
  ParameterString<std::map<std::string, int> > map_int_param("map_int_param");
  ParameterString<std::map<std::string, double> > map_double_param("map_double_param");
  ParameterString<std::map<std::string, std::string> > map_string_param("map_string_param");
  parameter_handler.registerParam(map_bool_param);
  parameter_handler.registerParam(map_int_param);
  parameter_handler.registerParam(map_double_param);
  parameter_handler.registerParam(map_string_param);
  std::map<std::string, bool> map_bool = parameter_handler.getParam(map_bool_param);
  std::map<std::string, int> map_int = parameter_handler.getParam(map_int_param);
  std::map<std::string, double> map_double = parameter_handler.getParam(map_double_param);
  std::map<std::string, std::string> map_string = parameter_handler.getParam(map_string_param);
  EXPECT_EQ(map_bool["first"], false);
  EXPECT_EQ(map_bool["second"], true);
  EXPECT_EQ(map_int["a"], 31);
  EXPECT_EQ(map_int["b"], -5);
  EXPECT_EQ(map_double["pi"], 3.1);
  EXPECT_EQ(map_double["e"], 2.6);
  EXPECT_EQ(map_string["name"], "strange");
  EXPECT_EQ(map_string["description"], "string test");


  parameter_handler.changeNamespace("parking_mode");
  map_bool = parameter_handler.getParam(map_bool_param);
  map_int = parameter_handler.getParam(map_int_param);
  map_double = parameter_handler.getParam(map_double_param);
  map_string = parameter_handler.getParam(map_string_param);
  EXPECT_EQ(map_bool["first"], true);
  EXPECT_EQ(map_bool.size(), 1);
  EXPECT_EQ(map_int["a"], 15);
  EXPECT_EQ(map_int.size(), 1);
  EXPECT_EQ(map_double["pi"], 4.0);
  EXPECT_EQ(map_double.size(), 1);
  EXPECT_EQ(map_string["name"], "parking string");
  EXPECT_EQ(map_string.size(), 1);
}

TEST(TestParameterHandler, testMultipleParameterRegistations) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> bool_param("bool_test_param");
  ParameterString<int> int_param("bool_test_param"); //This is not a mistake, thats what the test is about

  parameter_handler.registerParam(bool_param);
  EXPECT_NO_THROW(parameter_handler.registerParam(bool_param));
  EXPECT_ANY_THROW(parameter_handler.registerParam(int_param));
}

TEST(TestParameterHandler, testDynamicReconfigureIntegration) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> bool_param("bool_test_param");
  ParameterString<int> int_param("int_test_param");
  ParameterString<double> double_param("double_test_param");

  EXPECT_NO_THROW(parameter_handler.registerParam(bool_param););
  EXPECT_NO_THROW(parameter_handler.registerParam(int_param);
                  parameter_handler.registerParam(double_param););

  testing::internal::CaptureStderr();
  parameter_handler.addDynamicReconfigureServer<common::ParameterHandlerTestConfig>(nh);
  const std::string output = testing::internal::GetCapturedStderr();
  //not registered dynamic reconfigure parameter triggers error
  EXPECT_NE(output.find("Dynamic reconfigure uses not registered parameter"), std::string::npos);

  EXPECT_NO_FATAL_FAILURE(parameter_handler.changeNamespace("free_ride_mode"););
}

TEST(TestParameterHandler, testParameterNotRegistered) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> bool_param("bool_test_param");
  EXPECT_THROW(parameter_handler.getParam(bool_param), ros::InvalidParameterException);
}

TEST(TestParameterHandler, testParameterNotFound) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<bool> not_found_param("not_found_test_param");
  EXPECT_THROW(parameter_handler.registerParam(not_found_param), ros::InvalidParameterException);

  ParameterString<int> bool_param("to_be_deleted");
  parameter_handler.registerParam(bool_param);

  nh.deleteParam(bool_param);
  EXPECT_THROW(parameter_handler.getParam(bool_param), ros::InvalidParameterException);
}

TEST(TestParameterHandler, testWrongParameterType) {
  ros::NodeHandle nh("~");
  ParameterHandler parameter_handler(nh);

  ParameterString<int> bool_param("bool_test_param");
  EXPECT_THROW(parameter_handler.registerParam(bool_param), ros::InvalidParameterException);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "parameter_handler_test");
  // keep one instance of NodeHandle alive to make ROS_ERROR etc work.
  ros::NodeHandle UNUSED nh;
  return RUN_ALL_TESTS();
}
