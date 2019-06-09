#include "common/test/dummy_parameter_handler.h"
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <math.h>
#include <gtest/gtest.h>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

TEST(DummyParameterHandlerTest, basic) {
  const ParameterString<bool> bool_param("bool_param");
  const bool bool_param_value = true;
  const ParameterString<int> int_param("int_param");
  const int int_param_value = 42;
  const ParameterString<double> double_param("double_param");
  const double double_param_value = M_PI;
  const ParameterString<std::string> string_param("string_param");
  const std::string string_param_value = "test string";

  DummyParameterHandler parameter_handler;
  parameter_handler.addParam(bool_param, bool_param_value);
  parameter_handler.addParam(int_param, int_param_value);
  parameter_handler.addParam(double_param, double_param_value);
  parameter_handler.addParam(string_param, string_param_value);

  ParameterInterface * const interface = &parameter_handler;

  EXPECT_NO_THROW(interface->registerParam(bool_param));
  EXPECT_NO_THROW(interface->registerParam(int_param));
  EXPECT_NO_THROW(interface->registerParam(double_param));
  EXPECT_NO_THROW(interface->registerParam(string_param));

  ASSERT_ANY_THROW(interface->registerParam(ParameterString<bool>("junk_param")));

  EXPECT_EQ(interface->getParam(bool_param), bool_param_value);
  EXPECT_EQ(interface->getParam(int_param), int_param_value);
  EXPECT_EQ(interface->getParam(double_param), double_param_value);
  EXPECT_EQ(interface->getParam(string_param), string_param_value);

  ASSERT_ANY_THROW(interface->getParam(ParameterString<bool>("junk_param")));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
