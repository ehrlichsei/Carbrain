
#include "../src/controller_interface/command_handler.h"

THIRD_PARTY_HEADERS_BEGIN
//#include "command.pb.h"

#include <gtest/gtest.h>
#include <common/macros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/algorithm/clamp.hpp>
#include "controller_msgs/LightsCommand.h"
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

class CommandHandlerTest: public ::testing::Test {
	public:
		static const bool use_back_steering;
		static const std_msgs::ColorRGBA DEBUG_LIGHT_COLOR;
		//static kitcar::Command protobuf_command;
};

std_msgs::ColorRGBA getColorRGBAMessageFromFloats(float r, float g, float b, float a) {
	std_msgs::ColorRGBA return_val;
	return_val.r = r;
	return_val.g = g;
	return_val.b = b;
	return_val.a = a;
	return return_val;
}
const std_msgs::ColorRGBA CommandHandlerTest::DEBUG_LIGHT_COLOR = getColorRGBAMessageFromFloats(0.1, 0.98, 0.3, 0.0);


TEST_F(CommandHandlerTest, setDebugLightColorCommand) {
  CommandHandler commandHandler;
	kitcar::Command protobuf_command = kitcar::Command::default_instance();
 
	commandHandler.setDebugLightColorCommand(DEBUG_LIGHT_COLOR);
	commandHandler.createLightsCommand(&protobuf_command);	
  kitcar::Command::LightsControl lights_control = *protobuf_command.mutable_lights_control();

	uint32_t debug_light_protobuf_message = 0x4CF919;
  ASSERT_EQ(debug_light_protobuf_message, lights_control.debug_light_color());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
