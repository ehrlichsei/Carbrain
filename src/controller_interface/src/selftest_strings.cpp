#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <boost/assign.hpp>

#include "selftests.pb.h"
THIRD_PARTY_HEADERS_END

#include "selftest_strings.h"

StringMap SelftestStrings::createNameMap() {
  StringMap names;

  names[kitcar::SelfTest_Name_TRIVIAL] = "trivial test";
  names[kitcar::SelfTest_Name_SERIAL_BUFFER] = "serial read buffer test";
  names[kitcar::SelfTest_Name_BRAIN_INTERFACE] = "brain interface test";
  names[kitcar::SelfTest_Name_WHEEL_ENCODER] = "wheel encoder test";

  return names;
}

StringMap SelftestStrings::createMessageMap() {
  StringMap messages;

  messages[kitcar::SelfTest_Message_TRIVIAL_OK] = "trivial test passed";

  messages[kitcar::SelfTest_Message_SERIAL_BUFFER_OK] =
      "less than 50 bytes in Serial read buffer";
  messages[kitcar::SelfTest_Message_SERIAL_BUFFER_WARN] =
      "Serial read buffer is likely to overflow";
  messages[kitcar::SelfTest_Message_SERIAL_BUFFER_ERROR] =
      "Serial read buffer full! Overflow very likely!";

  messages[kitcar::SelfTest_Message_BRAIN_INTERFACE_OK] =
      "no problems in decoding or encoding";
  messages[kitcar::SelfTest_Message_BRAIN_INTERFACE_WARN] =
      "some errors occurred while decoding / encoding or invalid messages "
      "recieved (see values)";

  messages[kitcar::SelfTest_Message_WHEEL_ENCODER_OK] =
      "No sign error in wheel encoders detected";
  messages[kitcar::SelfTest_Message_WHEEL_ENCODER_WARN] =
      "Sign error detected and corrected";
  messages[kitcar::SelfTest_Message_WHEEL_ENCODER_ERROR] =
      "Unknown error, check Arduino code";

  return messages;
}

StringVectorMap SelftestStrings::createKeyVectorMap() {
  StringVectorMap keys;

  keys[kitcar::SelfTest_Name_BRAIN_INTERFACE] = StringVector();
  keys[kitcar::SelfTest_Name_BRAIN_INTERFACE].emplace_back(
      "invalid crc's of incoming messages [%]");
  keys[kitcar::SelfTest_Name_BRAIN_INTERFACE].emplace_back(
      "decoding errors [%]");
  keys[kitcar::SelfTest_Name_BRAIN_INTERFACE].emplace_back(
      "encoding errors [%]");

  return keys;
}
