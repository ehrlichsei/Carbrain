#include "controller_interface.h"

#include <ros/time.h>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/filesystem/operations.hpp>

#include "common/realtime_timings.h"

#include "measure.pb.h"
#include "command.pb.h"
#include "protobuf_msg_version.h"
#include "../../protobuf/kitcar-protobufs/BoardToBoardInterface/CobsPackage.h"

#define NUM_RUNTIME_MEASURES 2000

const ParameterString<int> ControllerInterface::BAUD_RATE("baud_rate");
const ParameterString<std::string> ControllerInterface::SERIAL_PORT(
    "serial_port");
const ParameterString<int> ControllerInterface::SERIAL_BUFFER_SIZE(
    "serial_buffer_size");
const ParameterString<int> ControllerInterface::SERIAL_READ_TIMEOUT(
    "serial_read_timeout");
const ParameterString<int> ControllerInterface::ARDUINO_BOOT_WAIT(
    "arduino_boot_wait");
const ParameterString<bool> ControllerInterface::WRITE_TO_SHARED_MEMORY(
    "write_to_shared_memory");

ControllerInterface::ControllerInterface(ParameterInterface *parameters,
                                         MeasuresHandler *measures_handler,
                                         CommandHandler *command_handler)
    : RealtimeController(RealtimeTimings::LOOP_RATE,
                         RealtimeTimings::CONTROLLER_INTERFACE_OFFSET,
                         parameters),
      parameters_ptr_(parameters),
      serial_port(io_service),
      read_timeout_timer(io_service),
      measures_handler_(measures_handler),
      command_handler_(command_handler),
      runtime_analyzer_("controller_interface", 3, NUM_RUNTIME_MEASURES, parameters) {

  runtime_analyzer_.registerPeriodName("startloop");
  runtime_analyzer_.registerPeriodName("measure_received");
  runtime_analyzer_.registerPeriodName("endloop");
  //  runtime_analyzer_.registerPeriodName("measure_decoded");
  //  runtime_analyzer_.registerPeriodName("command_sent");*/
  runtime_analyzer_.startMeasuringIfEnabled();

  parameters_ptr_->registerParam(BAUD_RATE);
  parameters_ptr_->registerParam(SERIAL_PORT);
  parameters_ptr_->registerParam(SERIAL_BUFFER_SIZE);
  parameters_ptr_->registerParam(SERIAL_READ_TIMEOUT);
  parameters_ptr_->registerParam(ARDUINO_BOOT_WAIT);
  parameters_ptr_->registerParam(WRITE_TO_SHARED_MEMORY);

  baud_rate = parameters_ptr_->getParam(BAUD_RATE);
  serial_port_param = parameters_ptr_->getParam(SERIAL_PORT);
  controller_buffer_size = parameters_ptr_->getParam(SERIAL_BUFFER_SIZE);
  serial_read_timeout = parameters_ptr_->getParam(SERIAL_READ_TIMEOUT);
  arduino_boot_wait = parameters_ptr_->getParam(ARDUINO_BOOT_WAIT);
  measures_handler_->setWriteToSharedMemory(parameters_ptr_->getParam(WRITE_TO_SHARED_MEMORY));
  measures_handler->setLog(&log_);
}

// In general it is a very good practice to have not-throwing destructors.
// In this special case it does not really matter though, because this
// destructor should only be called during shutdown. If boost::thread throws
// an exception the operating system will take care of the remains and clean
// them up.
// NOLINTNEXTLINE(bugprone-exception-escape)
ControllerInterface::~ControllerInterface() {
  control_thread_.interrupt();
  parameter_poll_thread_.interrupt();
  control_thread_.join();
  parameter_poll_thread_.join();
}

void ControllerInterface::update(boost::chrono::nanoseconds) {
  if (!serialPortIsOpen()) {
    synchronized_ = false;
    msg_version_received_ = false;

    if (initSerialPort()) {
      // time for arduino to reboot
      boost::this_thread::sleep_for(boost::chrono::microseconds(arduino_boot_wait));
      log_.push(SERIAL_PORT_OPENED);
    } else {
      log_.push(FAILED_TO_OPEN_SERIAL_PORT);
    }
    return;
  }
  runtime_analyzer_.measureAndPushCheckPointIfActive();

  const ros::WallTime start_measure = ros::WallTime::now();
  // poll measures (eventually with message version)
  requestMeasures(!msg_version_received_);

  try {
    do {
      receiveMeasure();
    } while (msg_version_received_ && !synchronized_);
  } catch (const boost::system::system_error &) {
    log_.push(ASIO_READ_EXCEPTION);
    serial_port.close();
  }
  const ros::WallTime end_measure = ros::WallTime::now();
  const int measure_duration = (end_measure - start_measure).toNSec() / 1000;
  if (measure_duration > RealtimeTimings::CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME) {
    ROS_WARN_THROTTLE(1, "measurement takes to long : %dus!", measure_duration);
  }
  runtime_analyzer_.measureAndPushCheckPointIfActive();

  if (!serialPortIsOpen()) {
    return;
  }

  // sleep to leave the state estimation and controllers time to finish
  const boost::chrono::microseconds time_to_sleep =
      boost::chrono::microseconds(RealtimeTimings::CONTROLLER_INTERFACE_MEASUREMENT_CYCLE_TIME +
                                  RealtimeTimings::CONTROLLER_INTERFACE_COMMAND_WAIT_TIME) -
      getTimeSliceMicroseconds();
  if (time_to_sleep > boost::chrono::microseconds(0L)) {
    boost::this_thread::sleep_for(time_to_sleep);
  } else {
    ROS_WARN_THROTTLE(
        1,
        "RealtimeTimings::CONTROLLER_INTERFACE_COMMAND_WAIT_TIME is to "
        "small! time_to_sleep: %lds!",
        time_to_sleep.count());
  }

  // read control values and send them
  kitcar::Command protobuf_command = kitcar::Command::default_instance();
  protobuf_command.set_request_measures(false);
  command_handler_->createSteeringCommand(&protobuf_command);
  command_handler_->createSpeedCommand(&protobuf_command);
  command_handler_->createLightsCommand(&protobuf_command);
  command_handler_->createMissionMode(&protobuf_command);
  transmitCommand(protobuf_command);

  runtime_analyzer_.measureAndPushCheckPointIfActive();
}

void ControllerInterface::onTimeout(const boost::system::error_code &error) {
  // error occures if timer was aborted
  if (!error) {
    // timer expired
    serial_port.cancel();
    log_.push(TIMEOUT);
  }
}

void ControllerInterface::pollParameters() {
  //! @TODO here goes the parameter updating.
}

bool ControllerInterface::initSerialPort() {
  // check if given serial port exists
  if (!boost::filesystem::exists(serial_port_param.c_str())) {
    log_.push(SERIAL_PORT_DOES_NOT_EXIST);
    return false;
  }

  try {
    serial_port.open(serial_port_param);
    boost::asio::serial_port_base::baud_rate baud_option(baud_rate);
    boost::asio::serial_port_base::stop_bits stop_option(
        boost::asio::serial_port_base::stop_bits::one);
    boost::asio::serial_port_base::character_size char_option(8);
    serial_port.set_option(baud_option);
    serial_port.set_option(stop_option);
    serial_port.set_option(char_option);
    return true;
  } catch (const boost::system::system_error &) {
    return false;
  }
}

using iterator = boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type>;

std::pair<iterator, bool> match_whitespace(iterator begin, iterator end) {
  const iterator i = std::find(begin, end, '\0');
  return i != end ? std::make_pair(std::next(i), true) : std::make_pair(end, false);
}

void ControllerInterface::receiveMeasure() {
  // wait for and read measures
  boost::asio::async_read_until(
      serial_port,
      response_buffer,
      match_whitespace,
      boost::bind(&ControllerInterface::readMeasure, this, _1, _2));

  // time for arduino to get the measure data from sensors
  read_timeout_timer.expires_from_now(boost::posix_time::microseconds(serial_read_timeout));
  read_timeout_timer.async_wait(boost::bind(
      &ControllerInterface::onTimeout, this, boost::asio::placeholders::error));
  // wait until message is recived or timeout is exceeded

  // reset io_serivice internal state in preperation of run()
  io_service.reset();
  // blocking until timeout or msg received
  io_service.run();
}

void ControllerInterface::readMeasure(const boost::system::error_code &e,
                                      const std::size_t n_bytes_transferred) {
  if (e != std::nullptr_t()) {
    serial_port.close();
    return;
  }
  read_timeout_timer.cancel();

  const unsigned char *response_ptr =
      boost::asio::buffer_cast<const unsigned char *>(response_buffer.data());

  std::array<unsigned char, 512> raw_data = {};
  unsigned short raw_data_length = raw_data.size();
  const bool cobs_decode_error =
      !cobs::decode(response_ptr, n_bytes_transferred, raw_data.data(), raw_data_length);
  response_buffer.consume(n_bytes_transferred);

  if (cobs_decode_error) {
    log_.push(COBS_DECODING_ERROR);
    return;
  }

  google::protobuf::io::ArrayInputStream data_input_stream(raw_data.data(), raw_data_length);
  kitcar::Measure protobuf_measure;
  const bool protobuf_parsing_error =
      !protobuf_measure.ParseFromZeroCopyStream(&data_input_stream);

  if (protobuf_parsing_error) {
    log_.push(PROTOBUF_PARSING_FAILED);
    return;
  }

  checkProtobufMessageVersion(protobuf_measure);
  const bool last_synchronized = synchronized_;
  synchronized_ = (protobuf_measure.response_id() == request_id_);
  if (!synchronized_) {
    log_.push(WRONG_RESPONSE_ID);
    return;
  }

  log_.push(VALID_PROTOBUF_MESSAGE_PARSED);
  const ros::Time stamp = ros::Time().fromNSec(ros::WallTime::now().toNSec());
  measures_handler_->handleReceivedMeasures(protobuf_measure, stamp);

  if (!last_synchronized) {
    log_.push(MEASURE_SYNCHRONIZED_AGAIN);
  }
}

bool ControllerInterface::serialPortIsOpen() const {
  return serial_port.is_open();
}

void ControllerInterface::requestMeasures(const bool with_proto_msg_version) {
  // modulo 128 to avoid the integer to need more than 7bit,
  // so it can be transfered in 1 Byte with protobuf
  request_id_ = (request_id_ + 1) % 128;

  kitcar::Command version_command = kitcar::Command::default_instance();
  command_handler_->createRequestCommand(with_proto_msg_version, request_id_, &version_command);
  transmitCommand(version_command);
}

void ControllerInterface::checkProtobufMessageVersion(const kitcar::Measure &protobuf_measure) {
  if (protobuf_measure.has_msg_version()) {
    msg_version_received_ = true;
    uint32_t version_response = protobuf_measure.msg_version();
    if (version_response == PROTOBUF_MSG_VERSION) {
      log_.push(MSG_VERSION_MATCH);
    } else {
      controllers_protobuf_version_ = version_response;
      log_.push(WRONG_MSG_VERSION);
    }
  }
  if (!msg_version_received_) {
    log_.push(NO_MSG_VERSION);
  }
}

void ControllerInterface::transmitCommand(const kitcar::Command &protobuf_command) {
  constexpr unsigned int protobuf_buffer_size = 512;
  std::array<unsigned char, protobuf_buffer_size> data = {};

  // after the encoded data, the crc byte is place. Therefore we can use on byte
  // less here.
  google::protobuf::io::ArrayOutputStream data_output_stream(data.data(), data.size() - 1);
  protobuf_command.SerializeToZeroCopyStream(&data_output_stream);

  std::array<unsigned char, protobuf_buffer_size + 2> encoded_data = {};
  unsigned short encoded_length = 0;
  cobs::encode(data.data(), data_output_stream.ByteCount(), encoded_data.data(), encoded_length);

  if (encoded_length > controller_buffer_size || !serial_port.is_open()) {
    return;
  }

  try {
    boost::asio::write(serial_port, boost::asio::buffer(&encoded_length, 1));
    boost::asio::write(serial_port, boost::asio::buffer(encoded_data.data(), encoded_length));
    tcflush(serial_port.lowest_layer().native_handle(), TCIOFLUSH);
  } catch (const boost::system::system_error &) {
    log_.push(ASIO_WRITE_EXCEPTION);
  }
}


bool ControllerInterface::getLoggingMessage(LoggingMessage *message) {
  return log_.pop(message);
}
