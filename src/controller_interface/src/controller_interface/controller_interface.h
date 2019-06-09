#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "measures_handler.h"
#include "command_handler.h"

#include "common/parameter_interface.h"

#include <common/realtimecontroller.h>
#include <common/realtimeipc.h>
#include <common/concurrent_queue.h>
#include <common/runtime_analyzer.h>

#include "controller_interface/logging.h"

/*!
 * \brief manages the communication with Arduino
 */
class ControllerInterface : public common::RealtimeController {
 public:
  /*!
  * \brief ControllerInterface is the consstructor. A ros indipendent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
   ControllerInterface(ParameterInterface* parameters, MeasuresHandler* measures_handler, CommandHandler* command_handler);

  /*!
   * \brief ~ControllerInterface deconstructor which terminates all threads cleanly.
   * RealtimeControllers deconstructor was applicable because the threads need to be
   * terminated befor the atrributes are getting destroyed.
   */
  virtual ~ControllerInterface() override;

  virtual void update(boost::chrono::nanoseconds) override;
  virtual void pollParameters() override;

  /*!
   * gets logging message from realtime thread using a queue to obtain realtime features
   * is blocking until a message appears or thread gets interrupted.
   */
  bool getLoggingMessage(LoggingMessage* message);

  //! contains last received protobuf version on controller
  uint32_t controllers_protobuf_version_ = 0;

 private:

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface *parameters_ptr_;
  bool initSerialPort();
  void receiveMeasure();
  void readMeasure(const boost::system::error_code& e, const std::size_t n_bytes_transferred);

  bool serialPortIsOpen() const;
  void requestMeasures(const bool with_proto_msg_version);
  void checkProtobufMessageVersion(const kitcar::Measure& protobuf_measure);
  void transmitCommand(const kitcar::Command& protobuf_command);
  void onTimeout(const boost::system::error_code& error);

  void pushRuntimeMeasures();
  void dumpRuntimeMeasuresToFile();

  static const ParameterString<int> BAUD_RATE;
  static const ParameterString<std::string> SERIAL_PORT;
  static const ParameterString<int> SERIAL_BUFFER_SIZE;
  static const ParameterString<int> SERIAL_READ_TIMEOUT;
  static const ParameterString<int> ARDUINO_BOOT_WAIT;
  static const ParameterString<bool> WRITE_TO_SHARED_MEMORY;

  std::string serial_port_param;
  int baud_rate;
  int controller_buffer_size;
  int serial_read_timeout;
  int arduino_boot_wait;
  bool synchronized_ = false;
  bool msg_version_received_ = false;

  boost::asio::io_service io_service;
  boost::asio::serial_port serial_port;
  boost::asio::deadline_timer read_timeout_timer;
  boost::asio::streambuf response_buffer;

  MeasuresHandler* measures_handler_;
  CommandHandler* command_handler_;

  LoggingQueue log_;
  RuntimeAnalyzer runtime_analyzer_;
  unsigned int request_id_ = 0;
};

#endif  // CONTROLLER_INTERFACE_H
