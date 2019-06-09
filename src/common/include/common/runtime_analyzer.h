#ifndef RUNTIME_ANALYZER_H
#define RUNTIME_ANALYZER_H

#include <string>
#include <vector>

#include <boost/chrono.hpp>

#include "common/parameter_interface.h"

class RuntimeAnalyzer {
 public:
  RuntimeAnalyzer(std::string analyzer_name,
                  const unsigned int n_checkpoints,
                  const unsigned int n_max_measurements,
                  ParameterInterface* parameter_interface);

  bool registerPeriodName(std::string checkpoint_name);

  void startMeasuringIfEnabled();
  void reset();

  void measureAndPushCheckPointIfActive();

 private:
  bool getIsEnabled(ParameterInterface* parameter_interface);
  void dumpMeasuresToFile();

  const std::string analyzer_name_;
  const unsigned int n_checkpoints_;
  const unsigned int n_max_measurements_;
  const bool is_enabled;

  bool is_active_;
  size_t checkpoint_counter_;
  size_t measure_counter_;
  size_t n_registered_periods = 0;

  std::vector<std::string> period_names_;
  std::vector<boost::chrono::steady_clock::time_point> last_checkpoint_measure_;
  std::vector<std::vector<boost::chrono::microseconds> > period_measures_;
};

#endif  // RUNTIME_ANALYZER_H
