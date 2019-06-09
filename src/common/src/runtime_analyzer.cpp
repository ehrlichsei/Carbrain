#include "common/runtime_analyzer.h"

#include <iostream>
#include <fstream>

RuntimeAnalyzer::RuntimeAnalyzer(std::string analyzer_name,
                                 const unsigned int n_checkpoints,
                                 const unsigned int n_max_measurements,
                                 ParameterInterface* parameter_interface)
    : analyzer_name_(std::move(analyzer_name)),
      n_checkpoints_(n_checkpoints),
      n_max_measurements_(n_max_measurements),
      is_enabled(getIsEnabled(parameter_interface)),
      is_active_(false),
      checkpoint_counter_(0),
      measure_counter_(0) {

  last_checkpoint_measure_.resize(n_checkpoints);
  period_measures_.resize(n_checkpoints);
  period_names_.resize(n_checkpoints);

  for (auto& period_measure : period_measures_) {
    period_measure.resize(n_max_measurements_);
  }
}

bool RuntimeAnalyzer::registerPeriodName(std::string checkpoint_name) {
  if (n_registered_periods >= period_names_.size()) {
    throw std::runtime_error(
        "you have to register exactly as many periods as checkpoints!");
  }

  period_names_[n_registered_periods++] = std::move(checkpoint_name);

  return (n_registered_periods == period_names_.size());
}

void RuntimeAnalyzer::startMeasuringIfEnabled() { is_active_ = is_enabled; }

void RuntimeAnalyzer::reset() {
  last_checkpoint_measure_.clear();
  period_measures_.clear();

  last_checkpoint_measure_.resize(n_checkpoints_);
  period_measures_.resize(n_checkpoints_);

  for (auto& period_measure : period_measures_) {
    period_measure.resize(n_max_measurements_);
  }

  checkpoint_counter_ = 0;
  measure_counter_ = 0;
}

void RuntimeAnalyzer::measureAndPushCheckPointIfActive() {
  size_t period_idx = checkpoint_counter_;

  if (!is_active_ || measure_counter_ >= n_max_measurements_) {
    return;
  }

  last_checkpoint_measure_[checkpoint_counter_] = boost::chrono::steady_clock::now();

  if (measure_counter_ > 0) {
    period_measures_[period_idx][measure_counter_] =
        boost::chrono::duration_cast<boost::chrono::microseconds>(
            last_checkpoint_measure_[checkpoint_counter_] -
            last_checkpoint_measure_[(checkpoint_counter_ + n_checkpoints_ - 1) % n_checkpoints_]);
  }

  checkpoint_counter_++;
  if (checkpoint_counter_ == n_checkpoints_) {
    checkpoint_counter_ = 0;
    measure_counter_++;
  }
  if (measure_counter_ == n_max_measurements_) {
    dumpMeasuresToFile();
    is_active_ = false;
  }
}

bool RuntimeAnalyzer::getIsEnabled(ParameterInterface* parameter_interface) {
  const ParameterString<bool> ENABLED("enable_runtime_analyzer");
  parameter_interface->registerParam(ENABLED);
  const bool enabled = parameter_interface->getParam(ENABLED);
  if (enabled) {
    std::cout << "\031[0mWARNING! runtime_analyzer of " << analyzer_name_
              << " is active!\033[0m" << std::endl;
  }
  return enabled;
}

void RuntimeAnalyzer::dumpMeasuresToFile() {
  std::cout << "\031[0mWARNING! dumping runtime measures of " << analyzer_name_
            << ". Realtime capabilities may suffer!\033[0m" << std::endl;

  std::stringstream file_name_stream;
  file_name_stream << "/tmp/runtimes_" << analyzer_name_ << ".txt";

  std::ofstream runtime_stats_file;
  runtime_stats_file.open(file_name_stream.str().c_str(), std::ofstream::out);

  if (runtime_stats_file.is_open()) {
    if (period_names_.size() == last_checkpoint_measure_.size()) {
      runtime_stats_file << "# ";

      for (auto& period_name : period_names_) {
        runtime_stats_file << period_name << " ";
      }
      runtime_stats_file << "\n";
    }

    for (size_t measure_idx = 1; measure_idx < n_max_measurements_; measure_idx++) {
      for (size_t period_idx = 0; period_idx < period_names_.size(); period_idx++) {
        runtime_stats_file << period_measures_[period_idx][measure_idx].count() << " ";
      }
      runtime_stats_file << "\n";
    }

    runtime_stats_file.close();
  }
}
