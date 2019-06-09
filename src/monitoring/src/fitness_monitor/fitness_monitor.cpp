#include "fitness_monitor.h"

FitnessMonitor::FitnessMonitor(ParameterInterface* parameters, FitnessCalculator* calculator)
    : parameters_ptr_(parameters), fitness_calculator_ptr_(calculator) {}

void FitnessMonitor::publishCurrentFitness() {
  ROS_INFO("Round completed:");
  ROS_INFO(
      "Fitness: duration: %f s, distance: %f m, average velocity: %f "
      "m/s, max: %f m/s, min: %f m/s",
      fitness_calculator_ptr_->getRoundDuration().toSec(),
      fitness_calculator_ptr_->getDrivenDistance(),
      fitness_calculator_ptr_->getAverageVelocity(),
      fitness_calculator_ptr_->getMaxVelocity(),
      fitness_calculator_ptr_->getMinVelocity());
  ROS_INFO(
      "Control: summarized squared lateral error: %f m^2, summarized absolute "
      "lateral control error: %f m",
      fitness_calculator_ptr_->getSquaredLateralControlError(),
      fitness_calculator_ptr_->getAbsLateralControlError());
}

void FitnessMonitor::resetCurrentFitness() { fitness_calculator_ptr_->reset(); }
