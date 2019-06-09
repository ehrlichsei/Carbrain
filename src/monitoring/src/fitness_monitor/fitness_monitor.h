#ifndef FITNESS_MONITOR_H
#define FITNESS_MONITOR_H
#include <common/macros.h>

#include "common/parameter_interface.h"

#include "fitness_calculator.h"

/*!
 * \brief monitores min/max/avg. velocity etc.
 */
class FitnessMonitor {
 public:
  /*!
  * \brief FitnessMonitor is the consstructor. A ros indipendent functionality
  * containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  FitnessMonitor(ParameterInterface* parameters, FitnessCalculator* calculator);

  /*!
   * \brief publishes current fitness values and resets calculation
   */
  void publishCurrentFitness();

  /*!
   * \brief resets current fitness values and resets calculation
   */
  void resetCurrentFitness();

 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* UNUSED parameters_ptr_;
  FitnessCalculator* fitness_calculator_ptr_;
};

#endif  // FITNESS_MONITOR_H
