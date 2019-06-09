#ifndef SERVO_CHARACTERISTIC_H
#define SERVO_CHARACTERISTIC_H

#include "common/parameter_interface.h"
#include <common/polynomial.h>
#include "car_specs.h"
#include <queue>

class ServoCharacteristic {
 public:
  ServoCharacteristic(ParameterInterface& params,
                      const ParameterString<double>& const_coeff_name,
                      const ParameterString<double>& lin_coeff_name,
                      const ParameterString<double>& quadratic_coeff_name,
                      const ParameterString<double>& cubic_coeff_name,
                      const ParameterString<double>& time_constant,
                      const ParameterString<double>& dead_time,
                      double update_frequency);

  double setValueToAngle(double servo_set_value) const;

  double angleToSetValue(double angle) const;

  void updateParams(double update_frequency);

  /*!
   * \brief Takes the target command and stores it in a FIFO queue. Outputs the
   * oldest value or in case the queue isn't full the current value as passed by
   * its second argument.
   */
  double delay(double servo_set_value, double current_angle);

  /*!
   * \brief Converts the target command to next
   * value according to PTD behaviour.
   */
  double usePT1(double servo_set_value);

  /*!
   * \brief Converts the target command to the next
   * value according to PTD behaviour. Use this function in case the current
   * angle of the servo can be measured or estimated.
   */
  double usePT1(double servo_set_value, double current_angle);

 private:
  /*!
   * \brief Contains the PTD values and the latest steering values
   */
  struct PT1Tt {
    double discrete_time_constant = 1.0;  // t = 1/[T*f + 1]
                                          // T = timeconstant of the Pt1,
                                          // f = sampling frequenz = 1/dt

    int discrete_dead_time = 0;  // length of the queue
                                 // discrete dead time =
                                 // (int)(update_frequency * dead_time) + 1;

    // stores the last values simulating the death time
    std::queue<double> lastValues;
    double next_angle;  // stores the "predicted is value"
    bool next_angle_initialized = false;
  } pt1Tt;

  bool steerable = true;

  double const_coeff = 0.0;
  double lin_coeff = 0.0;
  double quadratic_coeff = 0.0;
  double cubic_coeff = 0.0;

  common::CubicPolynomial angle_to_set_value_characteristic;

  ParameterInterface& params;
  CarSpecs car_specs;
  const ParameterString<double> const_coeff_name;
  const ParameterString<double> lin_coeff_name;
  const ParameterString<double> quadratic_coeff_name;
  const ParameterString<double> cubic_coeff_name;
  const ParameterString<double> time_constant;
  const ParameterString<double> dead_time;

  bool isPolynomialInvertible() const;
};

#endif  // SERVO_CHARACTERISTIC_H
