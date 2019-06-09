#include "servo_characteristic.h"
#include <common/macros.h>
#include "common/math.h"

THIRD_PARTY_HEADERS_BEGIN
#include <complex>
#include "ros/ros.h"
THIRD_PARTY_HEADERS_END

using common::sgn;
using common::squared;

ServoCharacteristic::ServoCharacteristic(ParameterInterface &params,
                                         const ParameterString<double> &const_coeff_name,
                                         const ParameterString<double> &lin_coeff_name,
                                         const ParameterString<double> &quadratic_coeff_name,
                                         const ParameterString<double> &cubic_coeff_name,
                                         const ParameterString<double> &time_constant,
                                         const ParameterString<double> &dead_time,
                                         double update_frequency)
    : angle_to_set_value_characteristic(0.5),
      params(params),
      car_specs(params),
      const_coeff_name(const_coeff_name),
      lin_coeff_name(lin_coeff_name),
      quadratic_coeff_name(quadratic_coeff_name),
      cubic_coeff_name(cubic_coeff_name),
      time_constant(time_constant),
      dead_time(dead_time) {
  try {
    params.registerParam(const_coeff_name);
    params.registerParam(lin_coeff_name);
    params.registerParam(quadratic_coeff_name);
    params.registerParam(cubic_coeff_name);
    params.registerParam(time_constant);
    params.registerParam(dead_time);

    updateParams(update_frequency);
  } catch (const ros::InvalidParameterException &e) {
    steerable = false;
    ROS_WARN("using not steerable servo in state estimation: %s", e.what());
  }
}

double ServoCharacteristic::setValueToAngle(double servo_set_value) const {
  if (steerable) {
    // source: https://math.vanderbilt.edu/schectex/courses/cubic/
    const double p = -quadratic_coeff / (3.0 * cubic_coeff);
    const double q = std::pow(p, 3.0) +
                     (quadratic_coeff * lin_coeff -
                      3.0 * cubic_coeff * (const_coeff - servo_set_value)) /
                         (6.0 * std::pow(cubic_coeff, 2.0));
    const double s = std::pow(lin_coeff / (3.0 * cubic_coeff) - std::pow(p, 2.0), 3.0);  // = (r-p^2)^3

    return std::cbrt(q + std::sqrt(std::pow(q, 2.0) + s)) +
           std::cbrt(q - std::sqrt(std::pow(q, 2.0) + s)) + p;
  } else {
    return 0.0;
  }
}

double ServoCharacteristic::angleToSetValue(double angle) const {
  return angle_to_set_value_characteristic.evaluate(angle);
}

void ServoCharacteristic::updateParams(double update_frequency) {
  if (!steerable) {
    return;
  }
  const_coeff = params.getParam(const_coeff_name);
  lin_coeff = params.getParam(lin_coeff_name);
  quadratic_coeff = params.getParam(quadratic_coeff_name);
  cubic_coeff = params.getParam(cubic_coeff_name);
  angle_to_set_value_characteristic =
      common::CubicPolynomial(common::CubicPolynomial::CoefficientList{
          {const_coeff, lin_coeff, quadratic_coeff, cubic_coeff}});

  if (!isPolynomialInvertible()) {
    throw std::runtime_error(
        "polynomial of ServoCharacteristic is not invertible in interval "
        "[-max_steering_angle_right, max_steering_angle_left]\npolynomial: " +
        std::to_string(cubic_coeff) + "*x^3 + " + std::to_string(quadratic_coeff) +
        "*x^2 + " + std::to_string(lin_coeff) + "*x + " + std::to_string(const_coeff) +
        "\ninterval: [" + std::to_string(-car_specs.max_steering_angle_right) +
        ", " + std::to_string(car_specs.max_steering_angle_left) + "]");
  }

  pt1Tt.discrete_time_constant =
      1.0 / (params.getParam(time_constant) * update_frequency + 1);
  int old_dead_time =
      pt1Tt.discrete_dead_time;  // discrete_dead_time is initiated with 0 !!!
  pt1Tt.discrete_dead_time =
      static_cast<int>(update_frequency * params.getParam(dead_time)) + 1;

  // in case the new dead time is shorter than the old one
  if (old_dead_time > pt1Tt.discrete_dead_time) {
    int dif = old_dead_time - pt1Tt.discrete_dead_time;
    for (int i = 0; i < dif; i++) {
      pt1Tt.lastValues.pop();  // delete dif oldest values
    }
  }  // else is handled in usePT1();
}

bool ServoCharacteristic::isPolynomialInvertible() const {
  // polynomial invertible for the interval I=[-max_steering_angle_right,
  // max_steering_angle_left] <=> polynomial is monotone function on I <=>
  // derivative has no real valued root in I
  common::QuadraticPolynomial polynomial_derivative =
      angle_to_set_value_characteristic.calculateDerivative<1>();

  if (polynomial_derivative.getCoefficientsList().at(2) == 0.0) {
    return sgn(polynomial_derivative(-car_specs.max_steering_angle_right) ==
               sgn(polynomial_derivative(car_specs.max_steering_angle_left)));
  } else {
    polynomial_derivative =
        polynomial_derivative *
        common::ConstantPolynomial(1.0 / polynomial_derivative.getCoefficientsList().at(2));
    const std::complex<double> p = polynomial_derivative.getCoefficientsList().at(1);
    const std::complex<double> q = polynomial_derivative.getCoefficientsList().at(0);

    const std::complex<double> x_1 = -p / 2.0 + std::sqrt(squared(p / 2.0) - q);
    const std::complex<double> x_2 = -p / 2.0 - std::sqrt(squared(p / 2.0) - q);

    constexpr double epsilon = 1.0e-8;
    const bool derivative_has_real_root = std::fabs(std::imag(x_1)) < epsilon;
    if (derivative_has_real_root) {
      const bool x_1_in_interval =
          -car_specs.max_steering_angle_right <= std::real(x_1) &&
          std::real(x_1) <= car_specs.max_steering_angle_left;
      const bool x_2_in_interval =
          -car_specs.max_steering_angle_right <= std::real(x_2) &&
          std::real(x_2) <= car_specs.max_steering_angle_left;
      return !x_1_in_interval && !x_2_in_interval;
    } else {
      return true;
    }
  }
}

double ServoCharacteristic::delay(double servo_set_value, double current_angle) {
  if (!steerable) {
    return current_angle;
  }
  double target_angle = setValueToAngle(servo_set_value);
  this->pt1Tt.lastValues.push(target_angle);

  if (pt1Tt.discrete_dead_time > static_cast<int>(pt1Tt.lastValues.size())) {
    return current_angle;  // no movement while queue isn't filled
  } else {                 // first value in vector is now as old as the delay
    double delayed_angle = pt1Tt.lastValues.front();  // oldest target angle
    pt1Tt.lastValues.pop();                           // delete out of queue
    return delayed_angle;
  }
}

double ServoCharacteristic::usePT1(double servo_set_value) {
  if (pt1Tt.next_angle_initialized) {
    return usePT1(servo_set_value, pt1Tt.next_angle);
  } else {
    return usePT1(servo_set_value, setValueToAngle(servo_set_value));
  }
}

double ServoCharacteristic::usePT1(double servo_set_value, double current_angle) {
  if (!steerable) {
    return servo_set_value;
  }
  double delayed_cmd = delay(servo_set_value, current_angle);
  // calculate the new "predicted is value" and save it in the class variable
  pt1Tt.next_angle = pt1Tt.discrete_time_constant * (delayed_cmd - current_angle) + current_angle;
  pt1Tt.next_angle_initialized = true;
  return pt1Tt.next_angle;
}
