#include "foot_finder.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <cmath>
THIRD_PARTY_HEADERS_END

#include "common/math.h"

namespace utils {

double findLotfusspunktX(const common::DynamicPolynomial &polynomial,
                         const Eigen::Vector2d &point,
                         const double epsilon,
                         const unsigned int max_iterations) {
  using namespace common;
  const auto distance_polynomial = DynamicPolynomial(squared(1.0_x - point.x())) +
                                   squared(polynomial - point.y());
  return findRoot(distance_polynomial.derivate(), epsilon, max_iterations, point.x());
}

double findRoot(const common::DynamicPolynomial &polynom,
                const double epsilon,
                const unsigned int max_iterations,
                const double x_start) {
  /*!
   * Newton Verfahren wird an der Ableitung unserer Distanz Funktion
   * durchgeführt um NULLSTELLE zu finden!
   * Die Herleitung der Gleichungen und Infos zum Newton Verfahren gibts in der
   * Wiki
   */

  double distance = std::numeric_limits<double>::max();
  double previous_distance = 0.0;
  double x_guess = x_start;
  common::DynamicPolynomial first_derivate = polynom.derivate();

  unsigned int i_newton = 0;
  while (std::abs(previous_distance - distance) > epsilon && (i_newton < max_iterations)) {
    i_newton++;                    // Iterator
    previous_distance = distance;  // Merken der letzten Distanz

    distance = polynom.evaluate(x_guess);
    double distance_d = first_derivate.evaluate(x_guess);
    // Folgen der Tangente bis zur nächsten Nullstelle. xguess wird unser
    // nächster Auswertepunkt werden.
    x_guess = x_guess - (distance / distance_d);
  }

  if (i_newton >= max_iterations) {
    ROS_WARN("Newton-Verfahren nach der 20ten Iteration abgebrochen ");
  }

  return x_guess;
}

}  // namespace utils
