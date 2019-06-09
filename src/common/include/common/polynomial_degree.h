#ifndef KITCAR_POLYNOMIAL_DEGREE_H
#define KITCAR_POLYNOMIAL_DEGREE_H

#include <limits>
#include <cassert>

namespace common {

namespace polynomial {

typedef int PolynomialDegree;

namespace PolynomialDegrees {

enum PolynomialDegree {
  Dynamic = -2,
  MinusInfinity = -1,
  Constant = 0,
  Linear = 1,
  Quadratic = 2,
  Cubic = 3,
  Quartic = 4,
  Quintic = 5,
  Sextic = 6,
  Septic = 7,
  Maximal = std::numeric_limits<common::polynomial::PolynomialDegree>::max()
};
}  // namespace PolynomialDegrees;

inline PolynomialDegree polynomial_degree_cast(std::size_t x) {
  assert(x <= static_cast<std::size_t>(PolynomialDegrees::Maximal));
  return static_cast<PolynomialDegree>(x);
}

// forward declaration needed in other headers.
template <PolynomialDegree MAX_DEGREE, typename T = double>
class Polynomial;
}  // namespace polynomial;
using namespace polynomial;
}  // namespace common;

#endif  // KITCAR_POLYNOMIAL_DEGREE_H
