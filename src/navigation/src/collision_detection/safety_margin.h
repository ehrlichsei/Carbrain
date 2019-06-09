#ifndef SAFETY_MARGIN_H
#define SAFETY_MARGIN_H
#include <common/macros.h>

#include "common/parameter_interface.h"
#include "navigation/gate.h"
#include "navigation/driving_corridor.h"
#include "vehicle.h"

THIRD_PARTY_HEADERS_BEGIN
#include "nav_msgs/Path.h"
THIRD_PARTY_HEADERS_END

/*!
 * \brief detects obstacles that collide with lane points
 */
class SafetyMargin {
 public:
  /*!
  * \brief SafetyMargin is the consstructor. A ros indipendent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  SafetyMargin(ParameterInterface* parameters);

  void applySafetyMargin(const DrivingCorridor &safe_corridor, DrivingCorridor& car_corridor);


 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* parameters_ptr_;

  Vehicle buildVehicleWithParams();
  void generateWarningsShrinkDistance(double largest_violation_left_side,
                                      double largest_violation_right_side,
                                      double gateWidth,
                                      double large_shrink_distance) const;
};

#endif  // SAFETY_MARGIN_H
