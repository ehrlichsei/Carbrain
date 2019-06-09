#ifndef PARKINGLOT_ALIGNER_H
#define PARKINGLOT_ALIGNER_H

#include "common/parameter_interface.h"
#include "navigation/driving_corridor.h"
THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
THIRD_PARTY_HEADERS_END
/*!
 * \brief ParkinglotAligner
 */
class ParkinglotAligner {
 public:
  /*!
  * \brief ParkinglotAligner is the consstructor. A ros indipendent
  * functionality containing
  * class needs a pointer to a ParameterInterface (in fact a ParameterHandler)
  * to get access to parameters.
  * \param parameters the ParameterInterface
  */
  ParkinglotAligner(ParameterInterface* parameters);

  /**
  * @brief Calculate the transformation from parkinglot to world and sets if
  * necessary the pointOfOrigin.
  * @param corridor The current corridor with the gates the method needs.
  * @param carPosition The current position of the car to find out if the origin
  * is behind the car
  *        and at which gate the car is.
  * @param transformation The method writes the calculated transformation in
  * this object.
  * @return true if the transformation could be calculated and otherwise false
  */
  bool calculateTransformation(const DrivingCorridor& corridor,
                               const Eigen::Vector3d& carPosition,
                               Eigen::Affine3d& transformation);

  /**
  * @brief Sets the start point. This method has to be called before a
  * transformation can be calculated
  *        and later calls change the start point which results in a new
  * calculation of the pointOfOrigin.
  * @param point The new start point.
  */
  void setStartPoint(const Eigen::Vector3d& point);

 private:
  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface* parameters_ptr_;
  Eigen::Vector3d pointOfOrigin;
  Eigen::Vector3d startPoint;
  bool newStartPoint = false;

  Eigen::Vector3d getStartPoint();
  double getDistanceToCar();
  std::unique_ptr<Eigen::Affine3d> calculatedTransformation;
  Gate getGateNextToPoint(const DrivingCorridor& corridor, const Eigen::Vector3d& point);
};

#endif  // PARKINGLOT_ALIGNER_H
