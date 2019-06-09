#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

#include "state_estimation_msgs/State.h"
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"


/*!
 * \brief Self-localization via integration of state_estimation data. This class contains
 * the dead reackoning logic (Rieman integration).
 */
class Localization {
 public:
  /*!
  * \brief Localization is the consstructor. A ros indipendent functionality
  * containing class needs a pointer to a ParameterInterface (in fact a
  * ParameterHandler) to get access to parameters.
  * \param parameters the ParameterInterface
  */
  Localization(ParameterInterface *parameters);

  /*!
   * \brief reset resets the transformation from 'vehicle' to 'world' frame.
   */
  void reset();

  /*!
   * \brief update updates the transformation from 'vehicle' to 'world' frame using the
   * given mesurements.
   * \param state_msg the mesurements to update the transformation.
   * \return the updated transformation.
   */
  const tf2::Stamped<Eigen::Affine3d> &update(const state_estimation_msgs::State::ConstPtr &state_msg);

 private:
  /*!
   * \brief INITIAL_X The initial x value of the position.
   */
  static const ParameterString<double> INITIAL_X;
  /*!
   * \brief INITIAL_Y The initial y value of the position.
   */
  static const ParameterString<double> INITIAL_Y;
  /*!
   * \brief INITIAL_YAW The initial value of the orientation.
   */
  static const ParameterString<double> INITIAL_YAW;
  /*!
   * \brief MAX_TIME_BETWEEN_TWO_MESUREMENTS maximal allowed time between two mesurements.
   * If this time gets exceeded, the transformation gets reseted (calling reset()).
   */
  static const ParameterString<double> MAX_TIME_BETWEEN_TWO_MESUREMENTS;

  /*!
  * \brief parameters_ptr_ is needed for parameter access
  */
  ParameterInterface *parameters_ptr_;
  /*!
   * \brief vehicle_world_transform_ is the transformation from 'vehicle' to 'world' frame
   * which discribes the pose of the vehicle in 'world' frame.
   */
  tf2::Stamped<Eigen::Affine3d> vehicle_world_transform_;
};

#endif  // LOCALIZATION_H
