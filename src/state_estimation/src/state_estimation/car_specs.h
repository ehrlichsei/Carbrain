#ifndef CAR_SPECS_H
#define CAR_SPECS_H

#include <cmath>

#include "common/parameter_interface.h"

class CarSpecs {
 public:
  CarSpecs(ParameterInterface &parameter_interface);

  /*!
  * \brief the tire radius of the vehicle
  */
  double tire_radius = 0.05;

  /*!
  * \brief the track of the vehicle
  */
  double track = 0.2;

  /*!
  * \brief the wheelbase of the vehicle
  */
  double wheelbase = 0.25;

  /*!
   * \brief max_steering_angle_left
   */
  double max_steering_angle_left = M_PI / 2.0;

  /*!
   * \brief max_steering_angle_right
   */
  double max_steering_angle_right = M_PI / 2.0;

  /*!
   * \brief distance_cog_front
   */
  double distance_cog_front = 0.14;
  /*!
   * \brief distance_cog_rear
   */
  double distance_cog_rear = 0.11;

  /*!
   * \brief updateParams
   * \param parameter_interface the parameter interface
   */
  void updateParams(const ParameterInterface &parameter_interface);

 private:
  static const ParameterString<double> TIRE_RADIUS;
  static const ParameterString<double> TRACK;
  static const ParameterString<double> WHEELBASE;
  static const ParameterString<double> MAX_STEERING_ANGLE_LEFT;
  static const ParameterString<double> MAX_STEERING_ANGLE_RIGHT;
  static const ParameterString<double> DISTANCE_COG_FRONT;
  static const ParameterString<double> DISTANCE_COG_BACK;


  void registerParams(ParameterInterface &parameter_interface) const;

  static bool parameters_registered;
};

#endif  // CAR_SPECS_H
