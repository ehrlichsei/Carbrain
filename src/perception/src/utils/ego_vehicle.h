#ifndef EGO_VEHICLE_H
#define EGO_VEHICLE_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core.hpp>
#include <array>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"

class EgoVehicle
{
public:
 EgoVehicle(ParameterInterface* parameter_interface);

 cv::_InputArray asInputArray() const;
 void update(const cv::Rect& image_limits);

 bool contains(const cv::Point& p) const;

 void update();

private:
 typedef std::array<cv::Point, 4> Trapezoid;
 /*!
  * \brief EGO_VEHICLE_BOTTOM_LEFT_LOC_X
  *
  * bottom left location (x-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_BOTTOM_LEFT_LOC_X;
 /*!
  * \brief EGO_VEHICLE_BOTTOM_LEFT_LOC_Y
  *
  * bottom left location (y-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_BOTTOM_LEFT_LOC_Y;
 /*!
  * \brief EGO_VEHICLE_TOP_LEFT_LOC_X
  *
  * top left location (x-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_TOP_LEFT_LOC_X;
 /*!
  * \brief EGO_VEHICLE_TOP_LEFT_LOC_Y
  *
  * top left location (y-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_TOP_LEFT_LOC_Y;
 /*!
  * \brief EGO_VEHICLE_TOP_RIGHT_LOC_X
  *
  * top right location (x-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_TOP_RIGHT_LOC_X;
 /*!
  * \brief EGO_VEHICLE_TOP_RIGHT_LOC_Y
  *
  * top right location (y-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_TOP_RIGHT_LOC_Y;
 /*!
  * \brief EGO_VEHICLE_BOTTOM_RIGHT_LOC_X
  *
  * bottom right location (x-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_BOTTOM_RIGHT_LOC_X;
 /*!
  * \brief EGO_VEHICLE_BOTTOM_RIGHT_LOC_Y
  *
  * bottom right location (y-axis) of the ego vehicle relative to recorded image size.
  */
 static const ParameterString<int> EGO_VEHICLE_BOTTOM_RIGHT_LOC_Y;

 const ParameterInterface* const parameter_ptr;

 Trapezoid trapezoid;
};

#endif // EGO_VEHICLE_H
