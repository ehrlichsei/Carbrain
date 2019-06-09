#include "ego_vehicle.h"
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/imgproc/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "opencv_utils.h"

const ParameterString<int> EgoVehicle::EGO_VEHICLE_BOTTOM_LEFT_LOC_X(
    "/perception/ego_vehicle/trapezoid_mask/bottom_left_loc_x");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_BOTTOM_LEFT_LOC_Y(
    "/perception/ego_vehicle/trapezoid_mask/bottom_left_loc_y");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_TOP_LEFT_LOC_X(
    "/perception/ego_vehicle/trapezoid_mask/top_left_loc_x");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_TOP_LEFT_LOC_Y(
    "/perception/ego_vehicle/trapezoid_mask/top_left_loc_y");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_TOP_RIGHT_LOC_X(
    "/perception/ego_vehicle/trapezoid_mask/top_right_loc_x");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_TOP_RIGHT_LOC_Y(
    "/perception/ego_vehicle/trapezoid_mask/top_right_loc_y");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_BOTTOM_RIGHT_LOC_X(
    "/perception/ego_vehicle/trapezoid_mask/bottom_right_loc_x");
const ParameterString<int> EgoVehicle::EGO_VEHICLE_BOTTOM_RIGHT_LOC_Y(
    "/perception/ego_vehicle/trapezoid_mask/bottom_right_loc_y");

EgoVehicle::EgoVehicle(ParameterInterface *parameter_interface)
 : parameter_ptr(parameter_interface) {

  parameter_interface->registerParam(EGO_VEHICLE_BOTTOM_LEFT_LOC_X);
  parameter_interface->registerParam(EGO_VEHICLE_BOTTOM_LEFT_LOC_Y);
  parameter_interface->registerParam(EGO_VEHICLE_TOP_LEFT_LOC_X);
  parameter_interface->registerParam(EGO_VEHICLE_TOP_LEFT_LOC_Y);
  parameter_interface->registerParam(EGO_VEHICLE_TOP_RIGHT_LOC_X);
  parameter_interface->registerParam(EGO_VEHICLE_TOP_RIGHT_LOC_Y);
  parameter_interface->registerParam(EGO_VEHICLE_BOTTOM_RIGHT_LOC_X);
  parameter_interface->registerParam(EGO_VEHICLE_BOTTOM_RIGHT_LOC_Y);

  update();
}

cv::_InputArray EgoVehicle::asInputArray() const {
 return toInputArray(trapezoid);
}

void EgoVehicle::update(const cv::Rect &image_limits) {
  update();

  for (cv::Point& p : trapezoid) {
    p -= image_limits.tl();
  }
}

bool EgoVehicle::contains(const cv::Point& p) const {
 return cv::pointPolygonTest(asInputArray(), p, false) >= 0;
}

void EgoVehicle::update() {
 trapezoid = {{
    cv::Point(parameter_ptr->getParam(EGO_VEHICLE_BOTTOM_LEFT_LOC_X),
              parameter_ptr->getParam(EGO_VEHICLE_BOTTOM_LEFT_LOC_Y)),
    cv::Point(parameter_ptr->getParam(EGO_VEHICLE_TOP_LEFT_LOC_X),
              parameter_ptr->getParam(EGO_VEHICLE_TOP_LEFT_LOC_Y)),
    cv::Point(parameter_ptr->getParam(EGO_VEHICLE_TOP_RIGHT_LOC_X),
              parameter_ptr->getParam(EGO_VEHICLE_TOP_RIGHT_LOC_Y)),
    cv::Point(parameter_ptr->getParam(EGO_VEHICLE_BOTTOM_RIGHT_LOC_X),
               parameter_ptr->getParam(EGO_VEHICLE_BOTTOM_RIGHT_LOC_Y))
              }};
}
