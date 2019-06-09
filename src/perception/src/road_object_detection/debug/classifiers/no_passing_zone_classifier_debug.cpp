#include "no_passing_zone_classifier_debug.h"
#include <ros/console.h>
#include "common/console_colors.h"
#include "common/types.h"


namespace road_object_detection {

NoPassingZoneClassifierDebug::NoPassingZoneClassifierDebug(
    const common::CameraTransformation* const camera_transformation,
    common::node_base::ParameterInterface* const parameters_ptr,
    DebugImages* debug_images)
    : NoPassingZoneClassifier(camera_transformation, parameters_ptr),
      ClassifierDebug(debug_images) {}


RoadObjects NoPassingZoneClassifierDebug::classify(const Features& features) {
  auto ret = NoPassingZoneClassifier::classify(features);  // RoadObjects no_passing_zones;


  // auto npzone = *dynamic_cast<NoPassingZone*>(ret.back().get());

  // const Obstacle obst = *dynamic_cast<Obstacle*>(ret.back().get());

  if (ret.empty()) {
    return ret;
  }

  NoPassingZone npzone = *dynamic_cast<NoPassingZone*>(ret.back().get());
  np_debug(std::to_string(npzone.score));
  ImagePoint np_start_img_point =
      camera_transformation_->transformGroundToImage(npzone.start_point_vehicle);
  ImagePoint np_end_img_point =
      camera_transformation_->transformGroundToImage(npzone.end_point_vehicle);

  cv::line(*debug_images->getCameraImage(),
           imagePointToCvPoint(np_start_img_point),
           imagePointToCvPoint(np_end_img_point),
           cv::Vec3b(0, 255, 255),
           3);


  /*
  for (auto &ro : ret) {

   ro->base_hull_polygon_in_vehicle;
   cv::line(*debug_images->getCameraImage(), ro->start, ro->end, cv::Vec3b(0,255,255), 3);
  }
  */

  // auto ret_steps = getSteps(features);
  // drawSteps(ret_steps,cv::Vec3b(0,255,255));



  // Draw NP Points from features
  /*for (auto np_point : features.no_passing_points) {
    auto np_img_point = camera_transformation_->transformGroundToImage(np_point);
    cv::circle(*debug_simages->getCameraImage(), imagePointToCvPoint(np_img_point), 2, cv::Vec3b(0,150,150), -1);
  }*/

  return ret;
}

/*void NoPassingZoneClassifierDebug::np_debug(std::string str) {
  ROS_DEBUG_STREAM("NP_DEBUG: " << str);
}
*/


void NoPassingZoneClassifierDebug::np_debug(const std::string& str) const {
  ROS_DEBUG_STREAM("NP_DEBUG:" << str);
}

void NoPassingZoneClassifierDebug::drawSteps(const std::vector<VehiclePoints>& step_clstrs_gnd,
                                             const cv::Vec3b& color) {

  for (const auto& step_clstr_gnd : step_clstrs_gnd) {
    ImagePoints step_clstr_img;
    camera_transformation_->transformGroundToImage(step_clstr_gnd, &step_clstr_img);

    for (const auto& step_img : step_clstr_img) {
      cv::circle(*debug_images->getCameraImage(), imagePointToCvPoint(step_img), 2, color, -1);
    }
  }
}
}  // namespace road_object_detection
