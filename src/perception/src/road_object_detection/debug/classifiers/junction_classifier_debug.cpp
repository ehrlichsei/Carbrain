#include "junction_classifier_debug.h"
#include "opencv_eigen_conversions.h"

#include "common/best_score.h"

#include<chrono>
#include<deque>

namespace road_object_detection {

JunctionClassifierDebug::JunctionClassifierDebug(ParameterInterface* parameter_interface,
                                                 const common::CameraTransformation* const cam_transform,
                                                 DebugImages* debug_images)
    : JunctionClassifier(parameter_interface, cam_transform),
      ClassifierDebug(debug_images) {
      JunctionClassifierDebug::DEBUG_MODE = true;
      }

RoadObjects JunctionClassifierDebug::classify(const Features& features) {


  RoadObjects road_objects = JunctionClassifier::classify(features);

  cv::Mat* debug_birdsview = debug_images->getBirdsviewPatch(features.cluster.id);

  // JunctionClassifierDebug::drawPoints(
  //     debug_birdsview, JunctionClassifier::neg_steps_box, cv::Scalar(0, 255,
  //     0));
  // JunctionClassifierDebug::drawPoints(
  //     debug_birdsview, JunctionClassifier::pos_steps_box, cv::Scalar(0, 255,
  //     0));

  for (auto& ro : road_objects) {
    const Junction* junction = dynamic_cast<Junction*>(ro.get());
    const VehiclePoint start_vec = junction->pose_in_vehicle.translation();
    const auto start = camera_transformation_->transformGroundToImage(start_vec);
    const auto end = camera_transformation_->transformGroundToImage(
        start_vec +
        0.4 * (junction->pose_in_vehicle.linear() * VehiclePoint::UnitX()).normalized());

    cv::arrowedLine(*debug_birdsview, toCV(start), toCV(end), cv::Scalar(255, 0, 0));
  }

  return road_objects;
}

JunctionClassifier::FourCandidates JunctionClassifierDebug::findFourCandidates(
    const Features& features, const ImagePoints& pos_steps_box, const ImagePoints& neg_steps_box) const {
  const auto candidates =
      JunctionClassifier::findFourCandidates(features, pos_steps_box, neg_steps_box);

  cv::Mat* debug_birdsview = debug_images->getBirdsviewPatch(features.cluster.id);

  const auto winning_type = *common::max_score(allJunctionTypes(),
                                               [&candidates](const auto& t) {
                                                 return candidates[t].score[t];
                                               });
  const CandidateData& winner_data = candidates.at(winning_type);

  cv::rectangle(*debug_birdsview, winner_data.window, cv::Scalar(255, 0, 255));

  // cv::Point eigen_vec = pca_result.eigen_vecs[0];
  // cv::Point mean = cv::Point(pca_result.means[0], pca_result.means[1]);
  // cv::Point mean_start_no_offset = mean + eigen_vec *
  // JunctionClassifier::lenght_eigenline;
  // cv::Point mean_end_no_offset = mean - eigen_vec *
  // JunctionClassifier::lenght_eigenline;
  // cv::Point mean_start_offset = mean + eigen_vec *
  // JunctionClassifier::lenght_eigenline_offset;
  // cv::Point mean_end_offset = mean - eigen_vec *
  // JunctionClassifier::lenght_eigenline_offset;

  // cv::Point mean_start_plus_offset = JunctionClassifier::CvPointOffsetInY(
  //    mean_start_offset, JunctionClassifier::offset_eigenline);
  // cv::Point mean_end_plus_offset = JunctionClassifier::CvPointOffsetInY(
  //    mean_end_offset, JunctionClassifier::offset_eigenline);

  // cv::Point mean_start_minus_offset = JunctionClassifier::CvPointOffsetInY(
  //    mean_start_offset, -JunctionClassifier::offset_eigenline);
  // cv::Point mean_end_minus_offset = JunctionClassifier::CvPointOffsetInY(
  //    mean_end_offset, -JunctionClassifier::offset_eigenline);

  cv::line(*debug_birdsview,
           toCV(winner_data.horizontal_eigen_line.start),
           toCV(winner_data.horizontal_eigen_line.end),
           cv::Scalar(155, 155, 255),
           1);
  cv::line(*debug_birdsview,
           toCV(winner_data.horizontal_eigen_line_plus.start),
           toCV(winner_data.horizontal_eigen_line_plus.end),
           cv::Scalar(155, 155, 255),
           1);
  cv::line(*debug_birdsview,
           toCV(winner_data.horizontal_eigen_line_minus.start),
           toCV(winner_data.horizontal_eigen_line_minus.end),
           cv::Scalar(155, 155, 255),
           1);
  if (winner_data.dist_middlelane_pca_mean > 0) {
    cv::line(*debug_birdsview,
             toCV(winner_data.vertical_eigen_line_plus.start),
             toCV(winner_data.vertical_eigen_line_plus.end),
             cv::Scalar(155, 155, 255),
             1);
  } else {
    cv::line(*debug_birdsview,
             toCV(winner_data.vertical_eigen_line_minus.start),
             toCV(winner_data.vertical_eigen_line_minus.end),
             cv::Scalar(155, 155, 255),
             1);
  }
  // cv::line(*debug_birdsview,
  //          mean_start_minus_offset,
  //          mean_end_minus_offset,
  //          cv::Scalar(255, 0, 255),
  //          1);
  // cv::line(*debug_birdsview, mean_start_no_offset, mean_end_no_offset,
  // cv::Scalar(255, 0, 255), 1);


  // cv::rectangle(*debug_birdsview,
  //               cv::Point(borders_rect.x, borders_rect.y),
  //               cv::Point(borders_rect.x + borders_rect.width,
  //                         borders_rect.y + borders_rect.height),
  //               cv::Scalar(255, 255, 255));

  JunctionClassifierDebug::drawPoints(
      debug_birdsview, winner_data.steps_window, cv::Scalar(0, 255, 255));

  int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.6;
  int thickness = 1;

  // ImagePoint point=ImagePoint(mean);
  // JunctionClassifier::getLineStepPointsNoDoubling(point, point, 4, 3,
  // *debug_birdsview, 5);
  //
  //
  std::string text = std::to_string(features.cluster.id);
  cv::putText(
      *debug_birdsview, text, cv::Point(30, 20), fontFace, fontScale, cv::Scalar(0, 0, 255), thickness, 6);
  // std::string text1 =
  // std::to_string(pca_result.eigen_vals[1]);
  // cv::putText(
  //   *debug_birdsview, text1, cv::Point(30, 40), fontFace, fontScale,
  //   cv::Scalar(255, 0, 0), thickness, 6);
  // std::string text2 = std::to_string(angle_diff);
  // cv::putText(
  //   *debug_birdsview, text2, cv::Point(30, 20), fontFace, fontScale,
  //   cv::Scalar(0, 0, 255), thickness, 6);
  // std::string text3 = std::to_string(pos_neg_ratio);
  // cv::putText(
  //   *debug_birdsview, text3, cv::Point(30, 40), fontFace, fontScale,
  //   cv::Scalar(0, 255, 255), thickness, 6);
  // std::string text4 = std::to_string(dist_middlelane_pca_mean);
  // cv::putText(
  //   *debug_birdsview, text4, cv::Point(30, 60), fontFace, fontScale,
  //   cv::Scalar(0, 255, 0), thickness, 6);
  // std::string text5 = std::to_string(features.cluster.id);
  // cv::putText(
  //   *debug_birdsview, text5, cv::Point(pca_result.means[0],
  //   pca_result.means[1]), fontFace, fontScale,
  //   cv::Scalar(100, 255, 100), thickness, 6);

  // cv::threshold(*debug_birdsview, *debug_birdsview, 100, 255
  // ,cv::THRESH_BINARY);
  // cv::medianBlur(*debug_birdsview, *debug_birdsview, 5);
  // cv::Mat1b birdsview_bw;
  // cv::adaptiveThreshold(*features.birdsview_patch.getImage(),
  //                       birdsview_bw,
  //                       255,
  //                       cv::ADAPTIVE_THRESH_GAUSSIAN_C,
  //                       cv::THRESH_BINARY,
  //                       15,
  //                       -20);
  // cv::medianBlur(birdsview_bw, birdsview_bw, 5);
  // debug_images->addBirdsviewPatch(153, JunctionClassifier::birdsview_median);
  // debug_images->addBirdsviewPatch(153, JunctionClassifier::birdsview_binary);


  // ROS_DEBUG("Junction Results: %s : %i :%f",
  //           "PCA Eigenval 0",
  //           features.cluster.id,
  //           pca_result.eigen_vals[0]);
  // ROS_DEBUG("Junction Results: %s : %i :%f",
  //           "PCA Eigenval 1",
  //           features.cluster.id,
  //           pca_result.eigen_vals[1]);
  // ROS_DEBUG("Junction Results: %s : %i :%f", "Angle Difference",
  // features.cluster.id, angle_diff);
  // ROS_DEBUG("Junction Results: %s : %i :%f", "Dist middle/mean",
  // features.cluster.id, dist_middlelane_pca_mean);
  // ROS_DEBUG("Junction Results: %s : %i :%i", "Steps Top line",
  // features.cluster.id, nr_horizontal_eigen_line_plus_points);
  // ROS_DEBUG("Junction Results: %s : %i :%i", "Steps mid line",
  // features.cluster.id, nr_horizontal_eigen_line_points);
  // ROS_DEBUG("Junction Results: %s : %i :%i", "Steps dwn line",
  // features.cluster.id, nr_horizontal_eigen_line_minus_points);
  // ROS_DEBUG("Junction Results: %s : %i :%i",
  //           "Steps no_med_mid line",
  //           features.cluster.id,
  //           nr_horizontal_eigen_line_points_no_median);
  // ROS_DEBUG("Junction Results: %s : %i :%f", "pos/neg ratio",
  // features.cluster.id, pos_neg_ratio);
  ROS_DEBUG("Junction Results: ID:  %i ", features.cluster.id);
  ROS_DEBUG(
      "Junction Results: E0: %f\tE1 :%f\tAD: %f\t DST: %f\tHP: %i\tH: %i\tHM: "
      "%i\tHNM: %i\tPN: %f\t",
      winner_data.pca_data.eigen_vals[0],
      winner_data.pca_data.eigen_vals[1],
      winner_data.angle_diff,
      winner_data.dist_middlelane_pca_mean,
      winner_data.nr_horizontal_eigen_line_plus_points,
      winner_data.nr_horizontal_eigen_line_points,
      winner_data.nr_horizontal_eigen_line_minus_points,
      winner_data.nr_horizontal_eigen_line_points_no_median,
      winner_data.pos_neg_ratio);

  ROS_DEBUG(
      "Junction Results: STOP-Links: %f\tSTOP-Rechts: %f\tGIVE_WAY-links: "
      "%f\tGIVE_WAY-rechts: %f\t ",
      winner_data.score[Junction::stopline_left],
      winner_data.score[Junction::stopline_right],
      winner_data.score[Junction::givewayline_left],
      winner_data.score[Junction::givewayline_right]);

  // ROS_DEBUG("pca Results: %s -> %f", "EigenVal 1", pca_result.eigen_vals[1]);
  // ROS_DEBUG("pca Results: %s -> %f", "Angle Diff", angle_diff);
  // ROS_DEBUG("pca Results: %s -> %f", "Pos/Neg Ratio", pos_neg_ratio);
  // ROS_DEBUG("pca Results: %s -> %f", "Distance Mean to Middle Lane",
  // dist_middlelane_pca_mean);
  // ROS_DEBUG("pca Results: %s -> %i", "Number of Points in top EigenLane",
  // nr_eigen_line_minus_points);
  // ROS_DEBUG("pca Results: %s -> %i", "Number of Points in middle EigenLane",
  // nr_eigen_line_points);
  // ROS_DEBUG("pca Results: %s -> %i", "Number of Points in bottom EigenLane",
  // nr_eigen_line_plus_points);
  if (winner_data.score[Junction::stopline_left] > 0.2) {
    ROS_DEBUG("Junction Results: LEFT STOP LINE FOUND!");
  }
  if (winner_data.score[Junction::stopline_right] > 0.2) {
    ROS_DEBUG("Junction Results: RIGHT STOP LINE FOUND!");
  }
  if (winner_data.score[Junction::givewayline_left] > 0.2) {
    ROS_DEBUG("Junction Results: LEFT GIVE WAY LINE FOUND!");
  }
  if (winner_data.score[Junction::givewayline_right] > 0.2) {
    ROS_DEBUG("Junction Results: RIGHT GIVE WAY LINE FOUND!");
  }

  return candidates;
}


void JunctionClassifierDebug::drawBox(cv::Mat* debug_birdsview, const Boarders& boarders) const {
  cv::line(*debug_birdsview,
           cv::Point(boarders.min_x, boarders.max_y),
           cv::Point(boarders.max_x, boarders.max_y),
           cv::Scalar(255, 0, 0));

  cv::line(*debug_birdsview,
           cv::Point(boarders.min_x, boarders.min_y),
           cv::Point(boarders.max_x, boarders.min_y),
           cv::Scalar(255, 0, 0));
  cv::line(*debug_birdsview,
           cv::Point(boarders.min_x, boarders.min_y),
           cv::Point(boarders.min_x, boarders.max_y),
           cv::Scalar(255, 0, 0));
  cv::line(*debug_birdsview,
           cv::Point(boarders.max_x, boarders.min_y),
           cv::Point(boarders.max_x, boarders.max_y),
           cv::Scalar(255, 0, 0));
}

void JunctionClassifierDebug::drawPoints(cv::Mat* debug_birdsview,
                                         const ImagePoints& points,
                                         const cv::Scalar& color) const {
  for (const ImagePoint& point : points) {
    cv::circle(*debug_birdsview, toCV(point), 1, color, -1);
  }
}


}  // namespace road_object_detection
