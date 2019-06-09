#include "features.h"
#include "../../utils/foot_finder.h"
#include "common/eigen_utils.h"
#include "opencv_eigen_conversions.h"

namespace road_object_detection {

Eigen::Affine3d calculateBvPatchToVehicle(const Eigen::Affine3d &vehicle_to_cluster_center,
                                          const Eigen::Vector2d &offset_to_center,
                                          double pixel_width) {
  /* offset to cluster center
   * pixel = (u,v)
   *      u
   *  +-->-------------+
   *  |     x          | birdsview
   *  v      ^         | image
   * v|      | cluster |
   *  | y <--+ center  |
   *  +----------------+
   */

  Eigen::Affine3d m_t = Eigen::Affine3d::Identity();
  // clang-format off
  m_t.linear() << 0, -1, 0,
                 -1,  0, 0,
                  0,  0, 1;
  // clang-format on
  m_t.linear() *= pixel_width;
  m_t.translation() = to3D(offset_to_center);
  return vehicle_to_cluster_center * m_t;
}

BirdsviewPatch::BirdsviewPatch(const VehiclePoint &cluster_center,
                               double cluster_center_orientation,
                               double offset_before_center,
                               double offset_after_center,
                               double offset_left_of_center,
                               double offset_right_of_center,
                               double pixel_width)
    : offset_to_center_(offset_after_center, offset_left_of_center),
      vehicle_to_cluster_center_(
          Eigen::Translation3d(cluster_center) *
          Eigen::AngleAxisd(cluster_center_orientation, VehiclePoint::UnitZ())),
      pixel_width_(pixel_width),
      bv_patch_to_vehicle_(calculateBvPatchToVehicle(
          vehicle_to_cluster_center_, offset_to_center_, pixel_width_)),
      vehicle_to_bv_patch_(bv_patch_to_vehicle_.inverse()) {
  const int height = static_cast<int>(
      std::floor((offset_before_center + offset_after_center) / pixel_width));
  const int width = static_cast<int>(
      std::floor((offset_left_of_center + offset_right_of_center) / pixel_width));
  image = cv::Mat::zeros(height, width, CV_8U);
}

const cv::Mat *BirdsviewPatch::getImage() const { return &image; }
cv::Mat *BirdsviewPatch::getImage() { return &image; }

void BirdsviewPatch::writeToImage(uchar value, int v, int u) {
  image.at<uchar>(v, u) = value;
}

ImagePoint BirdsviewPatch::vehicleToImage(const VehiclePoint &vehicle) const {
  return common::round(to2D(vehicle_to_bv_patch_ * vehicle));
}

VehiclePoint BirdsviewPatch::imageToVehicle(const ImagePoint &pixel) const {
  return imageToVehicle(ImagePointExact(pixel.cast<double>()));
}

VehiclePoint BirdsviewPatch::imageToVehicle(const ImagePointExact &pixel) const {
  return bv_patch_to_vehicle_ * to3D(pixel);
}

Features::Features(const ros::Time &timestamp,
                   const FeaturePointCluster &cluster,
                   const Eigen::Vector2d &cluster_center2i,
                   const Eigen::Vector3d &cluster_center3d,
                   const common::DynamicPolynomial &middle_lane_polynomial,
                   const LineVehiclePoints &points,
                   const Eigen::Vector3d &cluster_center_middle_lane_foot_point,
                   double cluster_center_lane_orientation,
                   const ImagePatch &image_patch,
                   // const ImagePatch& canny_patch,
                   const BirdsviewPatch &birdsview_patch,
                   const cv::Mat &image_complete,
                   const boost::optional<ROI> &roi)
    : timestamp(timestamp),
      cluster(cluster),
      cluster_center2i(cluster_center2i),
      cluster_center3d(cluster_center3d),
      middle_lane_polynomial(middle_lane_polynomial),
      cluster_center_middle_lane_foot_point(cluster_center_middle_lane_foot_point),
      cluster_center_lane_orientation(cluster_center_lane_orientation),
      points_left(points[LINESPEC_LEFT]),
      points_middle(points[LINESPEC_MIDDLE]),
      points_right(points[LINESPEC_RIGHT]),
      no_passing_points(points[LINESPEC_NO_PASSING]),
      image_patch(image_patch),
      // canny_patch(canny_patch),
      birdsview_patch(birdsview_patch),
      image_complete(image_complete),
      roi_(roi) {}

ImagePatch::ImagePatch(const cv::Point &position, const cv::Mat &image)
    : position(position), image(image) {}

}  // namespace road_object_detection
