#ifndef FEATURES_H
#define FEATURES_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/time.h>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
THIRD_PARTY_HEADERS_END

#include "../../utils/dbscan_clusterer.h"
#include "common/polynomial.h"
#include "line_vehicle_points.h"

namespace road_object_detection {

struct ImagePatch {
  /*!
   * \brief ImagePatch struct to contain input parameters: image (patch) and its
   * original position
   * \param position position of upper left corner in the original camera image
   * \param image the contained image patch.
   */
  ImagePatch(const cv::Point &position, const cv::Mat &image);
  //! \brief position of upper left corner in the original camera image
  const cv::Point position;
  //! \brief the contained image patch
  const cv::Mat image;
};



struct ROI {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * \brief img_patch_ image containing the roi; subset of the complete camera image
   */
  ImagePatch img_patch_;
  /*!
   * \brief pose_ pose representing the center of the roi
   */
  VehiclePose pose_;
  /*!
   * \brief boundary_ bounding points of roi
   */
  VehiclePoints boundary_;
};


/**
 * contains a birdsview patch of the camer image around the found feature
 * cluster
 */
class BirdsviewPatch {
 public:
  /**
   * @param cluster_center center of the feature cluster
   * @param cluster_center_orientation orientation of the lane at the feature
   * cluster
   * @param pixel_width width of pixel in meter in birdsview image
   */
  BirdsviewPatch(const VehiclePoint &cluster_center,
                 double cluster_center_orientation,
                 double offset_before_center,
                 double offset_after_center,
                 double offset_left_of_center,
                 double offset_right_of_center,
                 double pixel_width);

  /**
   * @return pointer to birdsview patch image
   */
  const cv::Mat *getImage() const;
  cv::Mat *getImage();

  /**
   * writes value to image coordinates
   *  +-->-------------+
   *  |    u           |
   *  v                |
   *  |v               |
   *  |                |
   *  +----------------+
   */
  void writeToImage(uchar value, int v, int u);

  /**
   * transformes vehicle point to pixel at birdsview image
   */
  ImagePoint vehicleToImage(const VehiclePoint &vehicle) const;

  /**
   * transforms pixel coordinate to vehicle coordinates
   */
  VehiclePoint imageToVehicle(const ImagePoint &pixel) const;
  VehiclePoint imageToVehicle(const ImagePointExact &pixel) const;

 private:
  cv::Mat image;
  const ImagePointExact offset_to_center_;
  const Eigen::Affine3d vehicle_to_cluster_center_;
  const double pixel_width_;
 public:
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
  const Eigen::Affine3d bv_patch_to_vehicle_;
  const Eigen::Affine3d vehicle_to_bv_patch_;
};

struct Features {
 public:
  /*!
   * \brief Features struct to contain all features/information concerning a
   * cluster of feature points along the road, i.e. a RoadObject
   * \param timestamp timestamp of original image
   * \param cluster feature point cluster
   * \param cluster_center2i center of the cluster in image coordinates
   * \param cluster_center3d center of the cluster in vehicle coordinates
   * \param middle_lane_polynomial polynomial representing the middle lane line
   * \param cluster_center_middle_lane_foot_point projected center point to
   * middle lane in vehicle coordinates
   * \param cluster_center_lane_orientation angle of lane orientation at cluster
   * in x,y-plane starting at x axis
   * \param image_patch part of the original image enclosing the region of
   * interest
   * \param birdsview_patch cv::Mat containing the birdsview transformation of
   * the image_patch
   */
  Features(const ros::Time &timestamp,
           const FeaturePointCluster &cluster,
           const ImagePointExact &cluster_center2i,
           const VehiclePoint &cluster_center3d,
           const common::DynamicPolynomial &middle_lane_polynomial,
           const LineVehiclePoints &points,
           const VehiclePoint &cluster_center_middle_lane_foot_point,
           double cluster_center_lane_orientation,
           const ImagePatch &image_patch,
           // const ImagePatch& canny_patch,
           const BirdsviewPatch &birdsview_patch,
           const cv::Mat &image_complete,
           const boost::optional<ROI> &roi);

  const ros::Time timestamp;

  const FeaturePointCluster cluster;

  /**
   * \brief cluster center in image coordinates
   */
  const ImagePointExact cluster_center2i;

  /**
   * \brief cluster center in vehicle coordinates
   */
  const VehiclePoint cluster_center3d;

  /**
   * \brief polynomial along the middle lane in vehicle coordinates
   */
  const common::DynamicPolynomial middle_lane_polynomial;

  /**
   * \brief projected center point to middle lane in vehicle coordinates
   */
  const VehiclePoint cluster_center_middle_lane_foot_point;

  /**
   * \brief angle of lane orientation at cluster in x,y-plane starting at x axis
   */
  const double cluster_center_lane_orientation;
  /**
   * @brief left bounding points of lane in vehicle coordinates
   */
  const VehiclePoints points_left;
  /**
   * @brief middle points of lane in vehicle coordinates
   */
  const VehiclePoints points_middle;
  /**
   * @brief right bounding points of lane in vehicle coordinates
   */
  const VehiclePoints points_right;
  /**
   * @brief no passing points in vehicle coordinates
   */
  const VehiclePoints no_passing_points;

  const ImagePatch image_patch;
  // const ImagePatch canny_patch;
  const BirdsviewPatch birdsview_patch;
  const cv::Mat image_complete;

  const boost::optional<ROI> roi_;

  // private:
};

}  // namespace road_object_detection

#endif  // FEATURES_H
