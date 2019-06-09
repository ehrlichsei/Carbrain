#pragma once

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <opencv2/core/core.hpp>
#include <vector>
THIRD_PARTY_HEADERS_END

#include <common/polynomial.h>
#include "../utils/tf_helper_interface.h"
#include "common/parameter_interface.h"
#include "parking_spot.h"
#include "vehicle_scan_line.h"

namespace common {
class CameraTransformation;
}

namespace perpendicular_parking {

struct ConditionalMapPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool detected = false;
  MapPose pose = MapPose::Identity();
};

using ParkingSpots = std::vector<ParkingSpot>;
using ParkingSpotsRef = std::vector<ParkingSpotRef>;
using ParkingSpotsConstRef = std::vector<ParkingSpotConstRef>;

class ParkingLot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ParkingLot(ParameterInterface *const parameters_ptr,
             const common::CameraTransformation *const camera_transformation,
             const tf_helper::TFHelperInterface<double> *const world_coordinates_helper);
  ParkingLot() = delete;
  ParkingLot(ParkingLot &&) = default;
  ParkingLot &operator=(ParkingLot &&) = default;
  virtual ~ParkingLot() = default;

  virtual void update(const cv::Mat &img, const common::DynamicPolynomial &left_lane_polynomial);

  virtual ParkingSpotsConstRef freeParkingSpots();
  virtual ParkingSpotsConstRef allParkingSpots() const;

  std::map<int, ImagePoint> scanParkingSlot(const cv::Mat &img,
                                            const std::map<int, ScanLine> &scan_lines);

  bool startDetected() const;

  //!
  //! \brief setStart sets the parking lot start pose so that new parking slots
  //! might be found
  //! \param pose the new start pose.
  //!
  void setStart(const WorldPose &pose);

  //!
  //! \brief setEnd sets the parking lot end pose so that now new parking slots
  //! will be created further than this pose
  //! \param pose the new end pose.
  //!
  void setEnd(const WorldPose &pose);

  static const std::string NAMESPACE;

  //!
  //! \brief mapPose returns the map pose in world coordinates frame
  //!
  const WorldPose mapPose() const;

  virtual void reset();

 protected:
  //! Layer 1 - Boundaries
  ConditionalMapPose start_;
  ConditionalMapPose end_;
  std::vector<MapPosePtr> markings_;

  //! Layer 2
  ParkingSpots parking_spots_;

  //! \brief updateMarkings
  //!
  void updateMarkings(const cv::Mat &img, const common::DynamicPolynomial &left_lane_polynomial);
  void addNewMarkings();

  std::map<int, ScanLine> createSpotScanLines(const ParkingSpot &parking_spot,
                                              const cv::Size &img_size);
  //!
  //! \brief scanParkingSlots uses Scan Lines to check occupation state
  //!
  void scanParkingSlots(const cv::Mat &img, const common::DynamicPolynomial &left_lane_polynomial);

  ScanLines createMarkingScanLines(const int marking_idx, const cv::Size &image_size);

  virtual std::pair<double, double> computeAngleField(
      const common::polynomial::DynamicPolynomial &left_lane_polynomial) const;

  using MapPoints = WorldPoints;

  struct MarkingFeaturePoints {
    MapPoints left, right;
  };
  //!
  //! \brief scanMarking uses the scan line to detect edges of the marking (both
  //! left and right)
  //! These points will then be normal shifted to the center of the marking, so
  //! in an ideal case, they should all lie on a single line
  //! \param img the image.
  //! \param scan_lines the scan lines to scan along.
  //! \param scan_from_left whether to scan from the left.
  //! \param scan_from_right whether to scan from the right.
  //! \return the found feature points.
  //!
  MarkingFeaturePoints scanMarking(const cv::Mat &img,
                                   const ScanLines &scan_lines,
                                   bool scan_from_left,
                                   bool scan_from_right) const;

  //!
  //! \brief trimToVisibilityLimit trims scan lines such that they stop right
  //! before an obstacle and do not touch the image boundary.
  //! \param scan_line in world frame
  //! \param reference_parking_spot the scan line's parking spot
  //! \param skip_first set this if reference_parking_spot should be considered
  //! for obstacle detection. Otherwise it will be skipped.
  //! \return false if scan_line is totally out of sight
  //!
  bool trimToVisibilityLimit(WorldScanLine *const scan_line,
                             const ParkingSpot &reference_parking_spot,
                             bool skip_first = true);

  Eigen::Vector3d intersectMarkingWithPolynomial(const MapPose &marking,
                                                 const common::DynamicPolynomial &left_lane_polynomial);

  const ParameterInterface *const parameters_ptr_;
  const common::CameraTransformation *const camera_transformation_;
  const tf_helper::TFHelperInterface<double> *const world_coordinates_helper_;

  //! defines the perpendicular parking map frame. It is
  //! - perpendicular to the left lane polynomial
  //! - located at the anchor point of the start pose
  Eigen::Affine3d world_T_pp_map_ = Eigen::Affine3d::Identity();

 private:
  //!
  //! \brief projectMarkingToPolynomial updates the x coordinate of the marking
  //! such that it is on the polynomial
  //! \param marking the markings pose.
  //! \param left_lane_polynomial the left lane polynomial.
  //!
  void projectMarkingToPolynomial(MapPose &marking,
                                  const common::DynamicPolynomial &left_lane_polynomial);

  bool isInFieldOfView(const cv::Point2f &p) const;

  //! Set by ParkingLot::setStart(). Triggers world_T_pp_map_ update
  bool start_pose_changed_;

  //! flag used in ParkingLot::addNewMarkings()
  bool spawn_no_new_markings_;

  std::array<cv::Point2f, 4> field_of_vision_;

  ///@{
  //! parameters defined by the rules
  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> PARKING_SPOT_WIDTH;
  static const ParameterString<double> MARKING_WIDTH;
  ///@}

  ///@{
  //! tuning parameters for scan line detection
  static const ParameterString<int> STEP_REFERENCE_FUNCTION_LENGTH;
  static const ParameterString<double> GRADIENT_DETECTION_THLD;
  ///@}

  ///@{
  //! generic parameters for scan line creation
  static const ParameterString<int> IMAGE_PADDING;
  ///@}

  ///@{
  //! marking scan line creation parameters
  static const ParameterString<double> MARKING_SCAN_STEP_WIDTH;
  static const ParameterString<double> MARKING_SCAN_LINE_WIDTH;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_FRONT;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_BACK;
  ///@}

  ///@{
  //! parking spot scan line creation parameters
  static const ParameterString<int> N_SCAN_LINES_PER_SPOT;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_X;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_Y;
  static const ParameterString<double> SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN;
  ///@}

  ///@{
  static const ParameterString<int> N_MIN_MARKINGS;
  static const ParameterString<double> MAX_SPAWN_DISTANCE;
  static const ParameterString<double> MAX_MARKING_UPDATE_DISTANCE;
  static const ParameterString<double> MARKING_UPDATE_SHIFT_FACTOR;
  static const ParameterString<double> START_POSE_MAX_ALLOWED_DEVIATION;
  ///@}

  static const ParameterString<double> NEWTON_EPSILON;
  static const ParameterString<int> NEWTON_MAX_ITERATIONS;

  static const ParameterString<int> FIELD_OF_VISION_TOP;
  static const ParameterString<int> FIELD_OF_VISION_BOTTOM;
  static const ParameterString<int> FIELD_OF_VISION_LEFT;
  static const ParameterString<int> FIELD_OF_VISION_RIGHT;
};

}  // namespace perpendicular_parking
