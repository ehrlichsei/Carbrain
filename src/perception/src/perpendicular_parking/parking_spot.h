#pragma once

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <perception_msgs/PerpendicularParkingSpot.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
THIRD_PARTY_HEADERS_END

#include <common/parameter_interface.h>
#include "common/camera_transformation.h"
#include "perception_types.h"
#include "scan_line.h"

namespace perpendicular_parking {

enum class OccupationState { FREE, OCCUPIED, X, UNKNOWN };

using MapPoint = WorldPoint;
using MapPose = WorldPose;
using MapPosePtr = std::shared_ptr<MapPose>;
using MapPoseConstPtr = std::shared_ptr<const MapPose>;
using MapLine = Eigen::ParametrizedLine<double, 2>;
using MapLines = std::vector<MapLine>;
using ImageLine = Eigen::ParametrizedLine<double, 2>;
using ImageLines = std::vector<ImageLine>;
using IntersectionLine = Eigen::Hyperplane<double, 2>;

using MapPoints = common::EigenAlignedVector<MapPoint>;

struct ObservationPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPoint point;  //!< end of scan line, but in world coordinates
  enum class Type { VISIBILITY_LIMIT, OBSTACLE };
  Type type;
};

using ObservationPoints = std::map<int, ObservationPoint>;

class ParkingSpot {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //!
  //! \brief ParkingSpot
  //! \param id the identifier of the parking spot.
  //! \param left_entrance pose with x pointing into the parking lot
  //! \param right_entrance pose with x pointing into the parking lot
  //!
  ParkingSpot(int id,
              const MapPoseConstPtr &left_entrance,
              const MapPoseConstPtr &right_entrance,
              const ParameterInterface *const parameter_interface,
              const common::CameraTransformation *const cam_transform,
              const std::array<cv::Point2f, 4> &fov);
  ParkingSpot() = delete;

  OccupationState occupationState() const;

  perception_msgs::PerpendicularParkingSpot asMsg() const;

  const MapPose &leftEntrance() const;
  const MapPose &rightEntrance() const;

  //!
  //! \brief setObservationPoints
  //! \param observation_points points that are ordered from left to right
  //! entrance
  //!
  void updateObservation(const ObservationPoints &observation_points,
                         const std::pair<double, double> &vertical_angle_limits,
                         const MapPose &map_T_vehicle);

  MapPoints freeSpace() const;

  const MapPoint &rightMostObstaclePoint() const;

  int id() const { return id_; }

  bool operator==(const ParkingSpot &rhs) const { return id_ == rhs.id_; }

  ObservationPoints observationPoints() const {
    return this->observation_points_;
  }

  MapLines detectedLines() const { return this->detected_lines_; }

  static void registerParams(ParameterInterface *parameters_ptr);

  static const std::string NAMESPACE;

 private:
  bool isSpotNoParkingArea() const;
  double occupiedArea() const;
  bool isShadowedByObstacle(const double occupied_area) const;
  MapLines fitLinesR(const std::size_t nr_lines) const;
  MapLine leastSquaresFit(const MapPoints &points) const;

  bool isInFieldOfView(const cv::Point2f &p) const;

  Eigen::Affine3d vehicle_T_map_ = MapPose::Identity();

  int id_;
  const MapPoseConstPtr left_entrance_;
  const MapPoseConstPtr right_entrance_;

  mutable bool recently_updated_ = false;
  mutable OccupationState latest_state_ = OccupationState::UNKNOWN;

  ObservationPoints observation_points_;
  MapLines detected_lines_;
  std::pair<double, double> vertical_angle_limits_;

  const common::CameraTransformation *const camera_transformation_;

  const std::array<cv::Point2f, 4> field_of_vision_;

  ///@{
  //! parameters for determination whether parking spot is no parking area
  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> PARKING_SPOT_WIDTH;
  static const ParameterString<int> POINTS_TO_SKIP_FRONT;
  static const ParameterString<int> POINTS_TO_SKIP_BACK;
  static const ParameterString<double> EPSILON_X_CLASSIFIER;
  static const ParameterString<double> SMALLEST_OBSTACLE_AREA;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_X;
  static const ParameterString<double> RANSAC_MIN_DIST;
  static const ParameterString<int> RANSAC_MIN_CLUSTER_SIZE;
  static const ParameterString<int> RANSAC_NR_LINES;
  ///@}

  const double PARKING_SPOT_DEPTH_;
  const double PARKING_SPOT_WIDTH_;
  const double EPSILON_X_CLASSIFIER_;
  const double SMALLEST_OBSTACLE_AREA_;
  const double SPOT_SCAN_LINE_PADDING_X_;
  const int POINTS_TO_SKIP_BACK_;
  const int POINTS_TO_SKIP_FRONT_;
  const double RANSAC_MIN_DIST_;
  const std::size_t RANSAC_MIN_CLUSTER_SIZE_;
  const std::size_t RANSAC_NR_LINES_;

  ///@{
  //! parameters for decision making
  static const ParameterString<double> FREE_MIN_DEPTH_TH;
  ///@}
};

using ParkingSpotRef = std::reference_wrapper<ParkingSpot>;
using ParkingSpotConstRef = std::reference_wrapper<ParkingSpot const>;

}  // namespace perpendicular_parking
