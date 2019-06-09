#ifndef PARKING_LOT_GENERATOR_H
#define PARKING_LOT_GENERATOR_H

#include <common/macros.h>
#include "common/camera_transformation.h"
#include "common/polynomial.h"
#include "perception_types.h"
THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

THIRD_PARTY_HEADERS_END

namespace perpendicular_parking {
namespace testing {
/*!
 * \brief ParkingLotPoint is 3D point expressed in the following
 *CoordinateSystem:
 * The ParkingLotCoordinateSysteme has its origin at the intersection of the
 *parking_start_line and the left_lane line.
 * The x-axis points in the driving direction parallel to the left lane.
 * The y-axis points in the dircection of the parking spots.
 *
 * This frame is used to define all points needed to create generate a
 *parkingLot for testing
 */
using ParkingLotPoint = Eigen::Vector3d;
using ParkingLotPoints = common::EigenAlignedVector<ParkingLotPoint>;
using ParkingLotPose = Eigen::Affine3d;

/*!
 * \brief The GeneratedOccupationState enum is used to specify the state of the
 * generated parking spots. It  differs from the OccupationState-enum in the
 * parking_spot.cpp
 * file, in such way that the size of the object occuping the parking spot can
 * be specified.
 */
enum class GeneratedOccupationState {
  FREE,
  X,
  OBSTACLE_SMALL,
  OBSTACLE_MEDIUM,
  OBSTACLE_LARGE
};

/*!
 * \brief The ObstacleSize struct
 * defines the physical dimensions of the obsatcles used in the above struct
 * "GeneratedOccupationState"
 */
struct ObstacleSize {
  const double length_in_m;
  const double width_in_m;
  const double height_in_m;
};


/*!
 * \brief The ParkingLotGenerator class Generates a ground truth (image and
 * feature points) for the PrakingLot test
 */
class ParkingLotGenerator {


 public:
  /*!
   * \brief ParkingLotGenerator constructs a ParkingLot with the specified
   * parameters
   * \param occupation_states a std::vector containing a
   * "GeneratedOccupationState" for each parking spot
   * \param parking_start_line_angle_in_radian the angle between the left lane
   * and the parking start line
   * as current carolo cup reglement this is between 45° and 60°. (here 60°
   * (1.0472 radian) is used as
   * default argument). Note that the same angle is used for the
   * "parkingEndLine"
   * \param parking_slot_width_in_m The width of each parking spot. The default
   * is 0.35 meter
   * \param parking_slot_depth_in_m The depth of each spot. Default 0.5 meter
   */
  ParkingLotGenerator(const std::vector<GeneratedOccupationState>& occupation_states,
                      const double parking_start_line_angle_in_radian = 1.0472,
                      const double parking_slot_width_in_m = 0.350,
                      const double parking_slot_depth_in_m = 0.500);

  /*!
    * \brief getLeftLanePoints returns "number_of_points" points on the left
   * line in the generated ParkingLotCoordinates
    * \param points (output argument (std::vector) to store the specified number
   * of points)
    * \param number_of_points number of points on the left lane that will be
   * stored in the "points" vector
    */
  void getLeftLanePoints(ParkingLotPoints& points, const std::size_t number_of_points) const;


  /*!
    * \brief getLeftLanePolynom creates a left lane polynom.
    * \param parkinglot_to_vehicle_transform the transformation from
   * vehicle-frame to parking_lot-frame.
    * \param number_of_left_lane_points the number of points to use.
    * \return a left lane polynomial.
    */
  common::DynamicPolynomial getLeftLanePolynom(const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                                               const int number_of_left_lane_points = 5) const;
  /*!
   * \brief getParkingStartPose :
   * \return ParkingStartPose in the generated ParkingLotCoordinates:
   * Note that this pose must be transformed into world coordinates before
   * calling setStart(pose) of the ParkingLot class
   */
  const ParkingLotPose& getParkingStartPose() const {
    return this->parking_start_pose_;
  }

  /*!
   * \brief getParkingStopPose
   * \return ParkingStopPose in the generated ParkingLotCoordinates:
   * Note that this pose must be transformed into world coordinates before
   * calling setStart(pose)
   * of the ParkingLot class
   */
  const ParkingLotPose& getParkingEndPose() const {
    return this->parking_end_pose_;
  }
  /*!
   * \brief parkingLotPointToCvImagePoint
   * \param parkinglot_point the parkinglot_point.
   * \param parkinglot_to_vehicle_transform the transformation from
   * parking_lot-frame to vehicle-frame.
   * \param camera_transformation the camera transformation.
   * \return the parking lot point.
   */
  cv::Point parkingLotPointToCvImagePoint(const ParkingLotPoint& parkinglot_point,
                                          const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                                          const common::CameraTransformation& camera_transformation) const;
  /*!
  * \brief drawConcatenatedLine
  * \param img pointer to cv::Mat image conating the image in whcih the line is
  * drawn
  * \param parkinglot_to_vehicle_transform Affine transfomration from the
  * parkinglotCoordinates
  * (in which all the points are generated) and the vehicle coordinates.
  * \param camera_transformation transfroms from 3D Vehicle to 2D Image
  * Coordinates
  * \param parking_lot_points vector containing parking_lot_points (3D Points)
  * \param shapeClosed if true a connection line between the last and first
  * point of the "parking_lot_points"
  * vector will be drawn
  */
  void drawConcatenatedLine(cv::Mat& img,
                            const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                            const common::CameraTransformation& camera_transformation,
                            const ParkingLotPoints& parking_lot_points,
                            const bool shapeClosed = false) const;
  /*!
   * \brief drawPolygon
   * \param img pointer to a grayscale image which is used to draw the lines
   * \param parkinglot_to_vehicle_transform the transformation from
   * parking_lot-frame to vehicle-frame.
   * \param camera_transformation the camera transformation.
   * \param parking_lot_points vector of 3D ParkingLotz points which define the
   * vertices of the polygon
   */
  void drawPolygon(cv::Mat& img,
                   const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                   const common::CameraTransformation& camera_transformation,
                   const ParkingLotPoints& parking_lot_points) const;

  /*!
   * \brief drawObstacle: Draws an obstacle of the defined size in the specified
   * Postion
   * \param img the image to draw in.
   * \param parkinglot_to_vehicle_transform the transformation from
   * parking_lot-frame to vehicle-frame.
   * \param camera_transformation the camera transformation.
   * \param obstacle_size contains the dimensions os the obstacle to draw
   * \param obstacle_center_point Center point of the obstacle (ignores
   * z-coordinate and sets always z=0)
   */
  void drawObstacle(cv::Mat& img,
                    const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                    const common::CameraTransformation& camera_transformation,
                    const ObstacleSize& obstacle_size,
                    const ParkingLotPoint& obstacle_center_point) const;
  /*!
   * \brief drawParkingLot: draws the whole parking lot in an 2D grayscale image
   * Note: The perspective of the lines drawn is not correct as they are drawn
   * as a 1D line shape and have a constant width no matter the distance from a
   * camera. This error however should not influence the test results and are
   * therefore accepted to keep things simple.
   * \param img the  grayscale image to draw in.
   * \param parkinglot_to_vehicle_transform the transformation from
   * parking_lot-frame to vehicle-frame.
   * \param camera_transformation the camera transformation.
   */
  void drawParkingLot(cv::Mat& img,
                      const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                      const common::CameraTransformation& camera_transformation) const;
  /*!
   * \brief debug_drawPolynom
   * \param debug_img the image to draw in.
   * \param left_lane_polynom the polynomial to draw.
   * \param camera_transformation the camera transformation.
   * \param color the color to use.
   */
  void debugDrawPolynom(cv::Mat& debug_img,
                        const common::DynamicPolynomial& left_lane_polynom,
                        const common::CameraTransformation& camera_transformation,
                        const cv::Scalar& color) const;



  /*!
   * \brief debug_printAllGeneratedPoints prints generated points in
   * ParkingLotFrameCoordinates to console
   */
  void debugPrintAllGeneratedPoints();


  /*!
   * \brief ParkingLotGenerator::drawPoints : draws specified points in
   * debugImage
   * \param points the points to draw (in parking_lot-frame).
   * \param debug_img the image to draw to.
   * \param camera_transformation the camera transformation.
   * \param parkinglot_to_vehicle_transform the transformation from
   *   parking_lot-frame to vehicle-frame.
   * \param color the colors to use.
   */
  void drawPoints(const ParkingLotPoints points,
                  cv::Mat& debug_img,
                  const common::CameraTransformation& camera_transformation,
                  const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                  const cv::Scalar& color) const;

  /*!
   * \brief debug_drawGeneratedPoints draws all generated points into the debug
   * image
   * \param debug_img the image to draw to.
   * \param camera_transformation the camera transformation.
   * \param parkinglot_to_vehicle_transform the transformation from
   *   parking_lot-frame to vehicle-frame.
   * \param color the color to use.
   */
  void debugDrawGeneratedPoints(cv::Mat& debug_img,
                                const common::CameraTransformation& camera_transformation,
                                const Eigen::Affine3d& parkinglot_to_vehicle_transform,
                                const cv::Scalar& color) const;
  /*!
   * \brief debug_drawPose
   * \param debug_img the image to draw to.
   * \param pose => expected to be in vehicle coordinates
   * \param camera_transformation the camera transformation.
   * \param color the color to use.
   */
  void debugDrawPose(cv::Mat& debug_img,
                     const VehiclePose& pose,
                     const common::CameraTransformation& camera_transformation,
                     const cv::Scalar& color,
                     const int drawAxis = 3) const;

 private:
  // Parameters for ParkinLotGeneration
  std::vector<GeneratedOccupationState> occupation_states_;
  std::size_t number_of_parking_slots_;

  const double parking_start_line_angle_in_radian_;
  const double parking_slot_width_in_m_;
  const double parking_slot_depth_in_m_;

  // generated points used to draw the parking lot:
  ParkingLotPoints left_lane_points_;
  ParkingLotPoints parking_start_line_points_;
  ParkingLotPoints parking_stop_line_points_;
  //  parking_spots_points_ containing 4 corners of each parking spot in the
  //  following order:
  // TopRight, BottomRight, BottomLeft, TopLeft,...
  ParkingLotPoints parking_spots_points_;

  // Pose of parkingStartLine in ParkingLot Coordinate:
  ParkingLotPose parking_start_pose_;
  ParkingLotPose parking_end_pose_;

  static const ObstacleSize OBSTACLE_SIZE_SMALL;
  static const ObstacleSize OBSTACLE_SIZE_MEDIUM;
  static const ObstacleSize OBSTACLE_SIZE_LARGE;

  static const int POLYNOMIAL_DEGREE_LINE = 1;
};
}  // namespace testing
}  // namespace perpendicular parking



#endif  // PARKING_LOT_GENERATOR_H
