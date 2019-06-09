#include <common/angle_conversions.h>
#include <common/camera_transformation.h>
#include <common/discretisize.h>
#include <common/macros.h>
#include <common/polynomial.h>
#include <common/polynomialfit.h>
#include <common/test/dummy_parameter_handler.h>
#include <opencv_eigen_conversions.h>
#include <perception_types.h>
#include "../../../src/perpendicular_parking/parking_lot.h"
#include "../../../src/utils/tf_helper_interface.h"
#include "parking_lot_generator.h"

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Geometry>
#include <boost/range/algorithm/transform.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

namespace perpendicular_parking {
namespace testing {

class MyDummyParamHandler : public DummyParameterHandler {

 public:
  MyDummyParamHandler() {
    // Initialize parking lot parameters
    addParam(PARKING_SPOT_DEPTH, 0.5);
    addParam(PARKING_SPOT_WIDTH, 0.35);
    addParam(MARKING_WIDTH, 0.02);

    addParam(STEP_REFERENCE_FUNCTION_LENGTH, 9);
    addParam(GRADIENT_DETECTION_THLD, 4.0);

    addParam(IMAGE_PADDING, 10);

    addParam(MARKING_SCAN_STEP_WIDTH, 0.02);
    addParam(MARKING_SCAN_LINE_WIDTH, 0.08);
    addParam(MARKKING_SCAN_LINE_PADDING_FRONT, 0.07);
    addParam(MARKKING_SCAN_LINE_PADDING_BACK, 0.15);

    addParam(N_SCAN_LINES_PER_SPOT, 19);
    addParam(SPOT_SCAN_LINE_PADDING_X, 0.03);
    addParam(SPOT_SCAN_LINE_PADDING_Y, 0.08);
    addParam(SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN, 0.02);

    addParam(N_MIN_MARKINGS, 3);  // This params sets minimum spots per lot
    addParam(MAX_SPAWN_DISTANCE, 1.8);
    addParam(MAX_MARKING_UPDATE_DISTANCE, 0.8);
    addParam(MARKING_UPDATE_SHIFT_FACTOR, 0.7);
    addParam(START_POSE_MAX_ALLOWED_DEVIATION, 0.2);

    addParam(NEWTON_EPSILON, 0.01);
    addParam(NEWTON_MAX_ITERATIONS, 20);

    addParam(FIELD_OF_VISION_TOP, 90);
    addParam(FIELD_OF_VISION_BOTTOM, 550);
    addParam(FIELD_OF_VISION_LEFT, 0);
    addParam(FIELD_OF_VISION_RIGHT, 1289);

    // Initializie ParkinSpot Parameters
    addParam(POINTS_TO_SKIP_FRONT, 3);
    addParam(POINTS_TO_SKIP_BACK, -2);
    addParam(EPSILON_X_CLASSIFIER, 0.2);
    addParam(MIN_NUMBER_HITS, 4);
    addParam(SMALLEST_OBSTACLE_AREA, 0.02);
    addParam(FREE_MIN_DEPTH_TH, 0.3);

    // Initialize camera parameters
    addParam(CAMERA_FOCAL_LENGTH_X, 386.704446);
    addParam(CAMERA_FOCAL_LENGTH_Y, 385.973724);
    addParam(CAMERA_OPTICAL_CENTER_X, 623.233113);
    addParam(CAMERA_OPTICAL_CENTER_Y, 125.752428);

    addParam(CAMERA_R11, 0.0214821);
    addParam(CAMERA_R12, -0.999761);
    addParam(CAMERA_R13, -0.00402491);
    addParam(CAMERA_R21, -0.149211);
    addParam(CAMERA_R22, 0.00077467);
    addParam(CAMERA_R23, -0.988805);
    addParam(CAMERA_R31, 0.988572);
    addParam(CAMERA_R32, 0.0218421);
    addParam(CAMERA_R33, -0.149159);

    addParam(CAMERA_T1, 7.42816);
    addParam(CAMERA_T2, 262.261);
    addParam(CAMERA_T3, 32.6001);

    // World coordinate helper parameters
    addParam(MAX_TF_LOOKUP_DURATION, 0.0);

    addParam(
        ParameterString<double>("parking_spot/ransac/minimal_model_distance"), 0.01);
    addParam(
        ParameterString<int>("parking_spot/ransac/minimal_consensus_set_size"), 5);
    addParam(ParameterString<int>("parking_spot/ransac/expected_nr_of_lines"), 3);
  }

  // ParkingLot class parameters
  static const std::string NAMESPACE;

  static const ParameterString<double> PARKING_SPOT_DEPTH;
  static const ParameterString<double> PARKING_SPOT_WIDTH;
  static const ParameterString<double> MARKING_WIDTH;

  static const ParameterString<int> STEP_REFERENCE_FUNCTION_LENGTH;
  static const ParameterString<double> GRADIENT_DETECTION_THLD;

  static const ParameterString<int> IMAGE_PADDING;

  static const ParameterString<double> MARKING_SCAN_STEP_WIDTH;
  static const ParameterString<double> MARKING_SCAN_LINE_WIDTH;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_FRONT;
  static const ParameterString<double> MARKKING_SCAN_LINE_PADDING_BACK;

  static const ParameterString<int> N_SCAN_LINES_PER_SPOT;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_X;
  static const ParameterString<double> SPOT_SCAN_LINE_PADDING_Y;
  static const ParameterString<double> SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN;

  static const ParameterString<int> N_MIN_MARKINGS;
  static const ParameterString<double> MAX_SPAWN_DISTANCE;
  static const ParameterString<double> MAX_MARKING_UPDATE_DISTANCE;
  static const ParameterString<double> MARKING_UPDATE_SHIFT_FACTOR;
  static const ParameterString<double> START_POSE_MAX_ALLOWED_DEVIATION;

  static const ParameterString<double> NEWTON_EPSILON;
  static const ParameterString<int> NEWTON_MAX_ITERATIONS;

  static const ParameterString<int> FIELD_OF_VISION_TOP;
  static const ParameterString<int> FIELD_OF_VISION_BOTTOM;
  static const ParameterString<int> FIELD_OF_VISION_LEFT;
  static const ParameterString<int> FIELD_OF_VISION_RIGHT;

  // ParkingSpot class parameters
  static const ParameterString<int> POINTS_TO_SKIP_FRONT;
  static const ParameterString<int> POINTS_TO_SKIP_BACK;
  static const ParameterString<double> EPSILON_X_CLASSIFIER;
  static const ParameterString<int> MIN_NUMBER_HITS;
  static const ParameterString<double> FREE_MIN_DEPTH_TH;
  static const ParameterString<double> SMALLEST_OBSTACLE_AREA;

  // Camera Parameters
  static const ParameterString<double> CAMERA_FOCAL_LENGTH_X;
  static const ParameterString<double> CAMERA_FOCAL_LENGTH_Y;
  static const ParameterString<double> CAMERA_OPTICAL_CENTER_X;
  static const ParameterString<double> CAMERA_OPTICAL_CENTER_Y;

  static const ParameterString<double> CAMERA_R11;
  static const ParameterString<double> CAMERA_R12;
  static const ParameterString<double> CAMERA_R13;
  static const ParameterString<double> CAMERA_R21;
  static const ParameterString<double> CAMERA_R22;
  static const ParameterString<double> CAMERA_R23;
  static const ParameterString<double> CAMERA_R31;
  static const ParameterString<double> CAMERA_R32;
  static const ParameterString<double> CAMERA_R33;

  static const ParameterString<double> CAMERA_T1;
  static const ParameterString<double> CAMERA_T2;
  static const ParameterString<double> CAMERA_T3;

  // World coorfinate helper parameters
  static const ParameterString<double> MAX_TF_LOOKUP_DURATION;
};

// ParkingLot Parameters
const std::string MyDummyParamHandler::NAMESPACE(std::string("parking_lot"));

const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_DEPTH(
    "parking_spot_depth");
const ParameterString<double> MyDummyParamHandler::PARKING_SPOT_WIDTH(
    "parking_spot_width");
const ParameterString<double> MyDummyParamHandler::MARKING_WIDTH(
    "marking_width");

const ParameterString<int> MyDummyParamHandler::STEP_REFERENCE_FUNCTION_LENGTH(
    NAMESPACE + "/step_reference_function_length");
const ParameterString<double> MyDummyParamHandler::GRADIENT_DETECTION_THLD(
    NAMESPACE + "/gradient_detection_thld");

const ParameterString<int> MyDummyParamHandler::IMAGE_PADDING(NAMESPACE +
                                                              "/image_padding");

const ParameterString<double> MyDummyParamHandler::MARKING_SCAN_STEP_WIDTH(
    NAMESPACE + "/marking_scan_step_width");
;
const ParameterString<double> MyDummyParamHandler::MARKING_SCAN_LINE_WIDTH(
    NAMESPACE + "/marking_scan_line_width");
;
const ParameterString<double> MyDummyParamHandler::MARKKING_SCAN_LINE_PADDING_FRONT(
    NAMESPACE + "/marking_scan_line_padding_front");
const ParameterString<double> MyDummyParamHandler::MARKKING_SCAN_LINE_PADDING_BACK(
    NAMESPACE + "/marking_scan_line_padding_back");

const ParameterString<int> MyDummyParamHandler::N_SCAN_LINES_PER_SPOT(
    NAMESPACE + "/n_scan_lines_per_spot");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_PADDING_X(
    "spot_scan_line_padding_x");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_PADDING_Y(
    NAMESPACE + "/spot_scan_line_padding_y");
const ParameterString<double> MyDummyParamHandler::SPOT_SCAN_LINE_OBSTACLE_SAFETY_MARGIN(
    NAMESPACE + "/spot_scan_line_obstacle_safety_margin");

const ParameterString<int> MyDummyParamHandler::N_MIN_MARKINGS(
    NAMESPACE + "/n_min_markings");
const ParameterString<double> MyDummyParamHandler::MAX_SPAWN_DISTANCE(
    NAMESPACE + "/max_spawn_distance");
const ParameterString<double> MyDummyParamHandler::MAX_MARKING_UPDATE_DISTANCE(
    NAMESPACE + "/max_marking_update_distance");
const ParameterString<double> MyDummyParamHandler::MARKING_UPDATE_SHIFT_FACTOR(
    NAMESPACE + "/marking_update_shift_factor");
const ParameterString<double> MyDummyParamHandler::START_POSE_MAX_ALLOWED_DEVIATION(
    NAMESPACE + "/start_pose_max_allowed_deviation");

const ParameterString<double> MyDummyParamHandler::NEWTON_EPSILON(
    NAMESPACE + "/newton_epsilon");
const ParameterString<int> MyDummyParamHandler::NEWTON_MAX_ITERATIONS(
    NAMESPACE + "/newton_max_iterations");

// ParkingSpot Class Parameter
const ParameterString<int> MyDummyParamHandler::POINTS_TO_SKIP_FRONT(
    "parking_spot/points_to_skip_front");
const ParameterString<int> MyDummyParamHandler::POINTS_TO_SKIP_BACK(
    "parking_spot/points_to_skip_back");
const ParameterString<double> MyDummyParamHandler::EPSILON_X_CLASSIFIER(
    "parking_spot/epsilon_x_classifier");
const ParameterString<int> MyDummyParamHandler::MIN_NUMBER_HITS(
    "parking_spot/min_number_hits");
const ParameterString<double> MyDummyParamHandler::SMALLEST_OBSTACLE_AREA(
    "parking_spot/smallest_obstacle_area");
const ParameterString<double> MyDummyParamHandler::FREE_MIN_DEPTH_TH(
    "parking_spot/free_min_depth_th");

const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_TOP(
    "parking_lot/field_of_vision/top");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_BOTTOM(
    "parking_lot/field_of_vision/bottom");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_LEFT(
    "parking_lot/field_of_vision/left");
const ParameterString<int> MyDummyParamHandler::FIELD_OF_VISION_RIGHT(
    "parking_lot/field_of_vision/right");

// Camera Parameters
const ParameterString<double> MyDummyParamHandler::CAMERA_FOCAL_LENGTH_X(
    "/camera/focal_length_x");
const ParameterString<double> MyDummyParamHandler::CAMERA_FOCAL_LENGTH_Y(
    "/camera/focal_length_y");
const ParameterString<double> MyDummyParamHandler::CAMERA_OPTICAL_CENTER_X(
    "/camera/optical_center_x");
const ParameterString<double> MyDummyParamHandler::CAMERA_OPTICAL_CENTER_Y(
    "/camera/optical_center_y");

const ParameterString<double> MyDummyParamHandler::CAMERA_R11("/camera/r11");
const ParameterString<double> MyDummyParamHandler::CAMERA_R12("/camera/r12");
const ParameterString<double> MyDummyParamHandler::CAMERA_R13("/camera/r13");
const ParameterString<double> MyDummyParamHandler::CAMERA_R21("/camera/r21");
const ParameterString<double> MyDummyParamHandler::CAMERA_R22("/camera/r22");
const ParameterString<double> MyDummyParamHandler::CAMERA_R23("/camera/r23");
const ParameterString<double> MyDummyParamHandler::CAMERA_R31("/camera/r31");
const ParameterString<double> MyDummyParamHandler::CAMERA_R32("/camera/r32");
const ParameterString<double> MyDummyParamHandler::CAMERA_R33("/camera/r33");

const ParameterString<double> MyDummyParamHandler::CAMERA_T1("/camera/t1");
const ParameterString<double> MyDummyParamHandler::CAMERA_T2("/camera/t2");
const ParameterString<double> MyDummyParamHandler::CAMERA_T3("/camera/t3");

// coordinate helper
const ParameterString<double> MyDummyParamHandler::MAX_TF_LOOKUP_DURATION(
    "max_tf_lookup_duration");

class WorldCoordinatesTFHelperDummy : public tf_helper::TFHelperInterface<double> {
 public:
  virtual bool update(const ros::Time&) override { return true; }

  virtual AffineTransformation getTransform() const override {
    return Eigen::Affine3d::Identity();
  }
};



/*!
 * \brief The ParkingLotTest class
 * TestFixure for the gtest. All needed objects are initialzed here.
 */
class ParkingLotTest : public ::testing::Test {
 public:
  ParkingLotTest()
      : param_(),
        camera_transformation_(&param_),
        parking_lot_(&param_, &camera_transformation_, &world_coordinates_helper_) {

    // initialize transformations
    parkinglot_to_vehicle_transform_ = Eigen::Affine3d::Identity();
    // Note: do rotation first translation always in new coordinates
    Eigen::AngleAxisd angle_axis =
        Eigen::AngleAxisd((2 * M_PI - 0.42 * M_PI), Eigen::Vector3d::UnitZ());
    parkinglot_to_vehicle_transform_.rotate(angle_axis);
    parkinglot_to_vehicle_transform_ =
        Eigen::Translation3d(0.45, 0.7, 0.0) * parkinglot_to_vehicle_transform_;

    parkinglot_to_world_transform_ =
        world_coordinates_helper_.getTransform() * parkinglot_to_vehicle_transform_;
  }

  /*!
   * \brief getLeftLanePolynom
   * \param parking_lot_generator a ParkingLotGenerator object
   * \param parkinglot_to_vehicle_transform an Affine3D transformation from
   * parkinglot to vehicle corrdinates
   * \return LeftLanePolynom Polynom of first order describing the left line in
   * vehicle coordinates
   */
  common::DynamicPolynomial getLeftLanePolynom(const ParkingLotGenerator& parking_lot_generator,
                                               const Eigen::Affine3d& parkinglot_to_vehicle_transform) const {
    ParkingLotPoints points;
    const int number_of_left_lane_points = 5;
    parking_lot_generator.getLeftLanePoints(points, number_of_left_lane_points);
    VehiclePoints vps;
    vps.reserve(number_of_left_lane_points);
    const auto parkingLotPointToVehiclePoint =
        [&parkinglot_to_vehicle_transform](const auto& parking_lot_point) {
          return (parkinglot_to_vehicle_transform * parking_lot_point);
        };
    boost::range::transform(points, std::back_inserter(vps), parkingLotPointToVehiclePoint);

    return common::fitToPoints(vps, POLYNOMIAL_DEGREE_LINE);
  }

  /*!
   * \brief occupationStateToString
   * \param state the state to convert.
   * \return a string represenatation of the occupation state
   */
  std::string occupationStateToString(const OccupationState& state) const {
    switch (state) {
      case OccupationState::FREE:
        return "free    ";
      case OccupationState::OCCUPIED:
        return "occupied";
      case OccupationState::X:
        return "X       ";
      case OccupationState::UNKNOWN:
        return "unknown ";
      default:
        return "invalid ";
    }
  }

  /*!
   * \brief generatedOccupationStateToString
   * \param state the state to convert.
   * \return  a string represenatation of the occupation state
   */
  std::string generatedOccupationStateToString(const GeneratedOccupationState& state) const {
    switch (state) {
      case GeneratedOccupationState::FREE:
        return "free           ";
      case GeneratedOccupationState::X:
        return "X              ";
      case GeneratedOccupationState::OBSTACLE_LARGE:
        return "obstacle_large ";
      case GeneratedOccupationState::OBSTACLE_MEDIUM:
        return "obstacle_medium";
      case GeneratedOccupationState::OBSTACLE_SMALL:
        return "obstacle_small ";
      default:
        return "invalid        ";
    }
  }

  void saveDebugImage(const cv::Mat& img,
                      const ParkingLotGenerator& parking_lot_generator,
                      const WorldPose& parking_start_pose,
                      const WorldPose& parking_stop_pose,
                      const common::DynamicPolynomial& /*left_lane_polynomial*/,
                      const std::string& filename) {
    // convert in colored image
    cv::Mat debug_img(650, 1280, CV_8UC3);
    cv::cvtColor(img, debug_img, 8);

    VehiclePose parking_start_pose_vehicle =
        world_coordinates_helper_.getTransform().inverse() * parking_start_pose;
    VehiclePose parking_stop_pose_vehicle =
        world_coordinates_helper_.getTransform().inverse() * parking_stop_pose;

    parking_lot_generator.debugDrawGeneratedPoints(debug_img,
                                                   camera_transformation_,
                                                   parkinglot_to_vehicle_transform_,
                                                   cv::Scalar(50, 50, 255));
    // parking_lot_generator.debug_drawPolynom(debug_img, left_lane_polynomial,
    // camera_transformation_, cv::Scalar(50,50,255));
    parking_lot_generator.debugDrawPose(
        debug_img, parking_start_pose_vehicle, camera_transformation_, cv::Scalar(0, 140, 255), 3);
    parking_lot_generator.debugDrawPose(
        debug_img, parking_stop_pose_vehicle, camera_transformation_, cv::Scalar(20, 255, 50), 3);

    // save image
    bool img_saved = cv::imwrite(filename, debug_img);
    if (img_saved) {
      std::cout << "\n image: " << filename << " saved. " << std::endl;
    }
  }

  /*!
   * \brief isCorrect checks if the detected occupation state is correct given
   * the generted occupation state.
   * \param generated_state the expected state.
   * \param detected_state the detected state.
   * \return whether  the generated state is consistent with the detected state.
   */
  bool isCorrect(const GeneratedOccupationState& generated_state,
                 const OccupationState& detected_state) const {
    switch (detected_state) {
      case OccupationState::FREE:
        if (generated_state == GeneratedOccupationState::FREE) {
          return true;
        } else {
          return false;
        }
      case OccupationState::OCCUPIED:
        if (generated_state == GeneratedOccupationState::OBSTACLE_LARGE ||
            generated_state == GeneratedOccupationState::OBSTACLE_MEDIUM ||
            generated_state == GeneratedOccupationState::OBSTACLE_SMALL) {
          return true;
        } else {
          return false;
        }
      case OccupationState::X:
        if (generated_state == GeneratedOccupationState::X) {
          return true;
        } else {
          return false;
        }
      case OccupationState::UNKNOWN:
        return false;
      default:
        return false;
    }
  }

  /*!
   * \brief runParkingLotTest : initializes a parkinglotGenerator object with
   * the specified paramters and runs
   * the update method of the parking lot class and compares results
   * \param generated_occupation_states the generated states.
   * \param parking_start_line_angle_in_radian the start line angle.
   * \param parking_slot_width_in_m the width of the slot.
   * \param parking_slot_depth_in_m the depth of the slot.
   */
  void runParkingLotTest(const std::vector<GeneratedOccupationState>& generated_occupation_states,
                         const double& parking_start_line_angle_in_radian,
                         const double& parking_slot_width_in_m,
                         const double& parking_slot_depth_in_m,
                         const int /*test_number = 0*/) {

    const ParkingLotGenerator parking_lot_generator(generated_occupation_states,
                                                    parking_start_line_angle_in_radian,
                                                    parking_slot_width_in_m,
                                                    parking_slot_depth_in_m);
    // get parking_start_pose
    const WorldPose parking_start_pose =
        parkinglot_to_world_transform_ * parking_lot_generator.getParkingStartPose();
    // get parking_end_pose
    const WorldPose parking_stop_pose =
        parkinglot_to_world_transform_ * parking_lot_generator.getParkingEndPose();

    // gete left lane polynom in vehicle coordinates
    const common::DynamicPolynomial left_lane_polynomial =
        getLeftLanePolynom(parking_lot_generator, parkinglot_to_vehicle_transform_);
    // generate image
    cv::Mat img;
    parking_lot_generator.drawParkingLot(
        img, parkinglot_to_vehicle_transform_, camera_transformation_);

    // set parking_start_pose
    parking_lot_.setStart(parking_start_pose);
    parking_lot_.setEnd(parking_stop_pose);

    // run update method N times and move vehicle
    for (int j = 0; j < 10; j++) {
      parking_lot_.update(img, left_lane_polynomial);
      const auto all = parking_lot_.allParkingSpots();
      // get occupation states
      std::vector<OccupationState> occupation_states;
      occupation_states.reserve(all.size());
      for (auto& parking_spot : all) {
        occupation_states.emplace_back(parking_spot.get().occupationState());
      }
    }

    // get results of parking lot class
    const auto all_spots = parking_lot_.allParkingSpots();

    // get occupation states
    std::vector<OccupationState> occupation_states;
    occupation_states.reserve(all_spots.size());
    for (auto& parking_spot : all_spots) {
      occupation_states.emplace_back(parking_spot.get().occupationState());
    }

    // compare number of detected spots to number of created spots
    std::cout << "Generated spots (total): " << generated_occupation_states.size()
              << std::endl;
    std::cout << "Detected Spots  (total): " << all_spots.size() << std::endl;

    int differenceOfDetectedSpots =
        abs(static_cast<int>(occupation_states.size()) -
            static_cast<int>(generated_occupation_states.size()));
    // throw error if not the corrected number of spots are detected
    // Note: it might occur that one additional spot marked as "X" ore
    // "Occupied"
    // is detected at the end of the spot. So this extra spot is not marked as
    // error!
    EXPECT_LE(differenceOfDetectedSpots, 1);

    // print detected spots compared to generated spots
    std::size_t M = (generated_occupation_states.size() < occupation_states.size())
                        ? occupation_states.size()
                        : generated_occupation_states.size();

    for (std::size_t i = 0; i < M; i++) {
      std::string generated = "               ";
      std::string detected = "        ";
      if (generated_occupation_states.size() > i) {
        generated = generatedOccupationStateToString(generated_occupation_states[i]);
      }
      if (occupation_states.size() > i) {
        detected = occupationStateToString(occupation_states[i]);
      }
      std::cout << "generated: " << generated << " | detected: " << detected << std::endl;
    }

    // Throw error if there was sth wrong
    for (std::size_t i = 0; i < occupation_states.size(); i++) {
      if (i < generated_occupation_states.size()) {
        EXPECT_TRUE(isCorrect(generated_occupation_states[i], occupation_states[i]));
      }
      // Note:
      // if there is one extra spot detected at the end of the lot that neither
      // marked as "free" or "unknwon"
      // this is expected behaviour and will not be marked as error.
      else {  // There are more spots detected than there have been created
        EXPECT_FALSE(occupation_states[i] == OccupationState::FREE ||
                     occupation_states[i] == OccupationState::UNKNOWN);
      }
    }

    // save debug image
    // NOTE: This must not run in the actul test!
    //            if(test_number > 0){
    //              std::stringstream ss;
    //              ss <<"IMG_test"<< test_number << ".jpg";
    //              saveDebugImage(img,
    //              parking_lot_generator,parking_start_pose,
    //              parking_stop_pose,
    //                             left_lane_polynomial,
    //                             ss.str());
    //            }
  }

  // Constants
  static const int POLYNOMIAL_DEGREE_LINE = 1;

 private:
  // Member variables
  MyDummyParamHandler param_;
  common::CameraTransformation camera_transformation_;
  WorldCoordinatesTFHelperDummy world_coordinates_helper_;
  perpendicular_parking::ParkingLot parking_lot_;

  // Transformations
  Eigen::Affine3d parkinglot_to_vehicle_transform_;
  Eigen::Affine3d parkinglot_to_world_transform_;
};

TEST_F(ParkingLotTest, test1_3_free_spots) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::FREE, GeneratedOccupationState::FREE};
  const double parking_start_line_angle_in_radian = common::toRad(60.0);
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    1);
}

TEST_F(ParkingLotTest, test2_different_occupation_states) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE,
      GeneratedOccupationState::X,
      GeneratedOccupationState::OBSTACLE_SMALL};
  const double parking_start_line_angle_in_radian = common::toRad(60.0);
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    2);
}

TEST_F(ParkingLotTest, test3_differentStartAngle) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::FREE, GeneratedOccupationState::FREE};
  const double parking_start_line_angle_in_radian = common::toRad(45.0);
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    3);
}

TEST_F(ParkingLotTest, test4_widerSpotSize) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::FREE, GeneratedOccupationState::FREE, GeneratedOccupationState::FREE};
  const double parking_start_line_angle_in_radian = common::toRad(60.0);
  const double parking_slot_width_in_m = 0.38;  // nur zweischen 31cm und 39 cm
  const double parking_slot_depth_in_m = 0.5;   // fixed size

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    4);
}

TEST_F(ParkingLotTest, test5_noFreeSpot) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::OBSTACLE_SMALL,
      GeneratedOccupationState::X,
      GeneratedOccupationState::OBSTACLE_MEDIUM};
  const double parking_start_line_angle_in_radian = common::toRad(60.0);
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    5);
}

TEST_F(ParkingLotTest, test6_ManySpots) {
  const std::vector<GeneratedOccupationState> generated_occupation_states = {
      GeneratedOccupationState::OBSTACLE_SMALL,
      GeneratedOccupationState::X,
      GeneratedOccupationState::FREE,
      GeneratedOccupationState::FREE,
      GeneratedOccupationState::OBSTACLE_MEDIUM};
  const double parking_start_line_angle_in_radian = common::toRad(60.0);
  const double parking_slot_width_in_m = 0.350;
  const double parking_slot_depth_in_m = 0.500;

  // seems to fail if iterrations of update() method calls are increased

  runParkingLotTest(generated_occupation_states,
                    parking_start_line_angle_in_radian,
                    parking_slot_width_in_m,
                    parking_slot_depth_in_m,
                    6);
}

}  // namespace testing
}  // namespace perpendicular_parking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
