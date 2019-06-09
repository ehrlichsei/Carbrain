#include "perpendicular_parking_debug.h"

#include "parking_end_classifier_debug.h"
#include "parking_lot_debug.h"
#include "parking_start_classifier_debug.h"

#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include "common/console_colors.h"
#include "common/discretisize.h"

namespace perpendicular_parking {
PerpendicularParkingDebug::PerpendicularParkingDebug(PerpendicularParking &&perpendicular_parking)
    : PerpendicularParking(std::move(perpendicular_parking)) {
  parking_lot = std::make_unique<ParkingLotDebug>(std::move(*parking_lot));
  parking_start_classifier =
      std::make_unique<ParkingStartClassifierDebug>(std::move(*parking_start_classifier));
  parking_end_classifier =
      std::make_unique<ParkingEndClassifierDebug>(std::move(*parking_end_classifier));

  activated_ = true;
}

ParkingSpotsConstRef PerpendicularParkingDebug::findFreeParkingSpots(
    const cv::Mat &img, const LineVehiclePoints &lanes, const ros::Time &timestamp) {
  return PerpendicularParking::findFreeParkingSpots(img, lanes, timestamp);
}

ScanLines PerpendicularParkingDebug::createScanLines(VehiclePoints &left_lane) {
  debug_scan_lines = PerpendicularParking::createScanLines(left_lane);
  debug_sc_points.clear();
  boost::transform(debug_scan_lines,
                   std::back_inserter(debug_sc_points),
                   [](const auto &sl) { return sl.start; });
  return debug_scan_lines;
}

ImagePoints PerpendicularParkingDebug::apply1DGradientDetector(const cv::Mat &img,
                                                               const ScanLines &scan_lines) {
  debug_feature_points = PerpendicularParking::apply1DGradientDetector(img, scan_lines);
  return debug_feature_points;
}

ParkingSpotsConstRef PerpendicularParkingDebug::allParkingSpots() const {
  return parking_lot->allParkingSpots();
}

ScanLines PerpendicularParkingDebug::parkingLotMarkingScanLines() const {
  auto parking_lot_debug = dynamic_cast<ParkingLotDebug *>(parking_lot.get());
  assert(parking_lot_debug && "Parking Lot has to be of Debug type!");
  return parking_lot_debug->getMarkingScanLines();
}

ScanLines PerpendicularParkingDebug::parkingSlotsScanLines() const {
  auto parking_lot_debug = dynamic_cast<ParkingLotDebug *>(parking_lot.get());
  assert(parking_lot_debug && "Parking Lot has to be of Debug type!");
  return parking_lot_debug->getSpotScanLines();
}

ImagePoints PerpendicularParkingDebug::parkingStartCrucialPoints() const {
  auto parking_start_classifier_debug =
      dynamic_cast<ParkingStartClassifierDebug *>(parking_start_classifier.get());
  assert(parking_start_classifier_debug &&
         "Parking Start Classifier has to be of Debug type!");
  ImagePoints crucial_points;
  if (parking_lot->startDetected()) {
    ImagePoints start_line_points = parking_start_classifier_debug->crucialPoints();
    boost::push_back(crucial_points, start_line_points);
  }
  return crucial_points;
}

ImagePoints PerpendicularParkingDebug::markingDetections() const {
  auto parking_lot_debug = dynamic_cast<ParkingLotDebug *>(parking_lot.get());
  assert(parking_lot_debug && "Parking Lot has to be of Debug type!");
  return parking_lot_debug->getMarkingDetections();
}

ImagePoints PerpendicularParkingDebug::endLinesPoints() const {
  auto parking_end_debug =
      dynamic_cast<ParkingEndClassifierDebug *>(parking_end_classifier.get());
  assert(parking_end_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_end_debug->debugEndLinePoints();
}

ImagePoints PerpendicularParkingDebug::parkingEndFeaturePoints() const {
  auto parking_end_debug =
      dynamic_cast<ParkingEndClassifierDebug *>(parking_end_classifier.get());
  assert(parking_end_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_end_debug->debugFeaturePoints();
}

ScanLines PerpendicularParkingDebug::parkingEndScanLines() const {
  auto parking_end_debug =
      dynamic_cast<ParkingEndClassifierDebug *>(parking_end_classifier.get());
  assert(parking_end_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_end_debug->debugScanLines();
}

ScanLines PerpendicularParkingDebug::parkingSpotLines() const {
  auto parking_lot_debug = dynamic_cast<ParkingLotDebug *>(parking_lot.get());
  assert(parking_lot_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_lot_debug->getSpotLines();
}

ImagePoints PerpendicularParkingDebug::parkingEndCrucialPoints() const {
  auto parking_end_debug =
      dynamic_cast<ParkingEndClassifierDebug *>(parking_end_classifier.get());
  assert(parking_end_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_end_debug->debugCrucialPoints();
}

FeaturePointClusters PerpendicularParkingDebug::parkingEndClusters() const {
  const auto parking_end_classifier_debug =
      dynamic_cast<ParkingEndClassifierDebug *>(parking_end_classifier.get());
  assert(parking_end_classifier_debug &&
         "Parking End Classifier has to be of Debug type!");
  return parking_end_classifier_debug->debugEndlineClusters();
}

ImagePoints PerpendicularParkingDebug::leftLanePolynom() const {
  common::DiscretizationParams params{0.4, 2, 0.02};
  VehiclePoints discretized_vehicle;
  common::discretisize(left_lane_polynom_, params, &discretized_vehicle);
  ImagePoints discretized_img;
  camera_transform_->transformGroundToImage(discretized_vehicle, &discretized_img);
  return discretized_img;
}

ScanLines PerpendicularParkingDebug::parkingStartScanLines() const {
  auto parking_start_classifier_debug =
      dynamic_cast<ParkingStartClassifierDebug *>(parking_start_classifier.get());
  assert(parking_start_classifier_debug &&
         "Parking Start Classifier has to be of Debug type!");
  return parking_start_classifier_debug->debugScanLines();
  //  return debug_scan_lines;
}

ImagePoints PerpendicularParkingDebug::parkingLotFeaturePoints() const {
  auto parking_lot_debug = dynamic_cast<ParkingLotDebug *>(parking_lot.get());
  assert(parking_lot_debug && "Parking Lot has to be of Debug type!");
  return parking_lot_debug->getFeaturePoints();
}

ImagePoints PerpendicularParkingDebug::parkingStartFeaturePoints() const {
  return debug_feature_points;
}

ImagePoints PerpendicularParkingDebug::startLinePoints() const {
  auto parking_start_classifier_debug =
      dynamic_cast<ParkingStartClassifierDebug *>(parking_start_classifier.get());
  assert(parking_start_classifier_debug &&
         "Parking start has to be of Debug type!");
  return parking_start_classifier_debug->startLine();
}

FeaturePointClusters PerpendicularParkingDebug::startLinePointsClusters() const {
  auto parking_start_classifier_debug =
      dynamic_cast<ParkingStartClassifierDebug *>(parking_start_classifier.get());
  assert(parking_start_classifier_debug &&
         "Parking start has to be of Debug type!");
  return parking_start_classifier_debug->clusters();
}
}  // namespace perpendicular_parking
