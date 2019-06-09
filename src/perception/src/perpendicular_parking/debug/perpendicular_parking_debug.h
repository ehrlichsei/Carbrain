#ifndef PERPENDICULAR_PARKING_DEBUG_H
#define PERPENDICULAR_PARKING_DEBUG_H

#include "../perpendicular_parking.h"

namespace perpendicular_parking {
class PerpendicularParkingDebug : public PerpendicularParking {
 public:
  PerpendicularParkingDebug(PerpendicularParking &&perpendicular_parking);

  ParkingSpotsConstRef findFreeParkingSpots(const cv::Mat &img,
                                            const LineVehiclePoints &lanes,
                                            const ros::Time &timestamp) override;

  ScanLines createScanLines(VehiclePoints &left_lane) override;

  ImagePoints apply1DGradientDetector(const cv::Mat &img, const ScanLines &scan_lines) override;

  ParkingSpotsConstRef allParkingSpots() const;

  ScanLines parkingStartScanLines() const;

  ImagePoints parkingLotFeaturePoints() const;

  ImagePoints parkingStartFeaturePoints() const;

  ImagePoints startLinePoints() const;

  FeaturePointClusters startLinePointsClusters() const;
  ScanLines parkingLotMarkingScanLines() const;
  ScanLines parkingSlotsScanLines() const;
  ImagePoints parkingStartCrucialPoints() const;

  ImagePoints markingDetections() const;

  ImagePoints endLinesPoints() const;

  ImagePoints parkingEndFeaturePoints() const;

  ScanLines parkingEndScanLines() const;

  ScanLines parkingSpotLines() const;

  ImagePoints parkingEndCrucialPoints() const;

  FeaturePointClusters parkingEndClusters() const;

  ImagePoints leftLanePolynom() const;

 private:
  ScanLines debug_scan_lines;
  ImagePoints debug_feature_points, debug_sc_points;
};
}  // namespace perpendicular_parking

#endif  // PERPENDICULAR_PARKING_DEBUG_H
