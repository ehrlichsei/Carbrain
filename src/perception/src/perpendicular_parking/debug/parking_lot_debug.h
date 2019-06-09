#ifndef PARKING_LOT_DEBUG_H
#define PARKING_LOT_DEBUG_H

#include "../parking_lot.h"
namespace perpendicular_parking {
class ParkingLotDebug : public ParkingLot {
 public:
  ParkingLotDebug(ParkingLot &&parking_lot);

  void update(const cv::Mat &img, const common::DynamicPolynomial &left_lane_polynomial) override {
    // make a copy for e.g. getMarkingDetections
    current_img_ = img.clone();
    current_polynomial_ = left_lane_polynomial;

    ParkingLot::update(img, left_lane_polynomial);
  }

  ParkingSpotsConstRef freeParkingSpots() override;

  ParkingSpotsConstRef getAllParkingSpots() const;

  ScanLines getMarkingScanLines();
  ImagePoints getMarkingDetections();
  ScanLines getSpotScanLines();
  ScanLines getSpotLines() const;

  ImagePoints getFeaturePoints() const;

  virtual void reset() override;

 private:
  //  virtual std::vector<std::pair<double, ImagePointExact>> computeAngleField(
  //      const common::polynomial::DynamicPolynomial &left_lane_polynomial) const override;

  cv::Mat current_img_;
  common::DynamicPolynomial current_polynomial_;
  //  mutable ScanLines verticals;
};
}  // namespace perpendicular_parking

#endif  // PARKING_LOT_DEBUG_H
