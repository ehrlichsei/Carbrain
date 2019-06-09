#ifndef PERCEPTION_VEHICLE_SCAN_LINE_H
#define PERCEPTION_VEHICLE_SCAN_LINE_H

#include <common/camera_transformation.h>
#include "perception_types.h"
#include "scan_line.h"

/*!
 * \brief the VehicleScanLine specifies a scan lines in vehice coordinates.
 */
struct VehicleScanLine {
  VehicleScanLine() = default;
  VehicleScanLine(const VehiclePoint& start, const VehiclePoint& end)
      : start(start), end(end) {}
  /*!
   * \ brief start the start point of the scan line.
   */
  VehiclePoint start;
  /*!
   * \brief end the end point of the scan line.
   */
  VehiclePoint end;
};

/*!
 * \brief VehicleScanLines a vector of VehicleScanLine.
 */
using VehicleScanLines = std::vector<VehicleScanLine>;
/*!
 * \brief WorldScanLine a scan line in world coordinates.
 */
using WorldScanLine = VehicleScanLine;
/*!
 * \brief WorldScanLines a vector of WorldScanLines.
 */
using WorldScanLines = std::vector<WorldScanLine>;

/*!
 * \brief transfoms a VehicleScanLine.
 * \param t the transformation ot apply.
 * \param l the VehicleScanLine to transform.
 * \return the transformed VehicleScanLine.
 */
inline VehicleScanLine operator*(const Eigen::Affine3d& t, const VehicleScanLine& l) {
  return {t * l.start, t * l.end};
}

/*!
 * \brief transformGroundToImage transforms VehicleScanLine to a Scanline, only for points lying on the ground.
 * \param trafo the common::CameraTransformation.
 * \param l the VehicleScanLine to transform.
 * \return the resulting ScanLine.
 */
inline ScanLine transformGroundToImage(const common::CameraTransformation* const trafo,
                                       const VehicleScanLine& l) {
  return {trafo->transformGroundToImage(l.start), trafo->transformGroundToImage(l.end)};
}

/*!
 * \brief transformVehicleToImage transforms VehicleScanLine to a Scanline, for
 * arbitrary vehicle points. \param trafo the common::CameraTransformation.
 * \param l the VehicleScanLine to transform. \return the resulting ScanLine.
 */
inline ScanLine transformVehicleToImage(const common::CameraTransformation* const trafo,
                                        const VehicleScanLine& l) {
  return {trafo->transformGroundToImage(l.start), trafo->transformVehicleToImage(l.end)};
}

#endif  // PERCEPTION_VEHICLE_SCAN_LINE_H
