#include "roi_birds_view_transformation.h"

ROIBirdsViewTransformation::ROIBirdsViewTransformation(ParameterInterface* parameters_ptr)
    : CameraTransformation(parameters_ptr) {}

VehiclePoint ROIBirdsViewTransformation::transformROIToGround(const ImagePoint& roi_image_point) const {
  return transformImageToGround(roi_image_point + roi_offset);
}

void ROIBirdsViewTransformation::transformROIToGround(const ImagePoints& roi_image_points,
                                                      VehiclePoints* vehicle_points) const {
  vehicle_points->reserve(vehicle_points->size() + roi_image_points.size());
  for (const ImagePoint& roi_image_point : roi_image_points) {
    vehicle_points->push_back(transformROIToGround(roi_image_point));
  }
}

VehiclePoints ROIBirdsViewTransformation::transformROIToGround(const ImagePoints& roi_image_points) const {
  VehiclePoints vehicle_points;
  transformROIToGround(roi_image_points, &vehicle_points);
  return vehicle_points;
}


ImagePoint ROIBirdsViewTransformation::transformGroundToROI(const VehiclePoint& vehicle_point) const {
  return transformGroundToImage(vehicle_point) - roi_offset;
}

void ROIBirdsViewTransformation::transformGroundToROI(const VehiclePoints& vehicle_points,
                                                      ImagePoints* roi_image_points) const {
  roi_image_points->reserve(roi_image_points->size() + vehicle_points.size());
  for (const VehiclePoint& vehicle_point : vehicle_points) {
    roi_image_points->push_back(transformGroundToROI(vehicle_point));
  }
}

void ROIBirdsViewTransformation::reconfigure(const ImagePoint& roi_offset) {
  this->roi_offset = roi_offset;
}
