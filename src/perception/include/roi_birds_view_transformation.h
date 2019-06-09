#ifndef ROIBIRDSVIEWTRANSFORMATION_H
#define ROIBIRDSVIEWTRANSFORMATION_H

#include "common/camera_transformation.h"

#include "perception_types.h"

class ROIBirdsViewTransformation : public common::CameraTransformation {
 public:
  ROIBirdsViewTransformation(ParameterInterface* parameters_ptr);

  VehiclePoint transformROIToGround(const ImagePoint& roi_image_point) const;
  void transformROIToGround(const ImagePoints& roi_image_points,
                            VehiclePoints* vehicle_points) const;
  VehiclePoints transformROIToGround(const ImagePoints& roi_image_points) const;


  ImagePoint transformGroundToROI(const VehiclePoint& vehicle_point) const;
  void transformGroundToROI(const VehiclePoints& vehicle_points,
                            ImagePoints* roi_image_points) const;

  void reconfigure(const ImagePoint& roi_offset);

 private:
  ImagePoint roi_offset;
};

#endif  // ROIBIRDSVIEWTRANSFORMATION_H
