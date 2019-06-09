#ifndef COMMON_CAMERA_TRANSFORMATION_H
#define COMMON_CAMERA_TRANSFORMATION_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "common/types.h"

namespace common {
/**
 * \brief CameraTransformation class for camera/vehicle coordinates.
 *
 * This class supports three transformations:
 * 1. image (u,v) -> vehicle (x,y,z=0) for points on the ground plane
 *(transformImageToGround)
 * 2. vehicle (x,y,z=0) -> image (u,v) for points on the ground plane
 *(transformGroundToImage)
 * 3. vehicle (x,y,z) -> image (u,v)   for arbitrary world points
 *(transformVehicleToImage)
 * (2. is a little bit faster than 3. - only significant if many thousand points
 *have to be transformed)
 */
class CameraTransformation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Constructor.
   *
   * \param parameters_ptr The intrinsic and extrinsic parameters of the camera.
   */
  CameraTransformation(ParameterInterface* parameters_ptr);

  void update();

  Eigen::Vector2d transformVehicleToImageExact(const Eigen::Vector3d& world_point) const;

  Eigen::Vector2i transformVehicleToImage(const Eigen::Vector3d& world_point) const;

  // only for points on the ground plane (z=0)
  Eigen::Vector2d transformGroundToImageExact(const Eigen::Vector3d& ground_point) const;

  // only for points on the ground plane (z=0)
  Eigen::Vector2i transformGroundToImage(const Eigen::Vector3d& ground_point) const;

  Eigen::Vector3d transformImageToGround(const Eigen::Vector2i& image_point) const;

  void transformVehicleToImage(const common::Vector3dVector& world_points,
                               common::Vector2iVector* image_points) const;

  // only for points on the ground plane (z=0)
  void transformGroundToImage(const common::Vector3dVector& ground_points,
                              common::Vector2iVector* image_points) const;

  void transformImageToGround(const common::Vector2iVector& image_points,
                              common::Vector3dVector* ground_points) const;

  const Eigen::Matrix3d& getIntrinsicCalibrationMatrix() const;

  const Eigen::Matrix3d& getRotationMatrix() const;

  /*!
   * \brief getTranslationVector
   * \return translation vector in camera frame
   */
  const Eigen::Vector3d& getTranslationVector() const;

  const Eigen::Matrix<double, 3, 4>& getProjectionMatrix() const;

 private:
  /*!
   * \brief A the instrinsic calibration matrix
   */
  Eigen::Matrix3d A;
  /*!
   * \brief R the rotation matrix from vehicle to camera frame
   */
  Eigen::Matrix3d R;
  /*!
   * \brief t the translation vector from camera to vehicle frame
   */
  Eigen::Vector3d t;
  /*!
   * \brief M the affine extrinsic transformation from camera to vehicle frame
   * M=[R|t]
   */
  Eigen::Matrix<double, 3, 4> M;
  /*!
   * \brief P the projection matrix  P=AM=A[R|t]
   */
  Eigen::Matrix<double, 3, 4> P;
  /*!
   * \brief H the homography matrix world-plane to image
   */
  Eigen::Matrix3d H;
  /*!
   * \brief H_inverse the homography matrix image to world-plane
   */
  Eigen::Matrix3d H_inverse;

  const ParameterInterface* parameters_ptr;
};

}  // namespace common

#endif  // COMMON_CAMERA_TRANSFORMATION_H
