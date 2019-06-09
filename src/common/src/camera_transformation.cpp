#include "common/camera_transformation.h"
#include "common/camera_transformation_parameters.h"
#include "common/eigen_utils.h"

using namespace Eigen;

namespace common {

CameraTransformation::CameraTransformation(ParameterInterface* parameters_ptr)
    : parameters_ptr(parameters_ptr) {

  parameters_ptr->registerParam(CAMERA_OPTICAL_CENTER_X);
  parameters_ptr->registerParam(CAMERA_OPTICAL_CENTER_Y);
  parameters_ptr->registerParam(CAMERA_FOCAL_LENGTH_X);
  parameters_ptr->registerParam(CAMERA_FOCAL_LENGTH_Y);

  parameters_ptr->registerParam(CAMERA_R11);
  parameters_ptr->registerParam(CAMERA_R12);
  parameters_ptr->registerParam(CAMERA_R13);

  parameters_ptr->registerParam(CAMERA_R21);
  parameters_ptr->registerParam(CAMERA_R22);
  parameters_ptr->registerParam(CAMERA_R23);

  parameters_ptr->registerParam(CAMERA_R31);
  parameters_ptr->registerParam(CAMERA_R32);
  parameters_ptr->registerParam(CAMERA_R33);

  parameters_ptr->registerParam(CAMERA_T1);
  parameters_ptr->registerParam(CAMERA_T2);
  parameters_ptr->registerParam(CAMERA_T3);

  update();
}

void CameraTransformation::update() {
  const double fx = parameters_ptr->getParam(CAMERA_FOCAL_LENGTH_X);
  const double fy = parameters_ptr->getParam(CAMERA_FOCAL_LENGTH_Y);
  const double cx = parameters_ptr->getParam(CAMERA_OPTICAL_CENTER_X);
  const double cy = parameters_ptr->getParam(CAMERA_OPTICAL_CENTER_Y);

  const double r11 = parameters_ptr->getParam(CAMERA_R11);
  const double r12 = parameters_ptr->getParam(CAMERA_R12);
  const double r13 = parameters_ptr->getParam(CAMERA_R13);

  const double r21 = parameters_ptr->getParam(CAMERA_R21);
  const double r22 = parameters_ptr->getParam(CAMERA_R22);
  const double r23 = parameters_ptr->getParam(CAMERA_R23);

  const double r31 = parameters_ptr->getParam(CAMERA_R31);
  const double r32 = parameters_ptr->getParam(CAMERA_R32);
  const double r33 = parameters_ptr->getParam(CAMERA_R33);

  const double t1 = parameters_ptr->getParam(CAMERA_T1);
  const double t2 = parameters_ptr->getParam(CAMERA_T2);
  const double t3 = parameters_ptr->getParam(CAMERA_T3);

  // clang-format off
  A << fx,  0, cx,
        0, fy, cy,
        0,  0,  1;

  R << r11, r12, r13,
       r21, r22, r23,
       r31, r32, r33;

  t << t1, t2, t3;
  // clang-format on
  t /= 1000.0;
  M << R, t;
  P = A * M;

  // see Multiple View Geometry by Hartley and Zisserman page 196
  Matrix3d r1_r2_t;
  r1_r2_t << R.col(0), R.col(1), t;
  H = A * r1_r2_t;
  H_inverse = H.fullPivHouseholderQr().inverse();
}

Vector2d CameraTransformation::transformVehicleToImageExact(const Vector3d& world_point) const {
  const Vector3d image_point_homogenous = P * toHomogenous(world_point);
  return to2D(image_point_homogenous) / image_point_homogenous.z();
}

Vector2i CameraTransformation::transformVehicleToImage(const Vector3d& world_point) const {
  return round(transformVehicleToImageExact(world_point));
}

// only for points on the ground plane (ground_point(2)=0)
Vector2d CameraTransformation::transformGroundToImageExact(const Vector3d& ground_point) const {
  const Vector3d image_point_homogenous = H * toHomogenous(to2D(ground_point));
  return to2D(image_point_homogenous) / image_point_homogenous.z();
}

// only for points on the ground plane (ground_point(2)=0)
Vector2i CameraTransformation::transformGroundToImage(const Vector3d& ground_point) const {
  return round(transformGroundToImageExact(ground_point));
}

Vector3d CameraTransformation::transformImageToGround(const Vector2i& image_point) const {
  const Vector3d ground_point_homogenous =
      H_inverse * toHomogenous(Vector2d(image_point.cast<double>()));
  return to3D(to2D(ground_point_homogenous) / ground_point_homogenous.z());
}

void CameraTransformation::transformVehicleToImage(const common::Vector3dVector& world_points,
                                                   common::Vector2iVector* image_points) const {
  image_points->reserve(image_points->size() + world_points.size());
  for (const auto& world_point : world_points) {
    image_points->push_back(transformVehicleToImage(world_point));
  }
}

// only for points on the ground plane (ground_point(2)=0)
void CameraTransformation::transformGroundToImage(const common::Vector3dVector& ground_points,
                                                  common::Vector2iVector* image_points) const {
  image_points->reserve(image_points->size() + ground_points.size());
  for (const auto& ground_point : ground_points) {
    image_points->push_back(transformGroundToImage(ground_point));
  }
}

void CameraTransformation::transformImageToGround(const common::Vector2iVector& image_points,
                                                  common::Vector3dVector* ground_points) const {
  ground_points->reserve(ground_points->size() + image_points.size());
  for (const auto& image_point : image_points) {
    ground_points->push_back(transformImageToGround(image_point));
  }
}

const Matrix3d& CameraTransformation::getIntrinsicCalibrationMatrix() const {
  return A;
}

const Matrix3d& CameraTransformation::getRotationMatrix() const { return R; }

const Vector3d& CameraTransformation::getTranslationVector() const { return t; }

const Matrix<double, 3, 4>& CameraTransformation::getProjectionMatrix() const {
  return P;
}

} // namespace common
