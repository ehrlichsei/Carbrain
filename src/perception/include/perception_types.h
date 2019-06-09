#ifndef KITCAR_PERCEPTION_TYPES_H
#define KITCAR_PERCEPTION_TYPES_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>
#include "common/types.h"

/**
 * Type definition for image points.
 *
 * uvec is 2-D column vector using unsigned integers.
 */
typedef Eigen::Vector2i ImagePoint;

/**
 * Type definition for image points.
 *
 * uvec is 2-D column vector using unsigned integers.
 */
typedef Eigen::Vector2d ImagePointExact;

/**
 * Type definiton for list of image points.
 */
typedef common::EigenAlignedVector<ImagePoint> ImagePoints;

/**
 * Type definiton for list of image points.
 */
typedef common::EigenAlignedVector<ImagePointExact> ImagePointsExact;

/**
 * Type definition for camera points.
 *
 * vec is a 2D column vector using doubles.
 */
typedef Eigen::Vector3d CameraPoint;

/**
 * Type definition for list of camera points.
 */
typedef common::EigenAlignedVector<CameraPoint> CameraPoints;

/**
 * Type definition for world points.
 *
 * vec is a 3D column vector using doubles.
 */
typedef Eigen::Vector3d WorldPoint;

/**
 * Type definition for world poses.
 *
 * Affine3d is an affine transformation stored as a (Dim+1)^2 matrix
 */
typedef Eigen::Affine3d WorldPose;

/**
 * Type definition for list of world points.
 */
typedef common::EigenAlignedVector<WorldPoint> WorldPoints;
/**
 * Type definition for vehicle points.
 *
 * vec is a 2D column vector using doubles.
 * \todo vehicle point should be 3D at some time.
 */
typedef Eigen::Vector3d VehiclePoint;

/**
 * Type definition for vehicle poses.
 *
 * Affine3d is an affine transformation stored as a (Dim+1)^2 matrix
 */
typedef Eigen::Affine3d VehiclePose;


/**
 * Type definition for list of vehicle points.
 */
typedef common::EigenAlignedVector<VehiclePoint> VehiclePoints;

/**
 * Enum type for indexing left, middle and right lanes in map
 * structures.
 */
enum LineSpec : std::size_t {
  LINESPEC_LEFT = 0,
  LINESPEC_MIDDLE,
  LINESPEC_RIGHT,
  LINESPEC_NO_PASSING,
  LINESPEC_N
};

/*!
 * \brief other return for outer lines the other line.
 * \param one the on line
 * \return the other line.
 */
inline LineSpec other(const LineSpec one) {
  switch (one) {
    case LINESPEC_LEFT:
      return LINESPEC_RIGHT;
    case LINESPEC_RIGHT:
      return LINESPEC_LEFT;
    case LINESPEC_MIDDLE:
    case LINESPEC_NO_PASSING:
    case LINESPEC_N:
      return one;
  }
  return one;
}

/**
 * Type definition of a list of 1D points in double.
 */
typedef typename std::vector<double> Points1D;

enum Vertex_Type { FRONT = 0 };

/*!
 * \brief toString
 *
 * Returns a String representling the given LineSpec.
 *
 * \param linespec the LineSpec
 * \return a String representling the given LineSpec.
 */
inline std::string toString(const LineSpec linespec) {
  switch (linespec) {
    case LINESPEC_LEFT:
      return "left";
    case LINESPEC_MIDDLE:
      return "middle";
    case LINESPEC_RIGHT:
      return "right";
    case LINESPEC_NO_PASSING:
      return "no_passing";
    case LINESPEC_N:
      return std::to_string(LINESPEC_N);
  }
  return "";
}

struct RegionToClassify {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Affine3d pose;

  double height;

  double width;

  std::size_t id;
};
using RegionsToClassify = std::vector<RegionToClassify>;


#endif /* KITCAR_PERCEPTION_TYPES_H */
