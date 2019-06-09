#ifndef TF_HELPER_INTERFACE_H
#define TF_HELPER_INTERFACE_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <Eigen/Geometry>
#include <ros/time.h>
THIRD_PARTY_HEADERS_END

namespace tf_helper {



template <typename Scalar>
class TFHelperInterface {
 public:
  using AffineTransformation = Eigen::Transform<Scalar, 3, Eigen::Affine>;
  virtual ~TFHelperInterface() = default;
  //! \brief update Updates internal transforms
  //! \param time_stamp time stamp for lookup
  //! \return false if lookup fails. otherwise true
  virtual bool update(const ros::Time &time_stamp) = 0;

  virtual AffineTransformation getTransform() const = 0;
};

}  // namespace perpendicular_parking

#endif  // TF_HELPER_INTERFACE_H
