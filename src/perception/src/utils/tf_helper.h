#ifndef TF_HELPER_H
#define TF_HELPER_H

#include "tf_helper_interface.h"
#include <common/parameter_interface.h>

namespace tf2_ros {
// forward declaration
class Buffer;
}

namespace tf_helper {
//!
//! \brief The FrameIDs struct indicates child
//!  and base frame for the transformation which is applied;
//! Usage: initialize a const FrameID-object before initializing TFHelper
//! and use it in TFHelper c'tor
//!
struct FrameIDs {
  //!
  //! \brief child_id id of child frame, i.e the
  //! target frame of the transformation
  //!
  std::string child_id;
  //!
  //! \brief base_id id of the base frame, i.e. the
  //! source frame of the transformation
  //!
  std::string base_id;
};

template <typename Scalar>
class TFHelper : public TFHelperInterface<Scalar> {
 public:
  //!
  //! \brief TFHelper looks up tf_tree for available transformation from base
  //! frame to child frame
  //! \param tf2_buffer the tf buffer.
  //! \param parameters_ptr the parameter-interface.
  //! \param frame_ids the frames to lookup.
  //!
  TFHelper(const tf2_ros::Buffer *const tf2_buffer,
           ParameterInterface *const parameters_ptr,
           const FrameIDs &frame_ids);

  TFHelper() = default;

  //! \brief update Updates internal transforms
  //! \param time_stamp time stamp for lookup
  //! \return false if lookup fails. otherwise true
  bool update(const ros::Time &time_stamp) override;
  //!
  //! \brief getTransform yields the desired transform from base to child frame
  //! \return Affine transformation as eigen data type
  //!
  typename TFHelperInterface<Scalar>::AffineTransformation getTransform() const override;

 private:
  bool update(const std::string &target_frame,
              const std::string &source_frame,
              const ros::Time &time_stamp,
              typename TFHelperInterface<Scalar>::AffineTransformation *base_to_child_transform);

  const tf2_ros::Buffer *const tf2_buffer_;
  const ParameterInterface *const parameters_ptr_;
  FrameIDs frame_ids_;

  typename TFHelperInterface<Scalar>::AffineTransformation base_to_child_transformation_;
};

}  // namespace tf_helper

#include "tf_helper.hpp"

#endif  // TF_HELPER_H
