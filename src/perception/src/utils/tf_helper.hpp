#include "tf_helper.h"

THIRD_PARTY_HEADERS_BEGIN
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
THIRD_PARTY_HEADERS_END

namespace tf_helper {

const ParameterString<double> MAX_TF_LOOKUP_DURATION("max_tf_lookup_duration");


template <typename Scalar>
TFHelper<Scalar>::TFHelper(const tf2_ros::Buffer *const tf2_buffer,
                           ParameterInterface *const parameters_ptr,
                           const FrameIDs &frame_ids)
    : TFHelperInterface<Scalar>::TFHelperInterface(),
      tf2_buffer_(tf2_buffer),
      parameters_ptr_(parameters_ptr),
      frame_ids_(frame_ids) {
  parameters_ptr->registerParam(MAX_TF_LOOKUP_DURATION);
}

template <typename Scalar>
bool TFHelper<Scalar>::update(const ros::Time &time_stamp) {
  return update(frame_ids_.child_id, frame_ids_.base_id, time_stamp, &base_to_child_transformation_);
}

template <typename Scalar>
typename TFHelperInterface<Scalar>::AffineTransformation TFHelper<Scalar>::getTransform() const {
  return base_to_child_transformation_;
}

template <typename Scalar>
bool TFHelper<Scalar>::update(const std::string &target_frame,
                              const std::string &source_frame,
                              const ros::Time &time_stamp,
                              typename TFHelperInterface<Scalar>::AffineTransformation *base_to_child_transform) {

  const ros::Duration MAX_DURATION(parameters_ptr_->getParam(MAX_TF_LOOKUP_DURATION));

  geometry_msgs::TransformStamped transformation;

  try {
    transformation = tf2_buffer_->lookupTransform(
        target_frame, time_stamp, source_frame, time_stamp, "world", MAX_DURATION);
  } catch (tf2::TransformException &ex) {
    ROS_WARN(
        "Can NOT look up %s pose in %s : %s. Taking ros::Time(0) as "
        "timestamp for the lookup.",
        source_frame.c_str(),
        target_frame.c_str(),
        ex.what());
    try {
      transformation = tf2_buffer_->lookupTransform(
          target_frame, source_frame, ros::Time(0), ros::Duration(0));
    } catch (tf2::TransformException &ex_in) {
      ROS_ERROR("Can NOT look up %s pose in %s : %s.",
                source_frame.c_str(),
                target_frame.c_str(),
                ex_in.what());
      *base_to_child_transform =
          TFHelperInterface<Scalar>::AffineTransformation::Identity();
      return false;
    }
  }
  *base_to_child_transform = tf2::transformToEigen(transformation).cast<Scalar>();
  return true;
}


}  // namespace tf_helper
