#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>

#include <perception_msgs/ArrowMarkings.h>
#include <perception_msgs/Crosswalks.h>
#include <perception_msgs/Junctions.h>
#include <perception_msgs/NoPassingZones.h>
#include <perception_msgs/Obstacles.h>
#include <perception_msgs/Pedestrians.h>
#include <perception_msgs/RoadClosures.h>
#include <perception_msgs/SpeedLimitMarkings.h>
#include <perception_msgs/StartLines.h>
#include <perception_msgs/Unidentifieds.h>
THIRD_PARTY_HEADERS_END

#include "road_object_visitor.h"

namespace road_object_detection {

template <typename Message, typename Submessage>
class MessageHelper {
 public:
  MessageHelper() = default;

  ~MessageHelper() { publisher.shutdown(); }

  void shutdown() { publisher.shutdown(); }

  //! \brief Advertise the ROS topic.
  void advertise(ros::NodeHandle& node_handle, const std::string& topic_name) {
    publisher = node_handle.advertise<Message>(topic_name, 1);
  }

  void addSubMessage(const Submessage& submessage) {
    sub_messages.push_back(submessage);
  }

  //! \brief Publish completed Message.
  void publishMessage(const Message& message) { publisher.publish(message); }

  Message generateAndClearMessage(const ros::Time& time_stamp) {
    Message message;
    message.header.stamp = time_stamp;
    message.header.frame_id = "world";
    message.sub_messages = this->sub_messages;

    sub_messages.clear();
    return message;
  }



 private:
  //! \brief Vector of Submessages which may be of different (RoadObject) types.
  std::vector<Submessage> sub_messages;

  ros::Publisher publisher;
};


//! \brief The MessageHandler generates messages as a visitor by 'visiting' the
//! RoadObjects.
class MessageHandler : public RoadObjectVisitor {
 public:
  MessageHandler(ros::NodeHandle& node_handle);

  void advertise();
  void shutdown();

  //! \brief Main function call. Iterates road_objects and 'visits' each one.
  //! The RoadObjects in return call the corresponding visit function.
  //! Calls the private publish method.
  void publishRoadObjects(RoadObjects& road_objects, const ros::Time& stamp);

  //! \brief Generates message and adds it to the correspondig MessageHelper by
  //! calling 'addSubMessage'.
  void visit(Unidentified& road_object) override;
  void visit(Obstacle& road_object) override;
  void visit(Junction& road_object) override;
  void visit(Crosswalk& road_object) override;
  void visit(RoadClosure& road_object) override;
  void visit(SpeedLimitMarking& road_object) override;
  void visit(StartLine& road_object) override;
  void visit(ArrowMarking& road_object) override;
  void visit(Pedestrian& road_object) override;
  void visit(NoPassingZone& road_object) override;


 private:
  //! \brief Calls the publish function of all MessageHelpers.
  void publish(const ros::Time& time_stamp);

  ros::NodeHandle& node_handle_;

  //! \brief MessageHelpers contain RoadObject messages of he same RoadObject
  //! type and are capable of advertising and publishing them.
  MessageHelper<perception_msgs::Unidentifieds, perception_msgs::Unidentified> helper_unidentified_msg;
  MessageHelper<perception_msgs::Junctions, perception_msgs::Junction> helper_junction_msg;
  MessageHelper<perception_msgs::Crosswalks, perception_msgs::Crosswalk> helper_crosswalk_msg;
  MessageHelper<perception_msgs::RoadClosures, perception_msgs::RoadClosure> helper_road_closure_msg;
  MessageHelper<perception_msgs::SpeedLimitMarkings, perception_msgs::SpeedLimitMarking> helper_speed_limit_marking_msg;
  MessageHelper<perception_msgs::StartLines, perception_msgs::StartLine> helper_start_stop_line_msg;
  MessageHelper<perception_msgs::Obstacles, perception_msgs::Obstacle> helper_obstacle_msg;
  MessageHelper<perception_msgs::ArrowMarkings, perception_msgs::ArrowMarking> helper_arrow_marking_msg;
  MessageHelper<perception_msgs::Pedestrians, perception_msgs::Pedestrian> helper_pedestrian_msg;
  MessageHelper<perception_msgs::NoPassingZones, perception_msgs::NoPassingZone> helper_no_passing_zone_msg;
  ros::Publisher road_objects_publisher;
};


}  // namespace road_object_detection

#endif  // MESSAGE_HANDLER_H
