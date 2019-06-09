#ifndef PEDESTRIAN_SERVER_DEBUG_H
#define PEDESTRIAN_SERVER_DEBUG_H

#include "../../servers/pedestrian_server.h"
#include "debug_images_publisher.h"

THIRD_PARTY_HEADERS_BEGIN

THIRD_PARTY_HEADERS_END

namespace look_at {

class PedestrianServerDebug : public PedestrianServer, public DebugImagesPublisher {
 public:
  PedestrianServerDebug(const ros::NodeHandle &node_handle,
                        ParameterInterface *parameters,
                        const std::string &action_name);

  virtual ~PedestrianServerDebug() override = default;

  virtual void regionsOfInterestCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                         const nav_msgs::PathConstPtr &right_points,
                                         const nav_msgs::PathConstPtr &middle_points,
                                         const nav_msgs::PathConstPtr &left_points,
                                         const nav_msgs::PathConstPtr &no_passing_points) override;

  virtual void advertise() override;

  virtual void shutdown() override;

  virtual bool isDebug() const override { return true; }

 private:
};

}  // namespace look_at


#endif  // PEDESTRIAN_SERVER_DEBUG_H
