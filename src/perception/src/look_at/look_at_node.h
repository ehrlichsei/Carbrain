#ifndef LOOK_AT_NODE_H
#define LOOK_AT_NODE_H

#include "look_at.h"
#include "servers/server_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
THIRD_PARTY_HEADERS_END

namespace look_at {
/*!
 * \brief Node in order to observe specific regions on demand, invoked from
 * navigation via action
 */
class LookAtNode : public NodeBase {
 public:
  /*!
   * \brief LookAtNode the constructor.
   * \param node_handle the NodeHandle to be used.
   */
  LookAtNode(ros::NodeHandle &node_handle);

  virtual ~LookAtNode() override = default;
  /*!
   * \brief returns the name of the node. It is needed in main and onInit
   * (nodelet) method. \return the name of the node
   */
  static const std::string getName();

 protected:
  // NodeBase interface
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

  std::vector<std::unique_ptr<ServerInterface>> servers_;
};
}  // namespace look_at

#endif  // LOOK_AT_NODE_H
