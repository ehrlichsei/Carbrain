#ifndef LONGITUDINAL_CONTROLLER_NODE_DEBUG_H
#define LONGITUDINAL_CONTROLLER_NODE_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

#include "../longitudinal_controller_node.h"

class LongitudinalControllerNodeDebug : public LongitudinalControllerNode {
 public:
  LongitudinalControllerNodeDebug(ros::NodeHandle& node_handle);

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
};

#endif  // LONGITUDINAL_CONTROLLER_NODE_DEBUG_H
