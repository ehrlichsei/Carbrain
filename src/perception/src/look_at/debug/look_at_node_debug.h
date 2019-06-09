#ifndef LOOK_AT_NODE_DEBUG_H
#define LOOK_AT_NODE_DEBUG_H

#include "../look_at_node.h"

namespace look_at {
class LookAtNodeDebug : public LookAtNode {
 public:
  LookAtNodeDebug(ros::NodeHandle &node_handle);

  virtual ~LookAtNodeDebug() override = default;

 private:
  virtual void startModule() override;

  virtual void stopModule() override;

  void removeNonDebugServers();
};
}  // namespace look_at

#endif  // LOOK_AT_NODE_DEBUG_H
