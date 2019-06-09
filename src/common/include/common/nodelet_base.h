#ifndef NODELET_BASE
#define NODELET_BASE
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
THIRD_PARTY_HEADERS_END

namespace common {
namespace node_base {
/*!
 * This class provides a simple way to create a nodelet based onbased on the 
 * NodeBase. If you use node_creation_macros, you will not need to care about
 * this anyway because the macros will do it for you.
 */

template <typename TYPE>
class NodeletBase : public nodelet::Nodelet {
  // Nodelet interface
 private:
  virtual void onInit() override {
    ROS_INFO_STREAM(TYPE::getName() << "_nodelet (built at "<< __DATE__ << " - " << __TIME__ << ") has been started.");
    // the object has to be dynamicly allocated (on the head), because the
    // NodeletBase class must be instantiated beford the Node class, to get the
    // private NodeHandle.
    node = std::make_unique<TYPE>(getPrivateNodeHandle());
    node->activateIfDesired();
  }

  std::unique_ptr<TYPE> node;
};

} // namepsace node_base;
} // namespace common;

#endif  // NODELET_BASE
