#ifndef NODE_CREATION_MAKROS
#define NODE_CREATION_MAKROS

/*!
 *  This file contains makros for creating nodes/nodelets.
 */

#define DISABLE_DATE_TIME_WARNING \
  _Pragma("GCC diagnostic push")  \
      _Pragma("GCC diagnostic ignored \"-Wdate-time\"")

#define ENABLE_WARNINGS _Pragma("GCC diagnostic pop")

#if defined(BUILD_NODELET)
#include "common/nodelet_base.h"
using common::node_base::NodeletBase;
#define CREATE_NODE(TYPE) \
  PLUGINLIB_EXPORT_CLASS(NodeletBase<TYPE>, nodelet::Nodelet)  // NOLINT
#else
/*!
 * \def CREATE_NODE(TYPE)
 * \brief CREATE_NODE creates the entry point for a node/nodelet.
 *
 * For a node it creates a main method, for a nodelet a plugin is created.
 * TYPE is the type of the node.
 */
#define CREATE_NODE(TYPE)                                                      \
  int main(int argc, char *argv[]) {                                           \
    ros::init(argc, argv, TYPE::getName());                                    \
    ros::NodeHandle nh("~");                                                   \
    DISABLE_DATE_TIME_WARNING                                                  \
    ROS_INFO_STREAM(TYPE::getName() << "_node (built at " << __DATE__ << " - " \
                                    << __TIME__ << ") has been started.");     \
    ENABLE_WARNINGS                                                            \
    TYPE node(nh);                                                             \
    node.activateIfDesired();                                                  \
    return node.loop();                                                        \
  }
#endif

#if defined(BUILD_NODELET)
#include "common/nodelet_base.h"
using common::node_base::NodeletBase;
#define CREATE_NODE_WITH_FANCY_DEBUG(TYPE, DEBUG_TYPE) \
  PLUGINLIB_EXPORT_CLASS(NodeletBase<TYPE>, nodelet::Nodelet)  // NOLINT
#elif defined(NO_FANCY_DEBUG)
#define CREATE_NODE_WITH_FANCY_DEBUG(TYPE, DEBUG_TYPE) CREATE_NODE(TYPE)
#else
/*!
 * \def CREATE_NODE_WITH_FANCY_DEBUG(TYPE)
 * \brief CREATE_NODE_WITH_FANCY_DEBUG creates the entry point for a
 *  node/nodelet (with fancy-debug).
 *
 * For a node it creates a main method, for a nodelet a plugin is created.
 * TYPE is the type of the node, DEBUG_TYPE it the debug type of the node.
 *
 * If during runtime the parameter 'debug_mode' is set, DEBUG_TYPE is used,
 * otherwise DEBUG is used.
 *
 * If NO_FANCY_DEBUG is defined, this decays to CREATE_NODE.
 */
#define CREATE_NODE_WITH_FANCY_DEBUG(TYPE, DEBUG_TYPE)                         \
  int main(int argc, char *argv[]) {                                           \
    ros::init(argc, argv, TYPE::getName());                                    \
    ros::NodeHandle nh("~");                                                   \
    DISABLE_DATE_TIME_WARNING                                                  \
    ROS_INFO_STREAM(TYPE::getName() << "_node (built at " << __DATE__ << " - " \
                                    << __TIME__ << ") has been started.");     \
    ENABLE_WARNINGS                                                            \
    bool debug_mode = false;                                                   \
    nh.param<bool>("debug_mode", debug_mode, false);                           \
    if (debug_mode) {                                                          \
      ROS_WARN("Starting in debug mode!");                                     \
      DEBUG_TYPE node(nh);                                                     \
      node.activateIfDesired();                                                \
      return node.loop();                                                      \
    } else {                                                                   \
      ROS_DEBUG("Starting in normal mode!");                                   \
      TYPE node(nh);                                                           \
      node.activateIfDesired();                                                \
      return node.loop();                                                      \
    }                                                                          \
  }
#endif

#if defined(NO_FANCY_DEBUG) || defined(BUILD_NODELET)
#define FANCY_DEBUG_INCLUDE(header) "common/detail/dummy.h"
#else
/*!
 * \def FANCY_DEBUG_INCLUDE(header)
 * \brief FANCY_DEBUG_INCLUDE conditionaly includes headers needed for fancy
 *  debug.
 *
 * If NO_FANCY_DEBUG is defined the given header file is not included.
 */
#define FANCY_DEBUG_INCLUDE(header) header
#endif

#endif  // NODE_CREATION_MAKROS
