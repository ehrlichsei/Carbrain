#ifndef TIMEOUT_H
#define TIMEOUT_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <limits>
#include <ros/ros.h>
THIRD_PARTY_HEADERS_END

class Timeout {

 public:
  Timeout() = default;
  Timeout(const ros::Time& start,
          const ros::Duration& duration);
  void setNewDuration(const ros::Time& start, const ros::Duration& duration);
  void update(const ros::Time& current);
  bool hasPassed();
  bool isEnabled();

  bool getBlocked() const;
  void setBlocked(bool value);
  void setTimeoutWhenBlocked(const ros::Duration& timeout_when_blocked);

 private:
  ros::Time end{0};
  ros::Time current;
  bool blocked = false;
  ros::Duration timeout_when_blocked;
};

#endif
