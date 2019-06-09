#include "timeout.h"

Timeout::Timeout(const ros::Time& start, const ros::Duration& duration)
    : end(start + duration), timeout_when_blocked(ros::Duration(0.0)) {}

void Timeout::setNewDuration(const ros::Time& start, const ros::Duration& duration) {
  end = start + duration;
}

void Timeout::update(const ros::Time& current) { this->current = current; }

bool Timeout::hasPassed() {
  return (!blocked && !end.isZero() && current > end) ||
         (blocked && !end.isZero() && current > (end + timeout_when_blocked));
}

bool Timeout::isEnabled() { return !end.isZero(); }

bool Timeout::getBlocked() const { return blocked; }

void Timeout::setBlocked(bool value) { blocked = value; }

void Timeout::setTimeoutWhenBlocked(const ros::Duration& timeout_when_blocked) {
  this->timeout_when_blocked = timeout_when_blocked;
}
