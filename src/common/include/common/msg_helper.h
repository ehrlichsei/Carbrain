#pragma once
#include <boost/make_shared.hpp>

namespace common {
/*!
 * \namespace common::msg_helper
 * \brief msg_helper contains functions to make using messages easier.
 */
namespace msg_helper {
/*!
 * \brief toConstPtr creates a const shared message (boost::shared_ptr).
 * \param msg the message to make.
 * \return a shared message.
 */
template <typename T>
auto toConstPtr(T&& msg) {
  return boost::make_shared<std::remove_reference_t<T> const>(std::forward<T>(msg));
}

}  // namespace msg_helper;
using namespace msg_helper;
}  // namespace common;
