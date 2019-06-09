#ifndef STATEMACHINELOGGER_H
#define STATEMACHINELOGGER_H
#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost_sml_catkin/sml.hpp>
THIRD_PARTY_HEADERS_END

struct StateMachineLogger {
  template <class SM, class TEvent>
  void log_process_event(const TEvent&) {
    /*printf("[%s][process_event] %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TEvent>());*/
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard&, const TEvent&, bool /*result*/) {
    /*printf("[%s][guard] %s %s %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TGuard>(),
           sml::aux::get_type_name<TEvent>(),
           (result ? "[OK]" : "[Reject]"));*/
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction&, const TEvent&) {
    /*printf("[%s][action] %s %s\n",
           sml::aux::get_type_name<SM>(),
           sml::aux::get_type_name<TAction>(),
           sml::aux::get_type_name<TEvent>());*/
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState& src, const TDstState& dst) {
    if (src.c_str() != dst.c_str()) {
      printf("[%s][transition] %s -> %s\n",
             boost::sml::aux::get_type_name<SM>(),
             src.c_str(),
             dst.c_str());
    }
  }
};
#endif  // STATEMACHINELOGGER_H
