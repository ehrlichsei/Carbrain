#ifndef BOOST_SML_HELPER_H
#define BOOST_SML_HELPER_H

#include "common/macros.h"

THIRD_PARTY_HEADERS_BEGIN
#include <boost_sml_catkin/sml.hpp>
#include <mission_control_msgs/StateMachineStates.h>
#include <type_traits>
THIRD_PARTY_HEADERS_END

namespace sml_helper {
namespace sml = boost::sml;

template <typename T>
struct remove_back_policy {};
template <class A>
struct remove_back_policy<boost::sml::v1_1_0::back::sm_policy<A>> {
  using type = A;
};

template <typename T>
struct remove_aux_string {};
template <class A>
struct remove_aux_string<boost::sml::v1_1_0::aux::string<A>> {
  using type = A;
};

template <typename T>
struct get_state_type {
  using type =
      typename remove_aux_string<typename boost::sml::back::policies::get_state_name<T>::type>::type;
};


/*
template <typename state, typename SubSMList>
struct IsSubSM;
template <typename state, typename First, typename... Args>
struct IsSubSM<state, boost::sml::v1_1_0::aux::type_list<First, Args...>> {
  static constexpr bool value =
      std::is_same<state, SM::ObstacleStateMachine>::value ||
      std::is_same<state, typename remove_back_policy<First>::type>::value ||
      IsSubSM<state, boost::sml::v1_1_0::aux::type_list<Args...>>::value;
};
template <typename state>
struct IsSubSM<state, boost::sml::v1_1_0::aux::type_list<>> {
  static constexpr bool value = false;
};*/

template <typename T>
class IsSM {
  // every state machine class has tooverload the () operator
 private:
  typedef char YesType[1];
  typedef char NoType[2];

  template <typename C>
  static YesType& test(decltype(&C::operator()));
  template <typename C>
  static NoType& test(...);

 public:
  enum { value = sizeof(test<T>(0)) == sizeof(YesType) };
};

template <class T>
mission_control_msgs::StateMachineState getStateMessage() {
  return T().toMsg();
}
template <>
mission_control_msgs::StateMachineState getStateMessage<boost::sml::v1_1_0::front::internal>() {
  mission_control_msgs::StateMachineState state;
  state.name = "boost::sml::v1_1_0::front::internal";
  return state;
}

template <typename high_state_machine, typename kitcar_state>
std::enable_if_t<!IsSM<kitcar_state>::value> call_all_states(
    mission_control_msgs::StateMachineStates& /*states_msg*/,
    std::shared_ptr<high_state_machine>& /*obstacle_state_machine*/) {}

template <typename high_state_machine, typename kitcar_state>
std::enable_if_t<IsSM<kitcar_state>::value> call_all_states(
    mission_control_msgs::StateMachineStates& states_msg,
    std::shared_ptr<high_state_machine>& obstacle_state_machine) {
  obstacle_state_machine->template visit_current_states<decltype(sml::state<kitcar_state>)>(
      [&](auto state) {
        using current_state = typename get_state_type<decltype(state)>::type;
        states_msg.states.push_back(getStateMessage<current_state>());
        call_all_states<high_state_machine, current_state>(states_msg, obstacle_state_machine);
      });
}

}  // namespace sml_helper

#endif  // BOOST_SML_HELPER_H
