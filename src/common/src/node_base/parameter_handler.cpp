#include "common/parameter_handler.h"

#include <boost/algorithm/cxx11/any_of.hpp>

#include "common/contains.h"
#include "common/type_to_string.h"

#include "parameter_name_generator.h"

namespace common {
namespace node_base {

using common::node_base_detail::ParameterNameGenerator;

ParameterHandler::ParameterHandler(const ros::NodeHandle &private_node_handle)
    : node_handle_(private_node_handle),
      parent_ns_(ros::names::parentNamespace(node_handle_.getNamespace())) {}


// For Reasons the definitions of template function have to appear before the
// first instantiation, so leave it on top of this file and it will work.

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<bool>() {
  return bool_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<int>() {
  return int_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<double>() {
  return double_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::string>() {
  return string_params_;
}

// vectors
template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::vector<bool> >() {
  return vector_bool_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::vector<int> >() {
  return vector_int_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::vector<double> >() {
  return vector_double_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::vector<std::string> >() {
  return vector_string_params_;
}

// maps
template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::map<std::string, bool> >() {
  return map_bool_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::map<std::string, int> >() {
  return map_int_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::map<std::string, double> >() {
  return map_double_params_;
}

template <>
std::set<std::string> &ParameterHandler::getTypedStringSet<std::map<std::string, std::string> >() {
  return map_string_params_;
}

// Quote from "Effective C++" (p. 79) : "Pretty this ain't, but sometimes a
// programmer's just gotta do what a programmer's gotta do." This avoids to
// add a const version of every getter which would be worse (code redundance
// is evil!).
template <typename TYPE>
const std::set<std::string> &ParameterHandler::getTypedStringSet() const {
  return const_cast<std::set<std::string> &>(  // NOLINT;
      (const_cast<ParameterHandler *>(this))->getTypedStringSet<TYPE>());  // NOLINT;
}

template <typename TYPE>
void ParameterHandler::registerTypedParameter(const std::string &parameter_name) {

  checkIfAlreadyRegistered<bool, TYPE>(parameter_name);
  checkIfAlreadyRegistered<int, TYPE>(parameter_name);
  checkIfAlreadyRegistered<double, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::string, TYPE>(parameter_name);

  // vectors
  checkIfAlreadyRegistered<std::vector<bool>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::vector<int>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::vector<double>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::vector<std::string>, TYPE>(parameter_name);

  // maps
  checkIfAlreadyRegistered<std::map<std::string, bool>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::map<std::string, int>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::map<std::string, double>, TYPE>(parameter_name);
  checkIfAlreadyRegistered<std::map<std::string, std::string>, TYPE>(parameter_name);

  loadDefaultFor<TYPE>(parameter_name);
  getTypedStringSet<TYPE>().insert(parameter_name);
}

template <typename TYPE>
TYPE ParameterHandler::getTypedParameter(const std::string &parameter_name) const {

  if (!common::contains(getTypedStringSet<TYPE>(), parameter_name)) {
    throw generateParameterNotRegisteredException(parameter_name);
  }

  TYPE param;
  if (!node_handle_.getParamCached(parameter_name, param)) {
    throw generateParameterNotFoundException(parameter_name);
  }

  return param;
}


template <typename TYPE>
void ParameterHandler::changeNamespaceFor(const std::string &sub_namespace) const {

  for (const std::string &param_name : getTypedStringSet<TYPE>()) {
    TYPE param;
    const ParameterNameGenerator parameter_name(param_name, parent_ns_);

    if (!node_handle_.getParam(parameter_name.getExtendedName(sub_namespace), param)) {
      node_handle_.getParam(parameter_name.getExtendedName("default"), param);
    }

    node_handle_.setParam(param_name, param);
  }
}

template <typename TYPE>
void ParameterHandler::loadDefaultFor(const std::string &param_name) const {
  const ParameterNameGenerator parameter_name(param_name, parent_ns_);
  const std::string default_param_name =
      parameter_name.getExtendedName("default");

  if (!node_handle_.hasParam(default_param_name)) {
    throw generateParameterNotFoundException(default_param_name);
  }

  TYPE param;
  if (!node_handle_.getParam(default_param_name, param)) {
    throw generateWrongParameterTypeException(param_name);
  }

  node_handle_.setParam(param_name, param);
}

// ParameterInterface

// Parameter Registrations
void ParameterHandler::registerParam(const ParameterString<bool> &parameter) {
  registerTypedParameter<bool>(parameter);
}

void ParameterHandler::registerParam(const ParameterString<int> &parameter) {
  registerTypedParameter<int>(parameter);
}

void ParameterHandler::registerParam(const ParameterString<double> &parameter) {
  registerTypedParameter<double>(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::string> &parameter) {
  registerTypedParameter<std::string>(parameter);
}

// Parameter Registrations - vectors
void ParameterHandler::registerParam(const ParameterString<std::vector<bool> > &parameter) {
  registerTypedParameter<std::vector<bool> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::vector<int> > &parameter) {
  registerTypedParameter<std::vector<int> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::vector<double> > &parameter) {
  registerTypedParameter<std::vector<double> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::vector<std::string> > &parameter) {
  registerTypedParameter<std::vector<std::string> >(parameter);
}


// Parameter Registrations - maps
void ParameterHandler::registerParam(const ParameterString<std::map<std::string, bool> > &parameter) {
  registerTypedParameter<std::map<std::string, bool> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::map<std::string, int> > &parameter) {
  registerTypedParameter<std::map<std::string, int> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::map<std::string, double> > &parameter) {
  registerTypedParameter<std::map<std::string, double> >(parameter);
}

void ParameterHandler::registerParam(const ParameterString<std::map<std::string, std::string> > &parameter) {
  registerTypedParameter<std::map<std::string, std::string> >(parameter);
}

// Parameter Getters
bool ParameterHandler::getParam(const ParameterString<bool> &parameter) const {
  return getTypedParameter<bool>(parameter);
}

int ParameterHandler::getParam(const ParameterString<int> &parameter) const {
  return getTypedParameter<int>(parameter);
}

double ParameterHandler::getParam(const ParameterString<double> &parameter) const {
  return getTypedParameter<double>(parameter);
}

std::string ParameterHandler::getParam(const ParameterString<std::string> &parameter) const {
  return getTypedParameter<std::string>(parameter);
}

// Parameter Getters - vectors
std::vector<bool> ParameterHandler::getParam(const ParameterString<std::vector<bool> > &parameter) const {
  return getTypedParameter<std::vector<bool> >(parameter);
}

std::vector<int> ParameterHandler::getParam(const ParameterString<std::vector<int> > &parameter) const {
  return getTypedParameter<std::vector<int> >(parameter);
}

std::vector<double> ParameterHandler::getParam(const ParameterString<std::vector<double> > &parameter) const {
  return getTypedParameter<std::vector<double> >(parameter);
}

std::vector<std::string> ParameterHandler::getParam(
    const ParameterString<std::vector<std::string> > &parameter) const {
  return getTypedParameter<std::vector<std::string> >(parameter);
}

// Parameter Getters - maps
std::map<std::string, bool> ParameterHandler::getParam(
    const ParameterString<std::map<std::string, bool> > &parameter) const {
  return getTypedParameter<std::map<std::string, bool> >(parameter);
}

std::map<std::string, int> ParameterHandler::getParam(
    const ParameterString<std::map<std::string, int> > &parameter) const {
  return getTypedParameter<std::map<std::string, int> >(parameter);
}

std::map<std::string, double> ParameterHandler::getParam(
    const ParameterString<std::map<std::string, double> > &parameter) const {
  return getTypedParameter<std::map<std::string, double> >(parameter);
}

std::map<std::string, std::string> ParameterHandler::getParam(
    const ParameterString<std::map<std::string, std::string> > &parameter) const {
  return getTypedParameter<std::map<std::string, std::string> >(parameter);
}

void ParameterHandler::changeNamespace(const std::string &sub_namespace) const {
  ROS_DEBUG("copying parameters from \"%s\" namespace to working namespace",
            sub_namespace.c_str());

  changeNamespaceFor<bool>(sub_namespace);
  changeNamespaceFor<int>(sub_namespace);
  changeNamespaceFor<double>(sub_namespace);
  changeNamespaceFor<std::string>(sub_namespace);

  // vectors
  changeNamespaceFor<std::vector<bool> >(sub_namespace);
  changeNamespaceFor<std::vector<int> >(sub_namespace);
  changeNamespaceFor<std::vector<double> >(sub_namespace);
  changeNamespaceFor<std::vector<std::string> >(sub_namespace);

  // maps
  changeNamespaceFor<std::map<std::string, bool> >(sub_namespace);
  changeNamespaceFor<std::map<std::string, int> >(sub_namespace);
  changeNamespaceFor<std::map<std::string, double> >(sub_namespace);
  changeNamespaceFor<std::map<std::string, std::string> >(sub_namespace);

  updateAllDynamicReconfigureServers();
}

bool ParameterHandler::isRegistered(const ros::NodeHandle &nh,
                                    const std::string &name,
                                    const std::string &type) const {
  const std::string resolved_name = nh.resolveName(name);
  return boost::algorithm::any_of(getStringSet(type),
                                  [&](const auto &param) {
                                    return resolved_name == node_handle_.resolveName(param);
                                  });
}

template <typename T1, typename T2>
void ParameterHandler::checkIfAlreadyRegistered(const std::string &parameter_name) const {
  if (common::contains(getTypedStringSet<T1>(), parameter_name)) {
    if (std::is_same<T1, T2>::value) {
      ROS_WARN("Parameter '%s' has been already registered.", parameter_name.c_str());
    } else {
      throw generateInvalidParameterException(
          "Parameter was already added with type " + toString<T1>() +
              " (you tried to register it with type " + toString<T2>(),
          parameter_name);
    }
  }
}

// Exception Generators
ros::InvalidParameterException ParameterHandler::generateParameterNotFoundException(
    const std::string &parameter_name) {
  return generateInvalidParameterException("parameter not found", parameter_name);
}

ros::InvalidParameterException ParameterHandler::generateWrongParameterTypeException(
    const std::string &parameter_name) {
  return generateInvalidParameterException("wrong type for parameter", parameter_name);
}

ros::InvalidParameterException ParameterHandler::generateParameterNotRegisteredException(
    const std::string &parameter_name) {
  return generateInvalidParameterException(
      "parameter has not yet been registered",
      parameter_name,
      "have you called ParameterHandler::registerParam()?");
}

ros::InvalidParameterException ParameterHandler::generateInvalidParameterException(
    const std::string &message, const std::string &parameter_name) {
  return ros::InvalidParameterException(message + ": \"" + parameter_name +
                                        "\"");
}

ros::InvalidParameterException ParameterHandler::generateInvalidParameterException(
    const std::string &message, const std::string &parameter_name, const std::string &hint) {
  return ros::InvalidParameterException(message + ": \"" + parameter_name +
                                        "\"! " + hint);
}

const std::set<std::string> &ParameterHandler::getStringSet(const std::string &type) const {
  if (type == "int") {
    return int_params_;
  } else if (type == "double") {
    return double_params_;
  } else if (type == "str") {
    return string_params_;
  } else if (type == "bool") {
    return bool_params_;
  } else {
    ROS_ERROR("Check for %s is not implemented", type.c_str());
    return int_params_;
  }
}

void ParameterHandler::updateAllDynamicReconfigureServers() const {
  for (const auto &update_reconfigure : dynamic_reconfigure_updater_) {
    update_reconfigure->update();
  }
}

} // namespace node_base
} // namespace common
