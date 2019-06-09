#pragma once

#include <string>
#include <vector>
#include <map>

namespace common {
namespace type_traits {
/*!
 * \brief toString
 *
 * returns a string representation of the given type.
 * \return a string representation of the given type.
 */
template <typename TYPE>
inline const std::string toString();

template <>
const std::string toString<bool>() {
  return "bool";
}

template <>
const std::string toString<int>() {
  return "int";
}

template <>
const std::string toString<double>() {
  return "double";
}

template <>
const std::string toString<std::string>() {
  return "std::string";
}

// vectors
template <>
const std::string toString<std::vector<bool> >() {
  return "std::vector<bool>";
}

template <>
const std::string toString<std::vector<int> >() {
  return "std::vector<int>";
}

template <>
const std::string toString<std::vector<double> >() {
  return "std::vector<double>";
}

template <>
const std::string toString<std::vector<std::string> >() {
  return "std::vector<std::string>";
}

// maps
template <>
const std::string toString<std::map<std::string, bool> >() {
  return "std::map<std::string, bool>";
}

template <>
const std::string toString<std::map<std::string, int> >() {
  return "std::map<std::string, int>";
}

template <>
const std::string toString<std::map<std::string, double> >() {
  return "std::map<std::string, double>";
}

template <>
const std::string toString<std::map<std::string, std::string> >() {
  return "std::map<std::string, std::string>";
}
}  // namespace type_traits;
using namespace type_traits;
}  // namespace common;
