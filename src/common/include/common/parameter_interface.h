#ifndef PARAMETER_INTERFACE_H
#define PARAMETER_INTERFACE_H

#include <string>
#include <vector>
#include <map>

namespace common {
namespace node_base {
/*!
 * \brief The ParameterString class adds an type assinement to the std::string.
 * This is needed to add a type to an parameter name, which takes only effect
 * during compile time.
 */
template <typename TYPE>
class ParameterString : public std::string {
 public:
  template<typename F>
  ParameterString(F&& str) : std::string(std::forward<F>(str)) {}
};


/*!
 * \brief The ParameterInterface class provides an intereface, which
 * functionality implementing classes can use to access parameters without
 * introducing ros-dependencies. For every kind of possible parameter types are
 * two different functions provided: registerXParam and getXParam (X stands for
 * the concrete type). The getXParam functions provide the parameter access. The
 * registerXParam functions are needed to register the every parameter which
 * will be acceded to by getXparam calls. This is needed to check if the
 * parameter has been setted before the node starts running and to know which
 * parameters have to be copied in the wokring namespace (at start an if
 * driving-mode is changing).
 */
class ParameterInterface {

 public:
  virtual ~ParameterInterface() = default;
  ParameterInterface() = default;
  ParameterInterface(ParameterInterface&&) = default;
  ParameterInterface& operator=(ParameterInterface&&) = default;
  /*!
   * \brief registerParam registers a boolean parameter to the parameter
   * handler, if a parameter of type ParameterString<bool> is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<bool>& parameter) = 0;

  /*!
   * \brief registerParam registers a integer parameter to the parameter
   * handler, if a parameter of type ParameterString<bool> is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<int>& parameter) = 0;

  /*!
   * \brief registerParam registers a double precision floating point
   * number parameter to the parameter handler, if a parameter of type
   * ParameterString<bool> is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<double>& parameter) = 0;

  /*!
   * \brief registerParam registers a string parameter to the parameter
   * handler, if a parameter of type ParameterString<std::string> is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::string>& parameter) = 0;

  // vectors
  /*!
   * \brief registerParam registers a vector of bools parameter to the parameter
   * handler, if a parameter of type ParameterString<std::vector<bool> > is
   * given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::vector<bool> >& parameter) = 0;

  /*!
   * \brief registerParam registers a vector of ints parameter to the parameter
   * handler, if a parameter of type ParameterString<std::vector<int> > is
   * given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::vector<int> >& parameter) = 0;

  /*!
   * \brief registerParam registers a vector of doubles parameter to the
   * parameter handler, if a parameter of type is
   * ParameterString<std::vector<double> > is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::vector<double> >& parameter) = 0;

  /*!
   * \brief registerParam registers a vector of string parameter to the
   * parameter handler, if a parameter of type
   * ParameterString<std::vector<std::string> > is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::vector<std::string> >& parameter) = 0;

  // maps
  /*!
   * \brief registerParam registers a map - mapping from strings to bools -
   * parameter to the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, bool>> is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::map<std::string, bool> >& parameter) = 0;

  /*!
   * \brief registerParam registers a a map - mapping from strings to ints -  to
   * the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, int> > is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::map<std::string, int> >& parameter) = 0;

  /*!
   * \brief registerParam registers a a map - mapping from strings to doubles -
   * to the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, double> > is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::map<std::string, double> >& parameter) = 0;

  /*!
   * \brief registerParam registers a a map - mapping from strings to strings -
   * to the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, std::string> > is given.
   * \param parameter the name of the parameter to register.
   */
  virtual void registerParam(const ParameterString<std::map<std::string, std::string> >& parameter) = 0;

  // Parameter Getters
  /*!
   * \brief getParam accesses a boolean parameter from the parameter
   * handler, if a parameter of type ParameterString<bool> is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual bool getParam(const ParameterString<bool>& parameter) const = 0;

  /*!
   * \brief getParam accesses a integer parameter from the parameter handler,
   * if a parameter of type ParameterString<int> is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual int getParam(const ParameterString<int>& parameter) const = 0;
  /*!
   * \brief getParam accesses a double precision floating point number parameter
   * from the parameter handler, if a parameter of type ParameterString<int> is
   * given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual double getParam(const ParameterString<double>& parameter) const = 0;

  /*!
   * \brief getParam accesses a string parameter from the parameter handler,
   * if a parameter of type ParameterString<std::string> is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::string getParam(const ParameterString<std::string>& parameter) const = 0;

  // Parameter Getters - vectors
  /*!
   * \brief getParam accesses a vector of bools parameter from the parameter
   * handler, if a parameter of type ParameterString<std::vector<bool> > is
   * given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::vector<bool> getParam(const ParameterString<std::vector<bool> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a vector of ints parameter from the parameter
   * handler, if a parameter of type ParameterString<std::vector<int> > is
   * given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::vector<int> getParam(const ParameterString<std::vector<int> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a vector of doubles parameter from the parameter
   * handler, if a parameter of type ParameterString<std::vector<double> > is
   * given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::vector<double> getParam(const ParameterString<std::vector<double> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a vector of string parameter from the parameter
   * handler, if a parameter of type ParameterString<std::vector<std::string> >
   * is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::vector<std::string> getParam(
      const ParameterString<std::vector<std::string> >& parameter) const = 0;

  // Parameter Getters - maps
  /*!
   * \brief getParam accesses a a map - mapping from strings to bools -  from
   * the parameter handler,
   * if a parameter of type ParameterString<map<std::string, bool> > is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::map<std::string, bool> getParam(
      const ParameterString<std::map<std::string, bool> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a a map - mapping from strings to bools - from the
   * parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, int> >
   * is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::map<std::string, int> getParam(
      const ParameterString<std::map<std::string, int> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a a map - mapping from strings to doubles -  from
   * the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, double> >
   * is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::map<std::string, double> getParam(
      const ParameterString<std::map<std::string, double> >& parameter) const = 0;

  /*!
   * \brief getParam accesses a a map - mapping from strings to  strings -  from
   * the parameter handler, if a parameter of type
   * ParameterString<std::map<std::string, std::string> >
   * is given.
   * \param parameter the name of the parameter to access.
   * \return the value of the parameter.
   */
  virtual std::map<std::string, std::string> getParam(
      const ParameterString<std::map<std::string, std::string> >& parameter) const = 0;
};

}  // namepsace node_base;
using namespace node_base;
}  // namespace common;

using common::ParameterInterface;
using common::ParameterString;

#endif  // PARAMETER_INTERFACE_H
