#ifndef PARAMETERHANDLER_H
#define PARAMETERHANDLER_H
#include <common/macros.h>

#include "parameter_interface.h"

THIRD_PARTY_HEADERS_BEGIN
#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
THIRD_PARTY_HEADERS_END

namespace common {
namespace node_base_detail {

/*!
 * \brief The AbstractUpdater class makes it possible to have a vector
 * of differenc Updaters for diffreren dynamic_reconfigure::Servers.
 */
class AbstractUpdater {
 public:
  typedef std::unique_ptr<AbstractUpdater> Ptr;
  virtual void update() = 0;
  virtual ~AbstractUpdater() = default;
};

template <class T>
/*!
 * \brief The Updater class is updates its dynamic_reconfigure::Server<T>.
 */
class Updater : public AbstractUpdater {
 private:
  const ros::NodeHandle nh;
  dynamic_reconfigure::Server<T> server;

 public:
  Updater(const ros::NodeHandle &nh, boost::recursive_mutex &mutex)
      : nh(nh), server(mutex, nh) {}

  /*!
   * \brief update loads the current parameters of the config from the
   * parameter-server and updates the dynamic_reconfigure::Server and
   * all its clients.
   */
  virtual void update() override {
    T config;
    config.__fromServer__(nh);
    server.updateConfig(config);
  }
};

}  // namespace node_base_detail

namespace node_base {
/*!
 * \brief The ParameterHandler class  encapsulates the parameter access and
 * switch at driving mode change. At initialisation the parameters in the
 * default subnamespace are transferred in the working namespace (which is the
 * only namespace the node itself can access parameters in). If the driving mode
 * has changed, the driving mode specific parameter are copied from the driving
 * mode specific subnamespace to the working subnamespace. If a parameter has
 * not been setted in a driving mode specific subnamespace, the value of the
 * default subnamespace is taken. If a parameter will be accessed, which has not
 * been registered or could not be found, a ros::InvalidParameterException will
 * be thrown with an appropriate error message.
 */
class ParameterHandler : public ParameterInterface {
 public:
  /*!
   * \brief ParameterHandler creates an instance of ParameterHandler.
   * \param private_node_handle the node_handle to access the parameter server
   * of ROS.
   */
  ParameterHandler(const ros::NodeHandle &private_node_handle);
  ParameterHandler() = delete;
  ~ParameterHandler() override = default;

  //! ParameterInterface

  // Parameter Registrations
  virtual void registerParam(const ParameterString<bool> &parameter) override;
  virtual void registerParam(const ParameterString<int> &parameter) override;
  virtual void registerParam(const ParameterString<double> &parameter) override;
  virtual void registerParam(const ParameterString<std::string> &parameter) override;

  // Parameter Registrations - vectors
  virtual void registerParam(const ParameterString<std::vector<bool>> &parameter) override;
  virtual void registerParam(const ParameterString<std::vector<int>> &parameter) override;
  virtual void registerParam(const ParameterString<std::vector<double>> &parameter) override;
  virtual void registerParam(const ParameterString<std::vector<std::string>> &parameter) override;

  // Parameter Registrations - maps
  virtual void registerParam(const ParameterString<std::map<std::string, bool>> &parameter) override;
  virtual void registerParam(const ParameterString<std::map<std::string, int>> &parameter) override;
  virtual void registerParam(const ParameterString<std::map<std::string, double>> &parameter) override;
  virtual void registerParam(const ParameterString<std::map<std::string, std::string>> &parameter) override;

  // Parameter Getters
  virtual bool getParam(const ParameterString<bool> &parameter) const override;
  virtual int getParam(const ParameterString<int> &parameter) const override;
  virtual double getParam(const ParameterString<double> &parameter) const override;
  virtual std::string getParam(const ParameterString<std::string> &parameter) const override;

  // Parameter Getters - vectors
  virtual std::vector<bool> getParam(const ParameterString<std::vector<bool>> &parameter) const override;
  virtual std::vector<int> getParam(const ParameterString<std::vector<int>> &parameter) const override;
  virtual std::vector<double> getParam(const ParameterString<std::vector<double>> &parameter) const override;
  virtual std::vector<std::string> getParam(
      const ParameterString<std::vector<std::string>> &parameter) const override;

  // Parameter Getters - maps
  virtual std::map<std::string, bool> getParam(
      const ParameterString<std::map<std::string, bool>> &parameter) const override;
  virtual std::map<std::string, int> getParam(
      const ParameterString<std::map<std::string, int>> &parameter) const override;
  virtual std::map<std::string, double> getParam(
      const ParameterString<std::map<std::string, double>> &parameter) const override;
  virtual std::map<std::string, std::string> getParam(
      const ParameterString<std::map<std::string, std::string>> &parameter) const override;

  /*!
   * \brief changeNamespace loads the parameter of the given subnamespace in the
   * working subnamespace. The switch between the different driving modes is
   * implemented in here.
   * \param sub_namespace the subnamespace to transferre the parameters from.
   */
  void changeNamespace(const std::string &sub_namespace) const;

  /*!
   * \brief isRegistered checks if the parameter name of type type is registeres
   * in namespace nh
   * \param nh the namespace to look in
   * \param name the parameter to be checked
   * \param type the type of the parameter
   */
  bool isRegistered(const ros::NodeHandle &nh,
                    const std::string &name,
                    const std::string &type) const;

  /*!
   * \brief addDynamicReconfigureServer adds a new dynamic_reconfigure Server
   * with its own update function. The template-parameter specifies the Config
   * type.
   * \param nh the namespace, where the parameters are lay in.
   */
  template <typename T>
  void addDynamicReconfigureServer(const ros::NodeHandle &nh);

 private:
  /*!
   * \brief generateParameterNotFoundException generates a
   * ros::InvalidParameterException with an error message pointing out, that the
   * given parameter could not be found.
   * \param parameter_name the parameter the exception occurred at.
   * \return the generated InvalidParameterException
   */
  static ros::InvalidParameterException generateParameterNotFoundException(const std::string &parameter_name);

  /*!
   * \brief generateWrongParameterTypeException generates a
   * ros::InvalidParameterException with an error message pointing out, that the
   * given parameter type was wrong.
   * \param parameter_name the parameter the exception occurred at.
   * \return the generated InvalidParameterException
   */
  static ros::InvalidParameterException generateWrongParameterTypeException(const std::string &parameter_name);

  /*!
   * \brief generateParameterNotRegisteredException generates a
   * ros::InvalidParameterException with an error message pointing out, that the
   * given parameter has not been registered at the parameter handler.
   * \param parameter_name the parameter the exception occurred at.
   * \return the generated InvalidParameterException
   */
  static ros::InvalidParameterException generateParameterNotRegisteredException(
      const std::string &parameter_name);

  /*!
   * \brief generateInvalidParameterException generates a
   * ros::InvalidParameterException, with an error message composed of an
   * general message and the concrete parameter.
   * \param message the message to be used.
   * \param parameter_name the parameter the exception occurred at.
   * \return the generated InvalidParameterException
   */
  static ros::InvalidParameterException generateInvalidParameterException(
      const std::string &message, const std::string &parameter_name);

  /*!
   * \brief generateInvalidParameterException generates a
   * ros::InvalidParameterException, with an error message composed of an
   * general message, the concrete parameter and a hint about the error.
   * \param message the message to be used.
   * \param parameter_name the parameter the exception occurred at.
   * \param hint the hint about the error.
   * \return the generated InvalidParameterException
   */
  static ros::InvalidParameterException generateInvalidParameterException(
      const std::string &message, const std::string &parameter_name, const std::string &hint);

  /*!
   * \brief changeNamespaceFor transfers the value of all parameter off the
   * specified type TYPE from
   * the given subnamespace to the working namespace.
   * \param sub_namespace the subnamespace to load the parameter from.
   */
  template <typename TYPE>
  void changeNamespaceFor(const std::string &sub_namespace) const;

  /*!
   * \brief loadDefaultFor transfers the value of a specific parameter from the
   * default subnamespace to the working namespace If the parameter has the
   * wrong type or could not be found, a ros::InvalidParameterException will be
   * thrown with an appropriate error message.
   * \param param_name params the parameter to transfer.
   */
  template <typename TYPE>
  void loadDefaultFor(const std::string &param_name) const;

  /*!
   * \brief getTypedParameter
   *
   * returns the value (of the given type) corresponding to the given name.
   *
   * \param parameter_name the name of the parameter to get the value of.
   * \return the value of the parameter.
   */
  template <typename TYPE>
  TYPE getTypedParameter(const std::string &parameter_name) const;

  /*!
   * \brief registerTypedParameter
   *
   * registers a parameter name for the given type.
   *
   * \param parameter_name the name of the parameter to register
   */
  template <typename TYPE>
  void registerTypedParameter(const std::string &parameter_name);

  /*!
   * \brief getTypedStringSet
   *
   * This template function provides an typed access to the string set
   *associated with the given type.
   *
   * \return the std::set<std::string> to the given type
   */
  template <typename TYPE>
  inline std::set<std::string> &getTypedStringSet();

  /*!
   * \brief getTypedStringSet
   *
   * This template function provides an typed access to the string set
   *associated with the given type.
   *
   * \return the std::set<std::string> to the given type
   */
  template <typename TYPE>
  inline const std::set<std::string> &getTypedStringSet() const;

  /*!
   * \brief checkIfAlreadyRegistereda
   *
   * checks of the given parameter of type T2 was already registrated as
   *parameter of type T1.
   * The following cases can occure:
   *   - the parameter has not been regstered as parameter of type T1: nothing
   *happens.
   *   - the parameter has been registered as parameter of the same type before
   *(T1 == T2): a warning is printed.
   *   - the parameter has been registered as parameter of another type before
   *(T1 != T2): a exception is thrown.
   *
   * \param parameter_name the parameter to be checked
   */
  template <typename T1, typename T2>
  void checkIfAlreadyRegistered(const std::string &parameter_name) const;

  /*!
   * \brief getStringSet returns the std::set<std::string> corresponding to the
   * given type.
   * \param type the type to get the string set of.
   * \return std::set<std::string> corresponding to the given type.
   */
  const std::set<std::string> &getStringSet(const std::string &type) const;

  template <typename T>
  void checkIfDynamicReconfigureParametersRegistered(const ros::NodeHandle &nh) const;

  /*!
   * \brief updateAllDynamicReconfigureServers
   *
   * excutes every function in dynamic_reconfigure_updater_ to update all
   *parameters in
   * dynamic_reconfigure.
   */
  void updateAllDynamicReconfigureServers() const;

  /*!
   * \brief node_handle_ is needed to accessing the ROS parameter server.
   */
  const ros::NodeHandle node_handle_;

  const std::string parent_ns_;

  /*!
   * \brief bool_params_ contains all registered parameters of the type boolean
   * to be able to perform the driving mode switch on them.
   */
  std::set<std::string> bool_params_;

  /*!
   * \brief int_params_ contains all registered parameters of the type integer
   * to be able to perform the driving mode switch on them.
   */
  std::set<std::string> int_params_;

  /*!
   * \brief double_params_ contains all registered parameters of the type
   * double precision floating point number to be able to perform the driving
   * mode switch on them.
   */
  std::set<std::string> double_params_;

  /*!
   * \brief string_params_ contains all registered parameters of the type
   * std::string to be able to perform the driving mode switch on them.
   */
  std::set<std::string> string_params_;

  // vectors
  /*!
   * \brief vector_bool_params_ contains all registered parameters of the type
   * std::vector<bool> to be able to perform the driving mode switch on them.
   */
  std::set<std::string> vector_bool_params_;

  /*!
   * \brief vector_int_params_ contains all registered parameters of the type
   * std::vector<int> to be able to perform the driving mode switch on them.
   */
  std::set<std::string> vector_int_params_;

  /*!
   * \brief vector_double_params_ contains all registered parameters of the type
   * std::vector<double> to be able to perform the driving mode switch on them.
   */
  std::set<std::string> vector_double_params_;

  /*!
   * \brief vector_string_params_ contains all registered parameters of the type
   * std::vector<std::string> to be able to perform the driving mode switch on
   * them.
   */
  std::set<std::string> vector_string_params_;

  // maps
  /*!
   * \brief map_bool_params_ contains all registered parameters of the type
   * std::map<std::string, bool> to be able to perform the driving mode switch
   * on them.
   */
  std::set<std::string> map_bool_params_;

  /*!
   * \brief map_int_params_ contains all registered parameters of the type
   * std::map<std::string, int> to be able to perform the driving mode switch on
   * them.
   */
  std::set<std::string> map_int_params_;

  /*!
   * \brief map_double_params_ contains all registered parameters of the type
   * std::map<std::string, double> to be able to perform the driving mode switch
   * on them.
   */
  std::set<std::string> map_double_params_;

  /*!
   * \brief map_string_params_ contains all registered parameters of the type
   * std::map<std::string, std::string> to be able to perform the driving mode
   * switch on
   * them.
   */
  std::set<std::string> map_string_params_;

  /*!
   * \brief dynamic_reconfigure_updater_ list of updater functions for
   * dynamic_reconfigure
   */
  std::vector<node_base_detail::AbstractUpdater::Ptr> dynamic_reconfigure_updater_;

  // To avoid warning from dynamic_reconfigure::Server
  boost::recursive_mutex dynamic_reconfigure_server_mutex_;
};

template <typename T>
void ParameterHandler::checkIfDynamicReconfigureParametersRegistered(const ros::NodeHandle &nh) const {
  for (const auto &param_description : T::__getParamDescriptions__()) {
    if (!isRegistered(nh, param_description->name, param_description->type)) {
      ROS_ERROR(
          "Dynamic reconfigure uses not registered parameter %s !!! Maybe the "
          "NodeHandle is wrong: %s or the type in the cfg (%s) differs from "
          "the type in the parameter server.",
          param_description->name.c_str(),
          nh.getNamespace().c_str(),
          param_description->type.c_str());
    }
  }
}

template <typename T>
void ParameterHandler::addDynamicReconfigureServer(const ros::NodeHandle &nh) {
  checkIfDynamicReconfigureParametersRegistered<T>(nh);

  dynamic_reconfigure_updater_.push_back(std::make_unique<node_base_detail::Updater<T>>(
      nh, dynamic_reconfigure_server_mutex_));
}

}  // namespace node_base;
using namespace node_base;
}  // namspace common;

#endif  // PARAMETERHANDLER_H
