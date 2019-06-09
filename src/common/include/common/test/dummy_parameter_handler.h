#ifndef PARAMETER_HANDLER_DUMMY_H
#define PARAMETER_HANDLER_DUMMY_H

#include "common/contains.h"
#include "common/parameter_interface.h"

#define ADD_GET_PARAM_MAP(type, attribute)                                          \
  template <>                                                                       \
  DummyParameterHandler::ParamMap<type> &DummyParameterHandler::getParameterMap() { \
    return attribute;                                                               \
  }

#define REGISTRATE \
  { registrate(parameter); }

#define GET \
  { return get(parameter); }

/**
 * @brief The DummyParameterHandler class provides an ros-indipendent
 * implementation of
 * ParameterInterface. It is intended to be used in gtest which are not
 * rostests. It is
 * possible to add typed parameters. Mode-switches are not supported. This
 * should be no
 * problem because the very most gtests should be mode-agnostic.
 */
class DummyParameterHandler : public ParameterInterface {

 private:
  template <typename T>
  using ParamMap = std::map<std::string, T>;

  template <typename T>
  ParamMap<T> &getParameterMap();

  template <typename T>
  const ParamMap<T> &getParameterMap() const {
    return const_cast<ParamMap<T> &>(
        (const_cast<DummyParameterHandler *>(this))->getParameterMap<T>());
  }

  template <typename T>
  void ensureExistance(const ParameterString<T> &param) const {
    if (!common::contains(getParameterMap<T>(), param)) {
      throw std::invalid_argument(
          "parameter: " + param +
          " does not exists (might not have the right type).");
    }
  }

  template <typename T>
  void registrate(const ParameterString<T> &param) {
    ensureExistance(param);
  }

  template <typename T>
  T get(const ParameterString<T> &param) const {
    ensureExistance(param);
    return getParameterMap<T>().at(param);
  }

 public:
  // ParameterInterface interface
  void registerParam(const ParameterString<bool> &parameter) override REGISTRATE
      void registerParam(const ParameterString<int> &parameter) override REGISTRATE
      void registerParam(const ParameterString<double> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::string> &parameter) override REGISTRATE

      void registerParam(const ParameterString<std::vector<bool>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::vector<int>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::vector<double>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::vector<std::string>> &parameter) override REGISTRATE

      void registerParam(const ParameterString<std::map<std::string, bool>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::map<std::string, int>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::map<std::string, double>> &parameter) override REGISTRATE
      void registerParam(const ParameterString<std::map<std::string, std::string>> &parameter) override REGISTRATE

      bool getParam(const ParameterString<bool> &parameter) const override GET
      int getParam(const ParameterString<int> &parameter) const override GET
      double getParam(const ParameterString<double> &parameter) const override GET std::string
      getParam(const ParameterString<std::string> &parameter) const override GET

      std::vector<bool> getParam(const ParameterString<std::vector<bool>> &parameter) const override
      GET std::vector<int> getParam(const ParameterString<std::vector<int>> &parameter) const override
      GET std::vector<double> getParam(const ParameterString<std::vector<double>> &parameter)
          const override GET std::vector<std::string> getParam(
              const ParameterString<std::vector<std::string>> &parameter) const override GET

      std::map<std::string, bool> getParam(const ParameterString<std::map<std::string, bool>> &parameter)
          const override GET std::map<std::string, int> getParam(
              const ParameterString<std::map<std::string, int>> &parameter)
              const override GET std::map<std::string, double> getParam(
                  const ParameterString<std::map<std::string, double>> &parameter) const
      override GET std::map<std::string, std::string> getParam(
          const ParameterString<std::map<std::string, std::string>> &parameter) const override GET

      template <class T>
      void addParam(ParameterString<T> parameter, T value) {
    getParameterMap<T>()[parameter] = std::move(value);
  }

 private:
  ParamMap<bool> bool_params;
  ParamMap<int> int_params;
  ParamMap<double> double_params;
  ParamMap<std::string> string_params;

  ParamMap<std::vector<bool>> bool_vector_params;
  ParamMap<std::vector<int>> int_vector_params;
  ParamMap<std::vector<double>> double_vector_params;
  ParamMap<std::vector<std::string>> string_vector_params;

  template <typename T>
  using MapParam = std::map<std::string, T>;

  ParamMap<MapParam<bool>> bool_map_params;
  ParamMap<MapParam<int>> int_map_params;
  ParamMap<MapParam<double>> double_map_params;
  ParamMap<MapParam<std::string>> string_map_params;
};

ADD_GET_PARAM_MAP(bool, bool_params)
ADD_GET_PARAM_MAP(int, int_params)
ADD_GET_PARAM_MAP(double, double_params)
ADD_GET_PARAM_MAP(std::string, string_params)

ADD_GET_PARAM_MAP(std::vector<bool>, bool_vector_params)
ADD_GET_PARAM_MAP(std::vector<int>, int_vector_params)
ADD_GET_PARAM_MAP(std::vector<double>, double_vector_params)
ADD_GET_PARAM_MAP(std::vector<std::string>, string_vector_params)

ADD_GET_PARAM_MAP(DummyParameterHandler::MapParam<bool>, bool_map_params)
ADD_GET_PARAM_MAP(DummyParameterHandler::MapParam<int>, int_map_params)
ADD_GET_PARAM_MAP(DummyParameterHandler::MapParam<double>, double_map_params)
ADD_GET_PARAM_MAP(DummyParameterHandler::MapParam<std::string>, string_map_params)


#endif  // PARAMETER_HANDLER_DUMMY_H
