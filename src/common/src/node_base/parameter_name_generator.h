#ifndef PARAMETER_NAME_GENERATOR_H
#define PARAMETER_NAME_GENERATOR_H

#include <string>

namespace common {
namespace node_base_detail {
/*!
 * \brief The ParameterNameGenerator class
 *
 * This class provides the ability to insert a driving-mode subnamespace in a
 * given parameter path. Relative Paths as well as absolute paths are possible -
 * currently driving-mode change is just for parameters at node or package level
 * possible.
 */
class ParameterNameGenerator {
 public:
  /*!
   * \brief ParameterNameGenerator generates a new Instance of
   * ParameterNameGenerator.
   *
   * \param parameter_name the parameter name/path to insert the driving-mode
   * subnamespace in.
   */
  ParameterNameGenerator(const std::string &parameter_name, const std::string& parent_ns) {

    if (parameter_name[0] == '/') {
      // starting with a slash indicates, that the parameter is not on node level
      const std::size_t name_space_position = parameter_name.find(parent_ns);
      //an empty namespace can be ignored; parnet_ns == "/" means there is no package namespace
      if ((parent_ns.size() > 1)  && name_space_position == 0) {
        //parameter_name starts with nodes namespace, so we got an package level parameter
        path_ = parameter_name.substr(0, parent_ns.length() + 1); //+1 for taking the slash
        name_ = parameter_name.substr(parent_ns.length() + 1, parameter_name.length() - parent_ns.length());
      } else {
        //string starts with a slash but not with the nodes namespace, so we got a global parameter
        path_ = "/";
        //we need to cut the starting slash off, beacuse in getExtendedName we will add it again
        name_ = parameter_name.substr(1, parameter_name.length() - 1);
      }
    } else {
      // string starts not with a slash, so we got a parameter on node level.
      name_ = parameter_name;
      path_ = "";
    }
  }

  /*!
   * \brief getExtendedName inserts the given sub_namespace in the Objects
   * parameter_path.
   * \param sub_namespace the subnamespace to be inserted.
   * \return the new path with correctly inserted subnamspace
   */
  std::string getExtendedName(const std::string &sub_namespace) const {
    // path_ and name_ have been created in the constructor in a way, that the
    // construction of the output string is always the same.
    return path_ + sub_namespace + "/" + name_;
  }

 private:
  /*!
   * \brief name_ the name of the parameter it self
   */
  std::string name_;
  /*!
   * \brief path_ the path to the parameter (if not node namespace relative)
   */
  std::string path_;
};

} // namespace node_base_detail;
} // namespace common;

#endif  // PARAMETER_NAME_GENERATOR_H
