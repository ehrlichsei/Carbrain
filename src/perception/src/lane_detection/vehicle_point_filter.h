#ifndef VEHICLE_POINT_FILTER_H
#define VEHICLE_POINT_FILTER_H
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/console.h>
#include <memory>
THIRD_PARTY_HEADERS_END

#include "common/parameter_interface.h"
#include "perception_types.h"
/*!
 * \brief The VehiclePointFilter class
 *
 * encapsules clustering based filtering of vehicle points. It is possible to
 * create mutliple instances of this class with different parameter sets. The
 * parameters are registered and searched under a specific sub-namespace.
 */
class VehiclePointFilter {
 public:
  typedef std::shared_ptr<VehiclePointFilter> Ptr;
  /*!
    * \brief VehiclePointFilter
    *
    * ctos of VehiclePointFilter
    *
    * \param parameter_interface the parameter_interface.
    * \param sub_name_space the sub-namespace for the specifice parameters of
    * this instance
    */
  VehiclePointFilter(ParameterInterface* parameter_interface, const std::string& sub_name_space);

  /*!
   * \brief filterPoints
   *
   * performs the filtering
   *
   * \param in the input.
   * \param out the output.
   */
  void filterPoints(const VehiclePoints& in, VehiclePoints* out) const;

 private:
  /*!
   * \brief registerParameters
   *
   * registeres the Parameter of this object.
   *
   * \param parameter_interface the parameter_interface.
   */
  void registerParameters(ParameterInterface* parameter_interface);

  VehiclePoints prefilter(const VehiclePoints& ps) const;

  /*!
   * \brief parameter_ptr
   *
   * the pointer to the parameter interface
   */
  const ParameterInterface* parameter_ptr;
  /*!
   * \brief POINT_FILTERING_DIST_THRESH
   *
   * the distance threshold used by the clustering algorithm.
   */
  const ParameterString<double> POINT_FILTERING_DIST_THRESH;
  /*!
   * \brief POINT_FILTERING_COUNT_TRESH
   *
   * threshold for taking a cluster by ratio of the number of all points
   */
  const ParameterString<int> POINT_FILTERING_COUNT_TRESH;
  /*!
   * \brief POINT_FILTERING_COUNT_THRESH_RATIO
   *
   * minimal size of a cluster to be taken
   */
  const ParameterString<double> POINT_FILTERING_COUNT_THRESH_RATIO;
  /*!
   * \brief POINT_FILTERING_NEAR_FIELD_DIST
   *
   * clusters are taken if their nearest point is in this distance.
   */
  const ParameterString<double> POINT_FILTERING_NEAR_FIELD_DIST;
  /*!
   * \brief POINT_FILTERING_PRE_FITLERING_COUNT_TRESH
   *
   * threshold for taking a cluster in prefiltering.
   */
  const ParameterString<int> POINT_FILTERING_PRE_FITLERING_COUNT_TRESH;
  /*!
   * \brief POINT_FILTERING_PRE_FILTERING_DIST_THRESH
   *
   * the distance threshold used by the clustering algorithm in prefiltering.
   */
  const ParameterString<double> POINT_FILTERING_PRE_FILTERING_DIST_THRESH;
};

#endif  // VEHICLE_POINT_FILTER_H
