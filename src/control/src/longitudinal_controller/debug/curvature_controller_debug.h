#ifndef CURVATURE_CONTROLLER_DEBUG_H
#define CURVATURE_CONTROLLER_DEBUG_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <ros/publisher.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/range/adaptor/indexed.hpp>
THIRD_PARTY_HEADERS_END

#include "../concrete_longitudinal_controller.h"
#include "longitudinal_controller_node_debug.h"

class CurvatureControllerDebug : public CurvatureController {
 public:
  CurvatureControllerDebug(CurvatureController&& curvature_controller,
                           LongitudinalControllerNode& longitudinal_controller_node);

  virtual double calculateSpeed(double measured_speed,
                                const ros::Duration& dur,
                                int driving_direction) override;

 private:
  LongitudinalControllerNode& longitudinal_controller_node;


  template <typename InputPlotPointType, typename Container = std::vector<InputPlotPointType>>
  class PathPlot {
   public:
    PathPlot(ros::NodeHandle& nh,
             std::string visualization_name,
             double r,
             double g,
             double b,
             const std::function<double(const InputPlotPointType&, const common::Path<>&)>& plotf,
             const std::function<double(const InputPlotPointType&)>& arclf)
        : base_id(base_id_count++),
          name(visualization_name),
          node_handle(nh),
          plotf(plotf),
          arclf(arclf),
          r(r),
          g(g),
          b(b) {
      pub = nh.advertise<visualization_msgs::MarkerArray>(
          "debug/curvature_controller_" + visualization_name + "_visualization", 1);
    }

    void plot(const std_msgs::Header& header,
              const Container& input_vector,
              const common::Path<>& path) {
      if (input_vector.empty()) {
        return;
      }

      visualization_msgs::Marker plot_marker;
      plot_marker.header = header;
      plot_marker.ns = "curvature_controller";
      plot_marker.id = base_id * 10'000;
      plot_marker.type = visualization_msgs::Marker::LINE_LIST;
      plot_marker.action = visualization_msgs::Marker::MODIFY;
      plot_marker.color.r = r;
      plot_marker.color.g = g;
      plot_marker.color.b = b;
      plot_marker.color.a = 0.4;
      plot_marker.lifetime = ros::Duration();
      plot_marker.scale.x = 0.01;
      plot_marker.pose.orientation.x = 0.;
      plot_marker.pose.orientation.y = 0.;
      plot_marker.pose.orientation.z = 0.;
      plot_marker.pose.orientation.w = 1.;

      plot_marker.points.reserve(input_vector.size() * 2);


      visualization_msgs::MarkerArray plot_markers;
      const int text_subsampling =
          node_handle.param("visualization_text_subsampling", 10);

      visualization_msgs::Marker text_marker_line;
      text_marker_line.header = header;
      text_marker_line.ns = "curvature_controller";
      text_marker_line.id = base_id * 10'000 + 1;
      text_marker_line.type = visualization_msgs::Marker::LINE_LIST;
      text_marker_line.action = visualization_msgs::Marker::MODIFY;
      text_marker_line.color.r = 0;
      text_marker_line.color.g = 0;
      text_marker_line.color.b = 0;
      text_marker_line.color.a = 1.0;
      text_marker_line.lifetime = ros::Duration();
      text_marker_line.scale.x = 0.004;

      text_marker_line.points.reserve(input_vector.size() / text_subsampling * 2);


      int highest_text_index = 0;

      static int highest_previous_text_index = 0;
      for (auto v : input_vector | boost::adaptors::indexed(0)) {
        const double vertical_scale =
            node_handle.param(name + "_visualization_vertical_scale", 1.0);

        const Eigen::Vector2d path_point = path(arclf(v.value()));

        Eigen::Vector3d base_point;
        base_point << path_point, 0.0;
        plot_marker.points.push_back(tf2::toMsg(base_point));

        Eigen::Vector3d plot_value_point;
        plot_value_point << path_point, vertical_scale* plotf(v.value(), path);
        plot_marker.points.push_back(tf2::toMsg(plot_value_point));


        if (v.index() % text_subsampling == 0) {
          visualization_msgs::Marker text_marker;
          text_marker.header = header;
          text_marker.ns = "curvature_controller";
          text_marker.id = base_id * 10'000 + 80 + v.index();
          text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          text_marker.action = visualization_msgs::Marker::MODIFY;
          text_marker.color.r = r;
          text_marker.color.g = g;
          text_marker.color.b = b;
          text_marker.color.a = 1.0;
          text_marker.lifetime = ros::Duration();
          text_marker.scale.z = 0.08;
          const Eigen::Vector3d text_position =
              plot_value_point + (plot_value_point.z() < 0
                                      ? -Eigen::Vector3d(0., 0., 0.2)
                                      : Eigen::Vector3d(0., 0., 0.2));
          text_marker.pose.position = tf2::toMsg(text_position);

          std::stringstream stream;
          stream << std::fixed << std::setprecision(2) << plotf(v.value(), path);
          text_marker.text = stream.str();

          plot_markers.markers.push_back(text_marker);

          highest_text_index = text_marker.id;


          text_marker_line.points.push_back(tf2::toMsg(plot_value_point));
          text_marker_line.points.push_back(tf2::toMsg(Eigen::Vector3d(
              text_position + (plot_value_point.z() < 0 ? Eigen::Vector3d(0., 0., 0.02)
                                                        : -Eigen::Vector3d(0., 0., 0.02)))));
        }
      }

      visualization_msgs::Marker description_marker;
      description_marker.header = header;
      description_marker.ns = "curvature_controller";
      description_marker.id = base_id * 10'000 + 3;
      description_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      description_marker.action = visualization_msgs::Marker::MODIFY;
      description_marker.color.r = r;
      description_marker.color.g = g;
      description_marker.color.b = b;
      description_marker.color.a = 1.0;
      description_marker.lifetime = ros::Duration();
      description_marker.scale.z = 0.1;
      description_marker.pose.position = plot_marker.points.back();
      description_marker.pose.position.z =
          std::max(0.0, description_marker.pose.position.z) + 0.4;
      description_marker.text = name;

      plot_markers.markers.push_back(description_marker);

      // delete old markers
      for (int id = highest_text_index; id <= highest_previous_text_index; id += text_subsampling) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "curvature_controller";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::DELETE;

        plot_markers.markers.push_back(marker);
      }

      highest_previous_text_index = highest_text_index;

      plot_markers.markers.push_back(plot_marker);
      plot_markers.markers.push_back(text_marker_line);

      pub.publish(plot_markers);
    }

   private:
    static long base_id_count;
    const long base_id;

    const std::string name;
    ros::Publisher pub;
    ros::NodeHandle node_handle;
    const std::function<double(const InputPlotPointType&, const common::Path<>&)> plotf;
    const std::function<double(const InputPlotPointType&)> arclf;
    const double r;
    const double g;
    const double b;
  };



  struct ArcLengthParameterizedCurvature {
    ArcLengthParameterizedCurvature(double curvature, double arc_length)
        : curvature(curvature), arc_length(arc_length) {}
    operator double() const { return curvature; }
    operator double&() { return curvature; }

    double curvature = 0.0;
    double arc_length = 0.0;
  };

  std::vector<ArcLengthParameterizedCurvature> curvature;

  void calculate_curvature(const common::Path<>& path);

  PathPlot<ArcLengthParameterizedSpeed, std::vector<ArcLengthParameterizedSpeed>> plot_backward_calculation_speed;
  PathPlot<ArcLengthParameterizedSpeed, std::vector<ArcLengthParameterizedSpeed>> plot_forward_calculation_speed;
  PathPlot<ArcLengthParameterizedCurvature, std::vector<ArcLengthParameterizedCurvature>> plot_curvature;
};

template <typename T1, typename T2>
long CurvatureControllerDebug::PathPlot<T1, T2>::base_id_count = 1;

#endif  // CURVATURE_CONTROLLER_DEBUG_H
