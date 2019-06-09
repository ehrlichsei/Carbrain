#include "look_at_visitor.h"
THIRD_PARTY_HEADERS_BEGIN
#include <tf2_eigen/tf2_eigen.h>
#include <numeric>
THIRD_PARTY_HEADERS_END
#include "common/basic_statistics_eigen.h"
#include "common/best_score.h"

namespace environmental_model {

const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_RIGHT_MIN_X(
    "junction_roi_right_min_x");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_RIGHT_MIN_Y(
    "junction_roi_right_min_y");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_RIGHT_MAX_X(
    "junction_roi_right_max_x");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_RIGHT_MAX_Y(
    "junction_roi_right_max_y");

const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_CENTER_MIN_X(
    "junction_roi_center_min_x");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_CENTER_MIN_Y(
    "junction_roi_center_min_y");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_CENTER_MAX_X(
    "junction_roi_center_max_x");
const ParameterString<double> LookAtVisitorParameter::PARAM_JUNCTION_ROI_CENTER_MAX_Y(
    "junction_roi_center_max_y");

const ParameterString<double> LookAtVisitorParameter::PARAM_ROAD_CLOSURE_ROI_WIDTH(
    "road_closure_roi_width");

const ParameterString<double> LookAtVisitorParameter::PARAM_CROSSWALK_ROI_WIDTH(
    "crosswalk_roi_width");
const ParameterString<double> LookAtVisitorParameter::PARAM_CROSSWALK_ROI_HEIGHT(
    "crosswalk_roi_height");
const ParameterString<double> LookAtVisitorParameter::PARAM_CROSSWALK_LOOK_AT_DIST(
    "crosswalk_look_at_dist");

LookAtVisitor::LookAtVisitor(const std::shared_ptr<LookAtVisitorParameter> &parameter,
                             const perception_msgs::Obstacles &obstacles_msg,
                             const std::shared_ptr<DrivingCorridor> &driving_corridor,
                             const Eigen::Affine3d &vehicle_pose)

    : parameter_(parameter),
      obstacles_msg_(obstacles_msg),
      driving_corridor_(driving_corridor),
      vehicle_pose_(vehicle_pose) {
  obstacle_rois_.header.stamp = obstacles_msg.header.stamp;
  obstacle_rois_.header.frame_id = "world";
  pedestrian_rois_.header = obstacle_rois_.header;
}

void LookAtVisitor::visit(Unidentified & /*road_object*/,
                          TrackingElement & /*tracking_element*/) {}

void LookAtVisitor::visit(Obstacle & /*road_object*/, TrackingElement & /*tracking_element*/) {
}

void LookAtVisitor::visit(Junction &road_object, TrackingElement &tracking_element) {
  if (road_object.getMsg().junction_type != perception_msgs::Junction::TYPE_GIVEWAY_RIGHT &&
      road_object.getMsg().junction_type != perception_msgs::Junction::TYPE_STOPLINE_RIGHT) {
    return;
  }
  if ((vehicle_pose_.inverse() * road_object.getPose().translation()).x() < 0) {
    return;
  }
  addObstacleRoiJunction(
      road_object.getPose(), tracking_element.getId(), parameter_->junction_right_area_roi_);
  addObstacleRoiJunction(
      road_object.getPose(), tracking_element.getId(), parameter_->junction_center_area_roi_);
  if (road_object.getObstacleWaitingScore() != 0) {
    return;
  }
  const double max_certainty =
      std::max(getBestObstacleScoreInRoi(parameter_->junction_right_area_roi_,
                                         road_object.getPose()),
               getBestObstacleScoreInRoi(parameter_->junction_center_area_roi_,
                                         road_object.getPose()));
  if (max_certainty > road_object.getObstacleWaitingScore()) {
    road_object.setObstacleWaitingScore(max_certainty);
  }
}


void LookAtVisitor::visit(Crosswalk &road_object, TrackingElement &tracking_element) {
  if (driving_corridor_->empty()) {
    return;
  }
  const Eigen::Vector3d position = road_object.getPose().translation();
  const double dist_vehicle_to_crosswalk = (vehicle_pose_.inverse() * position).x();
  if (dist_vehicle_to_crosswalk < parameter_->crosswalk_look_at_dist) {
    return;
  }
  const auto nearest_gate = common::min_score(*driving_corridor_, [&](const auto &gate) {
    return (gate.getLaneCenter() - position).norm();
  });
  const Eigen::Vector3d end_crosswalk =
      road_object.getPose() * Eigen::Vector3d(parameter_->height_roi, 0, 0);
  const auto nearest_gate_end = common::min_score(
      std::next(nearest_gate), driving_corridor_->end(), [&](const auto &gate) {
        return (gate.getLaneCenter() - end_crosswalk).norm();
      });

  // calculate direction
  const auto directions =
      [](const Gate &gate) { return gate.getVectorLeftToRight().normalized(); };
  const Eigen::Vector3d sum_directions = common::sum<Eigen::Vector3d>(
      boost::make_iterator_range(nearest_gate, nearest_gate_end) |
      boost::adaptors::transformed(directions));
  const Eigen::Vector3d direction = sum_directions.normalized();

  const double width_roi = nearest_gate->getWidth() + 0.4;
  const double width_roi_extension = 0.4;
  // fixme check here if everything is alright
  Eigen::Affine3d roi_pose =
      Eigen::Translation3d(nearest_gate->getCenter()) *
      Eigen::Quaterniond::FromTwoVectors(-Eigen::Vector3d::UnitY(), direction);

  // set roi translation to the center
  roi_pose.translation() +=
      roi_pose.linear() * (parameter_->height_roi / 2.0 * Eigen::Vector3d::UnitX());
  pedestrian_rois_.regions.push_back(createLookAtRegion(roi_pose,
                                                        width_roi + width_roi_extension,
                                                        parameter_->height_roi,
                                                        tracking_element.getId()));
}

void LookAtVisitor::visit(RoadClosure &road_object, TrackingElement &tracking_element) {
  auto msg = road_object.getMsg();
  const double extra_length_before = 0.2;
  const double extra_length_after = 0.7;
  const double length = (road_object.getEnd() - road_object.getStart()).norm() +
                        extra_length_before + extra_length_after;
  // roi width is equal to lane width
  if (driving_corridor_->empty()) {
    return;
  }
  const double max_dist_obstacle_to_start_road_closure = 1.0;
  if ((vehicle_pose_.inverse() * road_object.getStart()).x() <
      -max_dist_obstacle_to_start_road_closure) {
    return;
  }
  auto nearest_gate_start = common::min_score(
      *driving_corridor_,
      [&](const auto &gate) {
        return (gate.getLaneCenter() - road_object.getStart()).norm();
      });
  auto nearest_gate_end =
      common::min_score(nearest_gate_start,
                        driving_corridor_->end(),
                        [&](const auto &gate) {
                          return (gate.getLaneCenter() - road_object.getEnd()).norm();
                        });

  // calculate direction
  const auto directions =
      [](const auto &gate) { return gate.getVectorLeftToRight().normalized(); };
  const Eigen::Vector3d sum_directions = common::sum<Eigen::Vector3d>(
      boost::make_iterator_range(nearest_gate_start, nearest_gate_end) |
      boost::adaptors::transformed(directions));
  const Eigen::Vector3d direction = sum_directions.normalized();

  const double roi_width = nearest_gate_start->getWidth() / 2.0;
  ROI roi(-length / 2.0, length / 2.0, -roi_width / 2.0, roi_width / 2.0);
  // roi_width);
  // roi_pose.translation() =
  // roi_pose * Eigen::Vector3d{0.5 * (roi.min_x_ + roi.max_x_), 1.5 *
  // roi_width, 0.0};
  const Eigen::Vector3d avg_left_pole =
      0.5 * (nearest_gate_start->getLeftPole() + nearest_gate_end->getLeftPole());
  const Eigen::Vector3d avg_right_pole =
      0.5 * (nearest_gate_start->getRightPole() + nearest_gate_end->getRightPole());
  auto roi_pose =
      Eigen::Translation3d((0.75 * avg_left_pole + 0.25 * avg_right_pole)) *
      Eigen::Quaterniond::FromTwoVectors(-Eigen::Vector3d::UnitY(), direction);
  roi_pose = roi_pose * Eigen::Translation3d(
                            (extra_length_after - extra_length_before) / 2.0, 0, 0);

  obstacle_rois_.regions.push_back(
      createLookAtRegion(roi_pose, roi_width, length, tracking_element.getId()));
  if (road_object.getObstacleWaitingScore() != 0) {
    return;
  }
  const double max_certainty = getBestObstacleScoreInRoi(roi, roi_pose);
  if (max_certainty > road_object.getObstacleWaitingScore()) {
    road_object.setObstacleWaitingScore(max_certainty);
  }
}

void LookAtVisitor::visit(SpeedLimit & /*road_object*/,
                          TrackingElement & /*tracking_element*/) {}

void LookAtVisitor::visit(Arrow & /*road_object*/, TrackingElement & /*tracking_element*/) {}

void LookAtVisitor::visit(StartLine & /*road_object*/,
                          TrackingElement & /*tracking_element*/) {}

perception_msgs::LookForObstaclesGoal LookAtVisitor::getObstacleActionGoal() {
  perception_msgs::LookForObstaclesGoal goal;
  goal.obstacle_regions = obstacle_rois_;
  return goal;
}

perception_msgs::LookForPedestriansGoal LookAtVisitor::getPedestrianActionGoal() {
  perception_msgs::LookForPedestriansGoal goal;
  goal.pedestrian_regions = pedestrian_rois_;
  return goal;
}

void LookAtVisitor::addObstacleRoiJunction(const Eigen::Affine3d &pose,
                                           unsigned int id,
                                           const ROI &roi) {
  auto roi_pose = getRoiCenterPose(pose, roi);
  roi_pose.linear() =
      Eigen::AngleAxisd{M_PI_2, Eigen::Vector3d::UnitZ()}.toRotationMatrix() *
      roi_pose.linear();
  obstacle_rois_.regions.push_back(createLookAtRegion(
      roi_pose, (roi.max_x_ - roi.min_x_), (roi.max_y_ - roi.min_y_), id));
}

perception_msgs::LookAtRegion LookAtVisitor::createLookAtRegion(const Eigen::Affine3d &pose,
                                                                const double width,
                                                                const double height,
                                                                unsigned int id) {
  perception_msgs::LookAtRegion roi;
  roi.pose = tf2::toMsg(pose);
  roi.rect.x = height;
  roi.rect.y = width;
  roi.id = id;
  return roi;
}

double LookAtVisitor::getBestObstacleScoreInRoi(const LookAtVisitor::ROI &roi,
                                                const Eigen::Affine3d &road_object_pose) {
  if (obstacles_msg_.sub_messages.empty()) {
    return 0;
  }
  const auto certaintyOfObstacleInRois = [&, this](const auto &obstacle_msg) {
    if (this->obstacleInRoi(obstacle_msg, road_object_pose, roi)) {
      return obstacle_msg.certainty;
    }
    return 0.0f;
  };
  return certaintyOfObstacleInRois(*common::max_score(
      obstacles_msg_.sub_messages, certaintyOfObstacleInRois));
}

perception_msgs::LookAtRegions LookAtVisitor::getObstacleRois() const {
  return obstacle_rois_;
}

perception_msgs::LookAtRegions LookAtVisitor::getPedestriansRois() const {
  return pedestrian_rois_;
}

bool LookAtVisitor::hasObstacleRois() const {
  return !obstacle_rois_.regions.empty();
}

bool LookAtVisitor::hasPedestrianRois() const {
  return !pedestrian_rois_.regions.empty();
}

Eigen::Affine3d LookAtVisitor::getRoiCenterPose(const Eigen::Affine3d &road_object_pose,
                                                const ROI &roi) {
  Eigen::Affine3d roi_pose = road_object_pose;
  roi_pose.translation() =
      roi_pose *
      (0.5 * Eigen::Vector3d((roi.min_x_ + roi.max_x_), (roi.min_y_ + roi.max_y_), 0));
  return roi_pose;
}

bool LookAtVisitor::obstacleInRoi(const perception_msgs::Obstacle &obstacle,
                                  const Eigen::Affine3d &road_object_pose,
                                  const ROI &roi) {
  if (obstacle.vertices.empty()) {
    return false;
  }
  const Eigen::Vector3d sum_obstacle_vertices =
      std::accumulate(obstacle.vertices.begin(),
                      obstacle.vertices.end(),
                      Eigen::Vector3d(0, 0, 0),
                      [](const auto current_sum, const auto &obstacle_point) {
                        Eigen::Vector3d point = Eigen::Vector3d::Zero();
                        tf2::fromMsg(obstacle_point, point);
                        return current_sum + point;
                      });
  const Eigen::Vector3d obstacle_center =
      sum_obstacle_vertices / obstacle.vertices.size();
  const auto roi_pose = getRoiCenterPose(road_object_pose, roi);
  const Eigen::Vector3d obstacle_center_in_junction_frame = roi_pose * obstacle_center;
  return obstacle_center_in_junction_frame.x() < roi.max_x_ &&
         obstacle_center_in_junction_frame.x() > roi.min_x_ &&
         obstacle_center_in_junction_frame.y() < roi.max_y_ &&
         obstacle_center_in_junction_frame.y() > roi.min_y_;
}

LookAtVisitor::ROI::ROI(const double min_x, const double max_x, const double min_y, const double max_y)
    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y) {}

LookAtVisitorParameter::LookAtVisitorParameter(common::node_base::ParameterInterface *const parameters) {
  parameters->registerParam(PARAM_JUNCTION_ROI_RIGHT_MIN_X);
  parameters->registerParam(PARAM_JUNCTION_ROI_RIGHT_MIN_Y);
  parameters->registerParam(PARAM_JUNCTION_ROI_RIGHT_MAX_X);
  parameters->registerParam(PARAM_JUNCTION_ROI_RIGHT_MAX_Y);

  parameters->registerParam(PARAM_JUNCTION_ROI_CENTER_MIN_X);
  parameters->registerParam(PARAM_JUNCTION_ROI_CENTER_MIN_Y);
  parameters->registerParam(PARAM_JUNCTION_ROI_CENTER_MAX_X);
  parameters->registerParam(PARAM_JUNCTION_ROI_CENTER_MAX_Y);
  parameters->registerParam(PARAM_ROAD_CLOSURE_ROI_WIDTH);
  parameters->registerParam(PARAM_CROSSWALK_ROI_WIDTH);
  parameters->registerParam(PARAM_CROSSWALK_ROI_HEIGHT);
  parameters->registerParam(PARAM_CROSSWALK_LOOK_AT_DIST);
  updateParameter(parameters);
}

void LookAtVisitorParameter::updateParameter(common::node_base::ParameterInterface *const parameters) {
  junction_right_area_roi_ =
      LookAtVisitor::ROI(parameters->getParam(PARAM_JUNCTION_ROI_RIGHT_MIN_X),
                         parameters->getParam(PARAM_JUNCTION_ROI_RIGHT_MAX_X),
                         parameters->getParam(PARAM_JUNCTION_ROI_RIGHT_MIN_Y),
                         parameters->getParam(PARAM_JUNCTION_ROI_RIGHT_MAX_Y));
  junction_center_area_roi_ =
      LookAtVisitor::ROI(parameters->getParam(PARAM_JUNCTION_ROI_CENTER_MIN_X),
                         parameters->getParam(PARAM_JUNCTION_ROI_CENTER_MAX_X),
                         parameters->getParam(PARAM_JUNCTION_ROI_CENTER_MIN_Y),
                         parameters->getParam(PARAM_JUNCTION_ROI_CENTER_MAX_Y));

  height_roi = parameters->getParam(PARAM_CROSSWALK_ROI_HEIGHT);
  crosswalk_look_at_dist = parameters->getParam(PARAM_CROSSWALK_LOOK_AT_DIST);
}
}  // namespace environmental_model
