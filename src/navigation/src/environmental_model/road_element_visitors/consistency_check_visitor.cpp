#include "consistency_check_visitor.h"
#include <common/macros.h>
#include "common/basic_statistics.h"
#include "common/basic_statistics_eigen.h"
#include "common/eigen_utils.h"
THIRD_PARTY_HEADERS_BEGIN
#include <boost/algorithm/cxx11/any_of.hpp>
#include <boost/algorithm/cxx11/none_of.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
THIRD_PARTY_HEADERS_END

namespace environmental_model {
void ConsistencyCheckVisitor::visit(Obstacle &road_object, TrackingElement &tracking_element) {
  Eigen::Vector3d obst_position;
  obst_position << road_object.getEstimatedPosition(), 0.0;
  const auto distanceToGate =
      [&](const auto &gate) { return (gate.getCenter() - obst_position).norm(); };
  const auto nearest_gate = common::min_score(driving_corridor_approx_, distanceToGate);

  const bool on_left_lane = nearest_gate->toParam(obst_position) < 0.5;


  obstacle_informations_.emplace_back(&tracking_element,
                                      &road_object,
                                      on_left_lane,
                                      obst_position,
                                      road_object.getDynamicState().prob_dynamic);

  std::vector<double> vertices_gate_params;
  const auto vertex_to_gate_param = [&](const auto &vertex) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    tf2::fromMsg(vertex, v);
    return nearest_gate->toParam(v);
  };
  auto msg = road_object.getMsg();
  boost::transform(msg.vertices, std::back_inserter(vertices_gate_params), vertex_to_gate_param);
  if (vertices_gate_params.size() > 2) {
    const double left_gate_param =
        *std::min_element(vertices_gate_params.begin(), vertices_gate_params.end());
    const double right_gate_param =
        *std::max_element(vertices_gate_params.begin(), vertices_gate_params.end());
    obstacle_informations_.back().dist_to_left_gate_border =
        left_gate_param * nearest_gate->getWidth();
    obstacle_informations_.back().dist_to_right_gate_border =
        (1 - right_gate_param) * nearest_gate->getWidth();
  }
}

void ConsistencyCheckVisitor::visit(StartLine &road_object, TrackingElement &tracking_element) {
  if (getAngleInDegreeToCorridor(road_object.getPose()) > 45) {
    start_lines_to_delete_.insert(tracking_element.getId());
  }
}

void ConsistencyCheckVisitor::visit(Junction &road_object, TrackingElement &tracking_element) {
  junction_informations_.emplace_back(&tracking_element, &road_object);
  if (getAngleInDegreeToCorridor(road_object.getPose()) > 45) {
    junctions_to_delete_.insert(tracking_element.getId());
  }
}

void ConsistencyCheckVisitor::visit(Arrow &road_object, TrackingElement &tracking_element) {
  arrow_informations_.emplace_back(&tracking_element, &road_object);
}

void ConsistencyCheckVisitor::makeModelValid(
    const Eigen::Affine3d &vehicle_pose,
    const DrivingCorridor &driving_corridor,
    std::vector<TrackingElement> &tracking_elements,
    const std::deque<std::pair<Eigen::Vector3d, ros::Time>> &last_vehicle_positions,
    NoPassingZones &no_passing_zones) {
  if (driving_corridor.size() < 2) {
    return;
  }
  ConsistencyCheckVisitor consistency_check_visitor(driving_corridor);
  consistency_check_visitor.deleteTrackingElementsNotInCorridor(tracking_elements);

  visitor_helper::visitElements(consistency_check_visitor, tracking_elements);
  consistency_check_visitor.increaseProbMatchingJunctions();
  consistency_check_visitor.deleteInvalidStartLines(tracking_elements);
  consistency_check_visitor.deleteInvalidJunctions(tracking_elements);
  consistency_check_visitor.makeNoPassingZonesValid(
      vehicle_pose, last_vehicle_positions, no_passing_zones);

  // !!!!!! order releveant: makeObstaclePositionsValid invalidates the
  // obstacles !!!!!!

  consistency_check_visitor.makeObstaclePositionsValid(tracking_elements);
}

void ConsistencyCheckVisitor::makeNoPassingZonesValid(
    const Eigen::Affine3d &vehicle_pose,
    const std::deque<std::pair<Eigen::Vector3d, ros::Time>> &last_vehicle_positions,
    NoPassingZones &no_passing_zones) {
  if (obstacle_informations_.empty() || last_vehicle_positions.size() < 3) {
    return;
  }
  const auto last_position = last_vehicle_positions.front();
  const auto current_position = last_vehicle_positions.back();
  const double velocity_mps = (current_position.first - last_position.first).norm() /
                              (current_position.second - last_position.second).sec;
  const Eigen::Affine3d to_vehicle = vehicle_pose.inverse();
  const auto distanceInFront = [&to_vehicle](const ObstacleInformation &info) {
    const double dist = (to_vehicle * info.position).x();
    return (dist < 0.0) ? std::numeric_limits<double>::max() : dist;
  };
  const ObstacleInformation next_obstacle =
      *common::min_score(obstacle_informations_, distanceInFront);
  if (distanceInFront(next_obstacle) == std::numeric_limits<double>::max()) {
    return;
  }
  if (next_obstacle.prob_dynamic < 0.05 && velocity_mps < 0.025) {
    if (no_passing_zones.hasNoPassingZones()) {
      ROS_ERROR_STREAM(
          "static obstacle in no passing zone detected. Resetting no passing "
          "zones.");
    }
    const int ignore_number_messages_after_reset = 90;
    no_passing_zones.reset(ignore_number_messages_after_reset);
  }
}



void ConsistencyCheckVisitor::makeObstaclePositionsValid(std::vector<TrackingElement> &tracking_elements) {

  const auto conflicting_ids = getIdsOfConflictingObstacles();
  if (conflicting_ids.empty()) {
    return;
  }
  ROS_WARN_STREAM("Obstacle locations conflict!!! delete "
                  << conflicting_ids.size() << " conflicting obstacles");
  const auto shouldDeleteElement = [&conflicting_ids](const auto &tracking_element) {
    return std::find(conflicting_ids.begin(),
                     conflicting_ids.end(),
                     tracking_element.getId()) != conflicting_ids.end();
  };
  boost::remove_erase_if(tracking_elements, shouldDeleteElement);
}

bool ConsistencyCheckVisitor::conflictingObstaclesAreSameDynamicObstacle(
    std::vector<unsigned int> &conflicting_ids,
    const ConsistencyCheckVisitor::ObstacleInformation &a,
    const ConsistencyCheckVisitor::ObstacleInformation &b) const {

  if (a.on_left_lane || b.on_left_lane) {
    return false;
  }

  if ((a.position - b.position).norm() > 0.5) {
    return false;
  }
  const auto checkValidDynamic = [](const ObstacleInformation &info) {
    return info.prob_dynamic > 0.7 &&
           info.tracking_element->maxProbNotUnidentified() > 0.7 &&
           !info.obstacle->isNew();
  };
  const auto checkValidNew = [](const ObstacleInformation &info) {
    return info.obstacle->isNew() && info.tracking_element->maxProbNotUnidentified() > 0.2;
  };


  boost::optional<const ObstacleInformation &> new_obstacle = boost::none;
  boost::optional<const ObstacleInformation &> other_obstacle = boost::none;
  if (checkValidNew(a) && checkValidDynamic(b)) {
    new_obstacle = a;
    other_obstacle = b;
  } else if (checkValidNew(b) && checkValidDynamic(a)) {
    new_obstacle = b;
    other_obstacle = a;
  }
  if (new_obstacle == boost::none || other_obstacle == boost::none) {
    return false;
  }

  ROS_WARN_STREAM_THROTTLE(0.3,
                           "distance between two obstacles too low. Assume "
                           "they are from the same dynamic obstacle.");
  other_obstacle->obstacle->addMsgOfConflictingObstacle(new_obstacle->obstacle->getMsg());
  conflicting_ids.push_back(new_obstacle->tracking_element->getId());
  return true;
}

double ConsistencyCheckVisitor::getAngleInDegreeToCorridor(const Eigen::Affine3d &pose) {
  const auto distanceToGate = [&](const auto &gate) {
    return (gate.getCenter() - pose.translation()).norm();
  };
  const auto nearest_gate = common::min_score(driving_corridor_approx_, distanceToGate);
  const Eigen::Affine3d gate_pose = nearest_gate->getTransformationFromGateFrame();

  // pose.x should be in driving direction and gate_pose.y is in driving
  // direction
  const double scalar_product = (pose.linear() * Eigen::Vector3d::UnitX())
                                    .dot(gate_pose.linear() * Eigen::Vector3d::UnitY());
  const double abs_angle = std::abs(std::acos(scalar_product));
  return common::eigen_utils::toDegree(abs_angle);
}

void ConsistencyCheckVisitor::deleteTrackingElementsNotInCorridor(std::vector<TrackingElement> &tracking_elements) {
  if (driving_corridor_approx_.size() < 2) {
    return;
  }

  const auto is_behind_last_gate = [this](const Eigen::Vector3d &point) {
    const double safety_extra_dist = -0.1;
    return (this->driving_corridor_approx_.back().getTransformationToGateFrame() *
            point).y() > safety_extra_dist;
  };
  const auto is_before_first_gate = [this](const Eigen::Vector3d &point) {
    const double safety_extra_dist = 0.1;
    return (this->driving_corridor_approx_.at(0).getTransformationToGateFrame() *
            point).y() < safety_extra_dist;
  };

  const auto isNotInCorridor = [&, this](const TrackingElement &element) {
    auto base_area_and_center = toEigen3d(element.getBaseArea());
    if (base_area_and_center.empty()) {
      return true;
    }
    base_area_and_center.emplace_back(common::sum(base_area_and_center) /
                                      base_area_and_center.size());
    if (boost::algorithm::any_of(base_area_and_center, is_behind_last_gate)) {
      // if a point is behind the last gate the element is probably valid but
      // the corridor is too short
      return false;
    }
    if (boost::algorithm::any_of(base_area_and_center, is_before_first_gate)) {
      // if a point is in front of the first gate the element will be deleted in
      // the EnvironemntalModel class and
      // shouldn't be deleted here
      return false;
    }
    return boost::algorithm::none_of(base_area_and_center,
                                     std::bind(&DrivingCorridor::isPointApproximateContained,
                                               &this->driving_corridor_approx_,
                                               std::placeholders::_1));
  };
  const auto size_before = tracking_elements.size();
  boost::remove_erase_if(tracking_elements, isNotInCorridor);

  if (size_before != tracking_elements.size()) {
    ROS_WARN_STREAM((size_before - tracking_elements.size())
                    << " elements outside of the corridor. Delete them!!!");
  }
}

void ConsistencyCheckVisitor::deleteInvalidStartLines(std::vector<TrackingElement> &tracking_elements) {
  if (start_lines_to_delete_.empty()) {
    return;
  }
  ROS_WARN_STREAM("Invalid start lines. Delete "
                  << start_lines_to_delete_.size() << " start lines.");
  const auto shouldDeleteElement = [this](const auto &tracking_element) {
    return std::find(this->start_lines_to_delete_.begin(),
                     this->start_lines_to_delete_.end(),
                     tracking_element.getId()) != this->start_lines_to_delete_.end();
  };
  boost::remove_erase_if(tracking_elements, shouldDeleteElement);
}

void ConsistencyCheckVisitor::deleteInvalidJunctions(std::vector<TrackingElement> &tracking_elements) {
  if (junctions_to_delete_.empty()) {
    return;
  }
  ROS_WARN_STREAM("Invalid junctions. Delete " << junctions_to_delete_.size() << " junctions.");
  const auto shouldDeleteElement = [this](const auto &tracking_element) {
    return std::find(this->junctions_to_delete_.begin(),
                     this->junctions_to_delete_.end(),
                     tracking_element.getId()) != this->junctions_to_delete_.end();
  };
  boost::remove_erase_if(tracking_elements, shouldDeleteElement);
}


ConsistencyCheckVisitor::ConsistencyCheckVisitor(const DrivingCorridor &driving_corridor) {
  driving_corridor_approx_.reserve(driving_corridor.size() / 4);
  for (size_t i = 0; i < driving_corridor.size() - 1; i += 4) {
    driving_corridor_approx_.push_back(driving_corridor.at(i));
  }
  driving_corridor_approx_.push_back(driving_corridor.back());
}

void ConsistencyCheckVisitor::increaseProbMatchingJunctions() {
  const auto start_left_junctions =
      std::partition(junction_informations_.begin(),
                     junction_informations_.end(),
                     [](const JunctionInformation &info) {
                       return info.junction->typeRightSide();
                     });
  const auto rigthMatchesToLeft =
      [](const JunctionInformation &right, const JunctionInformation &left) {
        const Eigen::Vector3d left_junction_in_right_junction =
            right.junction->getPose().inverse() * left.junction->getPose().translation();
        return left_junction_in_right_junction.x() > 0.4 &&
               left_junction_in_right_junction.x() < 1.5;

      };

  std::for_each(
      junction_informations_.begin(),
      start_left_junctions,
      [&, this](const JunctionInformation &right) {
        if (std::any_of(start_left_junctions,
                        this->junction_informations_.end(),
                        std::bind(rigthMatchesToLeft, right, std::placeholders::_1))) {
          right.tracking_element->scaleHighestProb(1.3);
          ROS_INFO_STREAM_THROTTLE(1.0,
                                   "increase prob for right junction because "
                                   "matching left junction was found");
        }
      });

  const auto rightJunctionMatchesToArrow = [](
      const JunctionInformation &right, const ArrowInformation &arrow_info) {
    const Eigen::Vector3d arrow_in_right_junction =
        right.junction->getPose().inverse() * arrow_info.arrow->getPose().translation();
    return arrow_in_right_junction.x() < 0.0 && arrow_in_right_junction.x() > -0.8;
  };
  std::for_each(
      junction_informations_.begin(),
      start_left_junctions,
      [&, this](const JunctionInformation &right) {
        for (const auto &arrow : arrow_informations_) {
          if (rightJunctionMatchesToArrow(right, arrow)) {
            right.tracking_element->scaleHighestProb(1.3);
            arrow.tracking_element->scaleHighestProb(1.3);
            ROS_INFO_STREAM_THROTTLE(
                1.0, "increase prob for matching right junction and arrow");
          }
        }
      });
}

std::vector<unsigned int> ConsistencyCheckVisitor::getIdsOfConflictingObstacles() {
  const Eigen::Vector3d corridor_direction =
      (driving_corridor_approx_.back().getCenter() -
       driving_corridor_approx_.at(0).getCenter()).normalized();
  const auto compareObstacleInformations = [&](const auto &a, const auto &b) {
    const Eigen::Vector3d from_A_to_B = b.position - a.position;
    return (from_A_to_B).dot(corridor_direction) > 0;
  };
  std::sort(obstacle_informations_.begin(), obstacle_informations_.end(), compareObstacleInformations);
  const auto toEigen3d = [](const auto &vertex) {
    Eigen::Vector3d v;
    tf2::fromMsg(vertex, v);
    return v;
  };


  const auto obstaclesConflict = [&](const auto a, const auto b) {
    std::vector<Eigen::Vector3d> a_vertices;
    auto a_msg = a->getMsg();
    auto b_msg = b->getMsg();
    boost::transform(a_msg.vertices, std::back_inserter(a_vertices), toEigen3d);
    std::vector<Eigen::Vector3d> b_vertices;
    boost::transform(b_msg.vertices, std::back_inserter(b_vertices), toEigen3d);
    for (const auto &a_vertex : a_vertices) {
      if (boost::algorithm::any_of(b_vertices,
                                   [&](const auto &b_vertex) {
                                     return (a_vertex - b_vertex).norm() < 0.4;
                                   })) {
        return true;
      }
    }
    return false;
  };

  std::vector<unsigned int> conflicting_ids;
  if (obstacle_informations_.size() < 2) {
    return conflicting_ids;
  }
  for (size_t i = 0; i < obstacle_informations_.size() - 1; ++i) {
    const double min_dist_to_one_border = 0.3;
    if (obstacle_informations_[i].dist_to_left_gate_border < min_dist_to_one_border &&
        obstacle_informations_[i].dist_to_right_gate_border < min_dist_to_one_border) {
      conflicting_ids.push_back(obstacle_informations_[i].tracking_element->getId());
      ROS_WARN_STREAM("obstacle width to high");
    }
    /*if (obstacle_informations_[i].on_left_lane == obstacle_informations_[i +
    1].on_left_lane) {
      continue;
    }*/
    if (!obstaclesConflict(obstacle_informations_[i].obstacle,
                           obstacle_informations_[i + 1].obstacle)) {
      continue;
    }
    if (conflictingObstaclesAreSameDynamicObstacle(
            conflicting_ids, obstacle_informations_[i], obstacle_informations_[i + 1])) {
      continue;
    }
    ROS_WARN_STREAM("distance between two obstacles too low");

    const auto deleteScore = [](const ObstacleInformation &info) {
      if (info.obstacle->stoppedPrediction()) {
        return 0.0;
      }
      if (info.obstacle->getNumberUpdatesWithoutNewMeasurement() > 25) {
        return 0.0;
      }
      return info.tracking_element->maxProbNotUnidentified();
    };

    const double factor_first_obstacle = 1.3;
    if (factor_first_obstacle * deleteScore(obstacle_informations_[i]) <
        deleteScore(obstacle_informations_[i + 1])) {
      conflicting_ids.push_back(obstacle_informations_[i].tracking_element->getId());
    } else {
      conflicting_ids.push_back(obstacle_informations_[i + 1].tracking_element->getId());
      i++;
    }
  }
  return conflicting_ids;
}

void ConsistencyCheckVisitor::deleteAllObstaclesInRoi(
    std::vector<TrackingElement> &tracking_elements,
    const perception_msgs::LookAtRegions &obstacle_look_at_rois) {
  class ObstacleGetter : public RoadElementVisitor {
   public:
    ObstacleGetter() = default;
    void visit(Obstacle &road_object, TrackingElement &tracking_element) override {
      obstacles.emplace_back(&road_object, &tracking_element);
    }
    std::vector<std::pair<Obstacle *, TrackingElement *>> obstacles;
  } visitor;

  visitor_helper::visitElements(visitor, tracking_elements);
  Obstacle *current_obstacle;
  TrackingElement *current_tracking_element;
  std::set<unsigned int> ids_to_delete;
  for (const auto &pair : visitor.obstacles) {
    std::tie(current_obstacle, current_tracking_element) = pair;
    if (isInAnyObstacleRoi(current_obstacle->getEstimatedPosition(), 0.05, obstacle_look_at_rois)) {
      ids_to_delete.insert(current_tracking_element->getId());
      ROS_WARN_THROTTLE(1.0,
                        "ConsistencyCheckVisitor::deleteAllObstaclesInRoi: "
                        "Delete obstacle in roi");
    }
  }
  boost::remove_erase_if(tracking_elements,
                         [&ids_to_delete](const TrackingElement &tracking_element) {
                           return ids_to_delete.count(tracking_element.getId()) != 0;
                         });
}

bool ConsistencyCheckVisitor::isInAnyObstacleRoi(const Eigen::Vector2d &position,
                                                 const double roi_extension,
                                                 const perception_msgs::LookAtRegions &obstacle_look_at_rois) {
  const Eigen::Vector3d position_v = common::eigen_utils::to3D(position);
  const auto isInROI = [=](const auto &roi) {
    Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    tf2::fromMsg(roi.pose, roi_pose);
    // this line is strange, but it silences a clang-tidy warning...
    const Eigen::Affine3d to_pose = roi_pose.inverse(Eigen::Projective);
    const Eigen::Vector3d position_in_ROI = to_pose * position_v;
    return std::abs(position_in_ROI.x()) - std::abs(roi.rect.x / 2.0) < roi_extension &&
           std::abs(position_in_ROI.y()) - std::abs(roi.rect.y / 2.0) < roi_extension;
  };


  return std::any_of(
      obstacle_look_at_rois.regions.begin(), obstacle_look_at_rois.regions.end(), isInROI);
}

}  // namespace environmental_model
