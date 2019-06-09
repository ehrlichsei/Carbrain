#include <common/macros.h>
#include "../src/environmental_model/environmental_model.h"
#include "../src/environmental_model/environmental_model_node.h"
#include "../src/environmental_model/road_element_visitors/create_messages_visitor.h"
#include "common/test/dummy_parameter_handler.h"
THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <type_traits>
#include <boost/range/numeric.hpp>
THIRD_PARTY_HEADERS_END

DISABLE_SUGGEST_OVERRIDE_WARNING

using namespace environmental_model;
const ParameterString<std::vector<double>> PARAM_MEASUREMENT_MATRIX(
    "wonham_measurement_matrix");
const ParameterString<std::vector<double>> PARAM_MIN_PROB_PUBLISHING(
    "min_prob_publishing");
const ParameterString<std::vector<double>> PARAM_MIN_PROB_NEW_TRACKING(
    "min_prob_new_tracking");
const ParameterString<double> PARAM_SCALE_PROBABILITIES("scale_probabilities");
const ParameterString<double> PARAM_DECAY_MAX_DISTANCE("decay_max_distance");
const ParameterString<double> PARAM_DECAY_MIN_PROB("decay_min_prob");
const ParameterString<double> PARAM_NO_PASSING_ZONE_DECREASE_FACTOR(
    "no_passing_zone/decrease_factor");
const ParameterString<int> PARAM_NO_PASSING_ZONE_NUM_CONS(
    "no_passing_zone/number_msg_consecutive");
const ParameterString<double> PARAM_NO_PASSING_ZONE_MERGE_DIST(
    "no_passing_zone/merge_distance");
const ParameterString<double> PARAM_NO_PASSING_ZONE_THRESHOLD(
    "no_passing_zone/threshold");
DummyParameterHandler makeParameterHandler() {
  DummyParameterHandler parameter_handler;
  std::vector<double> measurement_matrix(8 * 8, 0.02);
  for (size_t i = 0; i < 8 * 8; i += 9) {
    measurement_matrix[i] = 0.86;
  }
  std::vector<double> min_prob_publishing(8, 0.0);
  std::vector<double> min_prob_new_tracking(8, 0.0);
  parameter_handler.addParam(PARAM_MEASUREMENT_MATRIX, measurement_matrix);
  parameter_handler.addParam(PARAM_MIN_PROB_PUBLISHING, min_prob_publishing);
  parameter_handler.addParam(PARAM_MIN_PROB_NEW_TRACKING, min_prob_new_tracking);
  parameter_handler.addParam(PARAM_SCALE_PROBABILITIES, 1.0);
  parameter_handler.addParam(PARAM_DECAY_MAX_DISTANCE, 1.3);
  parameter_handler.addParam(PARAM_DECAY_MIN_PROB, 0.3);
  parameter_handler.addParam(PARAM_NO_PASSING_ZONE_DECREASE_FACTOR, 0.8);
  parameter_handler.addParam(PARAM_NO_PASSING_ZONE_NUM_CONS, 0);
  parameter_handler.addParam(PARAM_NO_PASSING_ZONE_MERGE_DIST, 5.0);
  parameter_handler.addParam(PARAM_NO_PASSING_ZONE_THRESHOLD, 1.1);
  parameter_handler.addParam(PARAM_ROAD_CLOSURE_START_DISTANCE, 0.4);
  parameter_handler.addParam(PARAM_ROAD_CLOSURE_END_DISTANCE, 0.2);
  parameter_handler.addParam(PARAM_MIN_ASSUMED_ROAD_CLOSURE_LENGTH, 0.3);
  parameter_handler.addParam(PARAM_SPEED_LIMIT_SAVED_SPEED_LIMIT_MSGS_COUNT, 5);
  parameter_handler.addParam(PARAM_SPEED_LIMIT_ALPHA, 0.8);
  return parameter_handler;
}


template <class MSG>
void addSpecificMessageInformations(MSG& /*msg*/) {}
template <>
void addSpecificMessageInformations<perception_msgs::RoadClosure>(perception_msgs::RoadClosure& msg) {
  msg.hull_polygon = msg.base_hull_polygon;
}
template <>
void addSpecificMessageInformations<perception_msgs::SpeedLimitMarking>(
    perception_msgs::SpeedLimitMarking& msg) {
  msg.speed_limit = 20;
}
template <>
void addSpecificMessageInformations<perception_msgs::Obstacle>(perception_msgs::Obstacle& msg) {
  msg.vertices = msg.base_hull_polygon;
  msg.vertices_detected = {true, true, true, true};
}
class EnvironmentalModelTest : public ::testing::Test {
 public:
  std::vector<geometry_msgs::Point> generateBaseHullPolygonAt(const Eigen::Vector3d& position) {
    std::vector<Eigen::Vector3d> points_around_position = {
        {-0.1, -0.1, 0}, {0.1, -0.1, 0}, {0.1, 0.1, 0}, {-0.1, 0.1, 0}};
    std::vector<geometry_msgs::Point> result;
    std::transform(points_around_position.begin(),
                   points_around_position.end(),
                   std::back_inserter(result),
                   [&position](const auto& point) {
                     const Eigen::Vector3d v = point + position;
                     return tf2::toMsg(v);
                   });
    return result;
  }
  template <class ROAD_ELEMENT>
  std::shared_ptr<ROAD_ELEMENT> generateRoadElement(const Eigen::Vector3d& position) {
    using message_type =
        typename std::result_of<decltype(&ROAD_ELEMENT::getMsg)(ROAD_ELEMENT)>::type;
    message_type msg;
    msg.certainty = 0.8;
    msg.base_hull_polygon = generateBaseHullPolygonAt(position);
    addSpecificMessageInformations(msg);
    auto result = std::make_shared<ROAD_ELEMENT>(ROAD_ELEMENT(msg));
    result->setBaseArea(msg.base_hull_polygon);
    return result;
  }

  void assertNArrows(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.arrow_markings.sub_messages.size());
  }
  void assertNCrosswalks(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.crosswalks.sub_messages.size());
  }
  void assertNJunctions(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.junctions.sub_messages.size());
  }
  void assertNObstacles(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.obstacles.sub_messages.size());
  }
  void assertNRoadClosures(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.road_closures.sub_messages.size());
  }
  void assertNSpeedLimits(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.speed_limits.sub_messages.size());
  }
  void assertNStartLines(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    ASSERT_EQ(N, create_message_visitor.start_lines.sub_messages.size());
  }
  void assertNElements(const size_t N, const CreateMessagesVisitor& create_message_visitor) {
    size_t sum = 0;
    sum += create_message_visitor.crosswalks.sub_messages.size();
    sum += create_message_visitor.junctions.sub_messages.size();
    sum += create_message_visitor.road_closures.sub_messages.size();
    sum += create_message_visitor.arrow_markings.sub_messages.size();
    sum += create_message_visitor.obstacles.sub_messages.size();
    sum += create_message_visitor.start_lines.sub_messages.size();
    sum += create_message_visitor.speed_limits.sub_messages.size();
    sum += create_message_visitor.unidentifieds.sub_messages.size();
    ASSERT_EQ(N, sum);
  }
};

TEST_F(EnvironmentalModelTest, generateElements) {
  ros::Time::init();
  auto paramProvider = makeParameterHandler();
  EnvironmentalModel env_model(&paramProvider);
  std::vector<std::shared_ptr<RoadElement>> new_road_elements;
  new_road_elements.push_back(generateRoadElement<Arrow>({0, 0, 0}));
  new_road_elements.push_back(generateRoadElement<Crosswalk>({1, 0, 0}));
  new_road_elements.push_back(generateRoadElement<Junction>({2, 0, 0}));
  new_road_elements.push_back(generateRoadElement<Obstacle>({3, 0, 0}));
  new_road_elements.push_back(generateRoadElement<RoadClosure>({4, 0, 0}));
  new_road_elements.push_back(generateRoadElement<SpeedLimit>({5, 0, 0}));
  new_road_elements.push_back(generateRoadElement<StartLine>({6, 0, 0}));

  env_model.update(new_road_elements, {}, Eigen::Affine3d::Identity());
  navigation_msgs::TrackingElements tracking_elements;
  env_model.collectMsgs(tracking_elements);
  ASSERT_EQ(7, tracking_elements.sub_messages.size());
}

TEST_F(EnvironmentalModelTest, onlyOneElementPerPosition) {
  ros::Time::init();
  auto paramProvider = makeParameterHandler();
  EnvironmentalModel env_model(&paramProvider);
  std::vector<std::shared_ptr<RoadElement>> new_road_elements;
  const Eigen::Vector3d position(0, 0, 0);
  new_road_elements.push_back(generateRoadElement<Arrow>(position));
  new_road_elements.push_back(generateRoadElement<Crosswalk>(position));
  new_road_elements.push_back(generateRoadElement<Junction>(position));
  new_road_elements.push_back(generateRoadElement<Obstacle>(position));
  new_road_elements.push_back(generateRoadElement<RoadClosure>(position));
  new_road_elements.push_back(generateRoadElement<SpeedLimit>(position));
  new_road_elements.push_back(generateRoadElement<StartLine>(position));

  env_model.update(new_road_elements, {}, Eigen::Affine3d::Identity());
  navigation_msgs::TrackingElements tracking_elements;
  env_model.collectMsgs(tracking_elements);
  ASSERT_EQ(1, tracking_elements.sub_messages.size());
}

TEST_F(EnvironmentalModelTest, elementTypeChangesOverTime) {
  ros::Time::init();
  auto paramProvider = makeParameterHandler();
  EnvironmentalModel env_model(&paramProvider);
  const Eigen::Vector3d position(0, 0, 0);
  std::vector<std::shared_ptr<RoadElement>> arrow_road_element;
  arrow_road_element.push_back(generateRoadElement<Arrow>(position));
  std::vector<std::shared_ptr<RoadElement>> junction_road_element;
  junction_road_element.push_back(generateRoadElement<Junction>(position));


  env_model.update(arrow_road_element, {}, Eigen::Affine3d::Identity());
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNArrows(1, create_messages_visitor);
    assertNElements(1, create_messages_visitor);
  }


  env_model.update(junction_road_element, {}, Eigen::Affine3d::Identity());
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNElements(1, create_messages_visitor);
  }
  env_model.update(junction_road_element, {}, Eigen::Affine3d::Identity());
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNJunctions(1, create_messages_visitor);
    assertNElements(1, create_messages_visitor);
  }
  env_model.update(arrow_road_element, {}, Eigen::Affine3d::Identity());
  env_model.update(arrow_road_element, {}, Eigen::Affine3d::Identity());
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNArrows(1, create_messages_visitor);
    assertNElements(1, create_messages_visitor);
  }
}

TEST_F(EnvironmentalModelTest, trackingDynamicObstacle) {
  ros::Time::init();
  auto paramProvider = makeParameterHandler();
  EnvironmentalModel env_model(&paramProvider);
  const int id = 0;  // gate id
  const Gate gate1 = {id, {-2, 1, 0}, {-2, 0, 0}};
  const Gate gate2 = {id, {2, 1, 0}, {2, 0, 0}};
  const DrivingCorridor full_corridor(Gate::GateList{gate1, gate2});
  env_model.updateFullCorridor(full_corridor);


  const double diff_time = 1.0 / 60;
  const double velocity = 0.7;  // m/s
  const double diff_x = velocity * diff_time;
  double x = 0.5;
  double time = 0;
  for (; x < 1.5; x += diff_x, time += diff_time) {
    ros::Time::setNow(ros::Time{time});
    env_model.predict(diff_time, Eigen::Affine3d::Identity());
    std::vector<std::shared_ptr<RoadElement>> new_road_elements(
        {generateRoadElement<Obstacle>({x, 0, 0})});
    env_model.update(new_road_elements, {}, Eigen::Affine3d::Identity());
  }
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNObstacles(1, create_messages_visitor);
    assertNElements(1, create_messages_visitor);
    ASSERT_GE(create_messages_visitor.navigation_obstacles.sub_messages[0].is_dynamic, 0.8);
  }
}

TEST_F(EnvironmentalModelTest, trackingStaticObstacle) {
  ros::Time::init();
  auto paramProvider = makeParameterHandler();
  EnvironmentalModel env_model(&paramProvider);
  const int id = 0;  // gate id
  const Gate gate1 = {id, {-2, 1, 0}, {-2, 0, 0}};
  const Gate gate2 = {id, {2, 1, 0}, {2, 0, 0}};
  const DrivingCorridor full_corridor(Gate::GateList{gate1, gate2});
  env_model.updateFullCorridor(full_corridor);
  const double diff_time = 1.0 / 60;
  double time = 0;
  for (size_t i = 0; i < 50; ++i, time += diff_time) {
    ros::Time::setNow(ros::Time{time});
    env_model.predict(diff_time, Eigen::Affine3d::Identity());
    std::vector<std::shared_ptr<RoadElement>> new_road_elements(
        {generateRoadElement<Obstacle>({1.0, 0, 0})});
    env_model.update(new_road_elements, {}, Eigen::Affine3d::Identity());
  }
  {
    CreateMessagesVisitor create_messages_visitor(ros::Time(0));
    env_model.visitElements(create_messages_visitor);
    assertNObstacles(1, create_messages_visitor);
    assertNElements(1, create_messages_visitor);
    ASSERT_LE(create_messages_visitor.navigation_obstacles.sub_messages[0].is_dynamic, 0.2);
  }
}

TEST_F(EnvironmentalModelTest, updateSpeedLimitMessages) {
  ros::Time::init();

  auto paramProvider = makeParameterHandler();
  std::shared_ptr<RoadElementParameter> road_element_parameter_(
      std::make_shared<RoadElementParameter>(&paramProvider));

  SpeedLimit speed_limit(road_element_parameter_);

  perception_msgs::SpeedLimitMarking speed_limit_msg;
  speed_limit_msg.id = 42;
  speed_limit_msg.certainty = 0.8;

  std::vector<int> speed_limits = {20, 10, 20, 10, 20, 30, 30, 30, 50, 80, 30, 90};
  std::vector<int> expected_speed_limits = {20, 10, 20, 20, 20, 20, 30, 30, 30, 80, 30, 90};
  std::vector<bool> speed_limits_relieved = {
      false, false, false, true, false, false, false, false, true, true, true, true};
  std::vector<bool> expected_speed_limits_relieved = {
      false, false, false, false, false, false, false, false, false, true, true, true};

  for (size_t i = 0; i < speed_limits.size(); i++) {
    MsgCollection collection(ros::Time(0));
    speed_limit_msg.speed_limit = speed_limits[i];
    speed_limit_msg.limit_relieved = speed_limits_relieved[i];
    collection.speed_limits.sub_messages.push_back(speed_limit_msg);
    speed_limit.newMsg(collection, Eigen::Affine3d::Identity());
    ASSERT_EQ(speed_limit.getMsg().speed_limit, expected_speed_limits[i]);
    ASSERT_EQ(speed_limit.getMsg().limit_relieved, expected_speed_limits_relieved[i]);
  }
}



// nothing to do here
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
