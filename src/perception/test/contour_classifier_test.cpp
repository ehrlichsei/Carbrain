// Copyright 2017 KITcar
#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <math.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
THIRD_PARTY_HEADERS_END

namespace fs = ::boost::filesystem;

#include "../src/road_object_detection/classifiers/contour_classifier.h"
#include "../src/road_object_detection/classifiers/contour_classifier/arrow_contour_template.h"
#include "../src/road_object_detection/classifiers/contour_classifier/speed_limit_contour_template.h"
#include "../src/road_object_detection/classifiers/contour_classifier/unclassified_contour_template.h"

#include "../src/road_object_detection/classifiers/contour_classifier/contour_template.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

constexpr double MATCH_TOLERANCE = 0.02;

namespace road_object_detection {
namespace contour_classifier {

// Test fixture
class ContourClassifierTest : public ::testing::Test {
 protected:
  ContourClassifierTest() : ::testing::Test() {}
};

class ContourTemplateMock : public ContourTemplate {
 public:
  ContourTemplateMock(const CvContours2D &contours,
                      const CvHierarchy &hierarchy,
                      const common::DynamicPolynomial &middle_lane_polynomial)
      : ContourTemplate(contours, hierarchy, middle_lane_polynomial) {}

  // ContourTemplate interface
  int getTemplateId() const override { return -42; }

  RoadObjectPtr asRoadObject(const ros::Time &, double, const ContourTrees &) const override {

    assert(false &&
           "This is only a MOCK and should not be returned as a road object.");
    return nullptr;
  }


  void writeAdditionalFields(cv::FileStorage *) const override {}
};

TEST_F(ContourClassifierTest, BasicContourDistanceTest) {

  CvContours2D contours = {{{0.0, 0.0}, {0.1, 0.0}, {0.1, 0.3}, {0.0, 0.3}}};

  CvContours2D contours_other = {{{0, 0}, {.1, .1}, {.2, .1}}};  // different than contours

  CvHierarchy hierarchy{{-1, -1, -1, -1}};
  common::DynamicPolynomial polynomial;

  ContourTrees trees =
      ContourTemplate::cvContoursToContourTrees(contours, hierarchy, polynomial);

  for (ContourTree &contour : trees) {
    contour.normalizeContour();
  }
  EXPECT_EQ(trees.size(), 1);

  EXPECT_EQ(contours[0].size(), trees[0].contour.size());
  EXPECT_TRUE(trees[0].children.empty());

  ContourTemplateMock tmpl(contours, hierarchy, polynomial);

  // now advanced - comparing trees
  EXPECT_NEAR(1.0, tmpl.match(trees), MATCH_TOLERANCE);

  ContourTrees trees_other =
      ContourTemplate::cvContoursToContourTrees(contours_other, hierarchy, polynomial);

  // trees are different!
  EXPECT_LT(tmpl.match(trees_other), 0.8);
}

inline CvContour2D rasterEllipse(float rx, float ry, float x_offset = 0.0, float y_offset = 0.0) {
  CvContour2D contour;
  for (float t = 0.0; t < 2 * M_PI; t += 0.01) {
    float u = std::tan(t / 2);
    contour.push_back({rx * (1 - u * u) / (u * u + 1) + x_offset,
                       2 * ry * u / (u * u + 1) + y_offset});
  }
  return contour;
}

// Test to compare two ellipsoids that are shifted and different diameters
TEST_F(ContourClassifierTest, ShiftedEllipsesTest) {
  CvHierarchy hierarchy{{-1, -1, -1, -1}};
  common::DynamicPolynomial polynomial;

  auto ellipse1 = rasterEllipse(2, 4);
  auto ellipse2 = rasterEllipse(2, 2, 5, 7);

  ContourTemplateMock tmpl({ellipse1}, hierarchy, polynomial);
  ContourTrees ellipse_trees =
      ContourTemplate::cvContoursToContourTrees({ellipse2}, hierarchy, polynomial);

  EXPECT_LT(0.1, std::abs(tmpl.match(ellipse_trees) - 1));
}

//! ellipses with nested contour ("0")
TEST_F(ContourClassifierTest, HierarchicalEllipsesTest) {
  CvHierarchy hierarchy{{-1, -1, 1, -1}, {-1, -1, -1, 0}};


  common::CubicPolynomial::CoefficientList coeffs = {{0, 0, 0, 0}};
  common::CubicPolynomial polynomial(coeffs);

  auto ellipse1_outer = rasterEllipse(0.4, 0.45);
  auto ellipse1_inner = rasterEllipse(0.3, 0.35);

  // shifted by vector (0.04,0.03) which has length 0.05
  auto ellipse2_outer = rasterEllipse(0.4, 0.45, 0.04, 0.03);
  auto ellipse2_inner = rasterEllipse(0.3, 0.35, 0.04, 0.03);

  ContourTemplateMock tmpl({ellipse1_outer, ellipse1_inner}, hierarchy, polynomial);
  ContourTrees ellipse2_trees = ContourTemplate::cvContoursToContourTrees(
      {ellipse2_outer, ellipse2_inner}, hierarchy, polynomial);

  for (ContourTree &contour : ellipse2_trees) {
    contour.normalizeContour();
  }

  // distance should be only the initial contour shift (0.04,0.03)
  double score = tmpl.match(ellipse2_trees);
  // for the outer contour, the error is 0.03 (lateral shift) and
  EXPECT_NEAR(1.0, score, MATCH_TOLERANCE);
}

TEST_F(ContourClassifierTest, TemplateSelfTest) {
  // create a shifted "0" template
  CvHierarchy hierarchy{{-1, -1, 1, -1}, {-1, -1, -1, 0}};
  common::DynamicPolynomial polynomial;
  UnclassifiedContourTemplate tmpl(
      {rasterEllipse(0.4, 0.4), rasterEllipse(0.3, 0.3)}, hierarchy, polynomial);

  EXPECT_NEAR(1.0, tmpl.match(tmpl.getContourTrees()), MATCH_TOLERANCE);
}

//! tests template read and write
TEST_F(ContourClassifierTest, TemplateStorageTest) {
  // create a shifted "0" template
  CvHierarchy hierarchy{{-1, -1, 1, -1}, {-1, -1, -1, 0}};
  common::DynamicPolynomial polynomial;

  UnclassifiedContourTemplate tmpl(
      {rasterEllipse(0.4, 0.45), rasterEllipse(0.3, 0.35)}, hierarchy, polynomial);

  const std::string TEMPLATE_STORAGE_FILENAME = "/tmp/tmpl.yaml";
  tmpl.write(TEMPLATE_STORAGE_FILENAME);

  auto file = boost::filesystem::path(TEMPLATE_STORAGE_FILENAME);
  EXPECT_TRUE(fs::exists(file) && fs::is_regular_file(file));

  auto loaded_tmpl = ContourTemplateFactory::createFromFile(TEMPLATE_STORAGE_FILENAME);
  EXPECT_TRUE(loaded_tmpl != nullptr);

  // tmpl.normalizeContours();

  // this is the same object (at least should be)
  EXPECT_NEAR(1.0, loaded_tmpl->match(tmpl.getContourTrees()), MATCH_TOLERANCE);
}

//! checks that all template files are valid
TEST_F(ContourClassifierTest, ValidTemplatesTest) {
  const auto path =
      ros::package::getPath("perception") + "/data/contour_templates/";

  ContourTemplates templates;
  EXPECT_NO_THROW(templates = ContourClassifier::loadTemplates(path));

  for (const auto &contour : templates) {
    auto tmpl = dynamic_cast<UnclassifiedContourTemplate *>(contour.get());
    EXPECT_FALSE(tmpl);
  }
}

ContourTemplateType getTemplateType(const ContourTemplate &tmpl) {
  if (dynamic_cast<const ArrowContourTemplate *>(&tmpl)) {
    return ContourTemplateType::ARROW;
  }
  if (dynamic_cast<const SpeedLimitContourTemplate *>(&tmpl)) {
    return ContourTemplateType::SPEED_LIMIT;
  }

  return ContourTemplateType::UNCATEGORIZED;
}

bool checkisSameTemplateObject(const ContourTemplate &a, const ContourTemplate &b) {
  auto type_a = getTemplateType(a);
  auto type_b = getTemplateType(b);

  if (type_a != type_b) {
    return false;
  }

  switch (type_a) {
    case ContourTemplateType::ARROW: {
      auto arrow_a = dynamic_cast<const ArrowContourTemplate *>(&a);
      auto arrow_b = dynamic_cast<const ArrowContourTemplate *>(&b);

      return *arrow_a == *arrow_b;
    }
    case ContourTemplateType::SPEED_LIMIT: {
      auto speed_limit_a = dynamic_cast<const SpeedLimitContourTemplate *>(&a);
      auto speed_limit_b = dynamic_cast<const SpeedLimitContourTemplate *>(&b);

      return *speed_limit_a == *speed_limit_b;
    }
    case ContourTemplateType::UNCATEGORIZED:
    default:
      return false;
  }
}

//! checks that different templates do not match (if they are different)
TEST_F(ContourClassifierTest, TemplatesMatchTest) {
  const auto path =
      ros::package::getPath("perception") + "/data/contour_templates/";

  ContourTemplates contour_templates;
  EXPECT_NO_THROW(contour_templates = ContourClassifier::loadTemplates(path));

  for (size_t i = 0; i < contour_templates.size(); ++i) {
    for (size_t j = 0; j < contour_templates.size(); ++j) {
      const auto &contour_i = contour_templates[i];
      const auto &contour_j = contour_templates[j];

      double matching_score = contour_i->match(contour_j->getContourTrees());

      if (i == j) {  // self match
        EXPECT_NEAR(1.0, matching_score, MATCH_TOLERANCE);
      } else if (checkisSameTemplateObject(*contour_i, *contour_j)) {  // multiple contours describing
        EXPECT_GT(matching_score, 0.3);
      } else {
        EXPECT_LT(matching_score, 0.6);
      }
    }
  }
}

}  // namespace contour_classifier

}  // namespace road_object_detection

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  road_object_detection::contour_classifier::ContourTree::sampling_resolution = 0.001;

  return RUN_ALL_TESTS();
}
