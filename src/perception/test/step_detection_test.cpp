#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <gtest/gtest.h>
#include <ros/console.h>
#include <opencv2/imgproc.hpp>
THIRD_PARTY_HEADERS_END

#include "../src/utils/step_detection.h"

DISABLE_SUGGEST_OVERRIDE_WARNING

// creation of test image with equal height and width
cv::Mat createTestImage(const cv::Point2i& orientation, const int& number_areas, const int& sizes) {
  cv::Mat image = cv::Mat::zeros(cv::Size(sizes, sizes), CV_8UC1);
  cv::Point start(0, 0);
  int u_end, v_end;
  if (orientation.x > orientation.y) {
    u_end = image.cols - 1;
    v_end = static_cast<int>(u_end * static_cast<double>(orientation.y) /
                             static_cast<double>(orientation.x));
  } else {
    v_end = image.rows - 1;
    u_end = static_cast<int>(v_end * static_cast<double>(orientation.x) /
                             static_cast<double>(orientation.y));
  }
  cv::Point end(u_end, v_end);
  const int line_iterator_connect = 8;
  cv::LineIterator steps(image, start, end, line_iterator_connect, false);

  std::vector<cv::Point> center_points_right, center_points_left;
  std::vector<std::pair<cv::Point, cv::Point>> right_points, left_points;

  center_points_left.reserve(number_areas);
  center_points_right.reserve(number_areas);
  right_points.reserve(number_areas);
  left_points.reserve(number_areas);

  int area_width = steps.count / (2 * number_areas);

  cv::Point normal_vector = cv::Point(orientation.y, -orientation.x);

  for (int i = 1; i <= steps.count; i++, steps++) {
    cv::Point step = steps.pos();
    cv::Point high(step.x + sizes * normal_vector.x, step.y + sizes * normal_vector.y);
    cv::Point low(step.x - sizes * normal_vector.x, step.y - sizes * normal_vector.y);

    if (i % (2 * area_width) == 0) {

      right_points.push_back(std::pair<cv::Point, cv::Point>(high, low));
      center_points_right.push_back(step);

    } else if (i % (2 * area_width) == area_width) {

      left_points.push_back(std::pair<cv::Point, cv::Point>(high, low));
      center_points_left.push_back(step);
    }
  }
  const cv::Size img_size(sizes, sizes);

  std::transform(right_points.begin(),
                 right_points.end(),
                 center_points_right.begin(),
                 right_points.begin(),
                 [&](auto clip, auto center) {
                   cv::clipLine(img_size, center, clip.first);
                   cv::clipLine(img_size, center, clip.second);
                   return clip;
                 });
  std::transform(left_points.begin(),
                 left_points.end(),
                 center_points_left.begin(),
                 left_points.begin(),
                 [&](auto clip, auto center) {
                   cv::clipLine(img_size, center, clip.first);
                   cv::clipLine(img_size, center, clip.second);
                   return clip;
                 });

  std::vector<std::vector<cv::Point>> polygons;
  polygons.reserve(number_areas);
  std::transform(left_points.begin(),
                 left_points.end(),
                 right_points.begin(),
                 std::back_inserter(polygons),
                 [](auto left, auto right) {
                   std::vector<cv::Point> ret;
                   ret.push_back(left.first);
                   ret.push_back(right.first);
                   ret.push_back(right.second);
                   ret.push_back(left.second);
                   return ret;
                 });

  for (const auto& polygon : polygons) {
    cv::fillConvexPoly(image, polygon, cv::Scalar(255));
  }
  return image;
}

TEST(StepDetectionTests, oneStepHorizontal) {
  const int number_steps = 1;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 0), number_steps, size);

  ImagePoint start(0, image.rows / 2);
  ImagePoint end(image.cols, image.rows / 2);

  const int ref_func_size = 11;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, false);

  const unsigned long expected_size = 1UL;
  const int expected_u = 58;

  EXPECT_EQ(expected_size, steps.size());
  EXPECT_EQ(expected_u, steps.front()(0));
  EXPECT_EQ(image.rows / 2, steps.front()(1));
}

TEST(StepDetectionTests, twoStepsHorizontal) {
  const int number_steps = 2;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 0), number_steps, size);

  const int expected_first_step = 28;
  const int expected_second_step = 88;

  ImagePoint start(0, image.rows / 2);
  ImagePoint end(image.cols, image.rows / 2);

  const int ref_func_size = 11;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, false);
  const unsigned long expected_size = 2UL;

  EXPECT_EQ(expected_size, steps.size());
  EXPECT_EQ(expected_first_step, steps.front()(0));
  EXPECT_EQ(image.rows / 2, steps.front()(1));
  EXPECT_EQ(expected_second_step, steps.back()(0));
  EXPECT_EQ(image.rows / 2, steps.back()(1));

  steps = step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, true);

  EXPECT_EQ(3UL, steps.size());

  int i = 0;
  for (const auto& step : steps) {
    if (i % 2 == 0) {
      EXPECT_EQ(expected_first_step + 30 * i, step(0));
    } else {
      EXPECT_EQ(expected_first_step + 30 * i + 1, step(0));
    }
    EXPECT_EQ(image.rows / 2, step(1));
    i++;
  }
}

TEST(StepDetectionTests, tenStepsHorizontal) {
  const int number_steps = 10;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 0), number_steps, size);

  ImagePoint start(0, image.rows / 2);
  ImagePoint end(image.cols, image.rows / 2);

  const int ref_func_size = 3;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, true);

  cv::Mat canny_gradients;
  ImagePoints canny_steps;
  const int thresh_low = 50;
  const int thresh_high = 250;
  const int mask_size = 3;
  cv::Canny(image(cv::Range(image.rows / 2 - 2, image.rows / 2 + 3), cv::Range::all()),
            canny_gradients,
            thresh_low,
            thresh_high,
            mask_size,
            true);
  for (int i = 1; i < canny_gradients.cols - 1; i++) {
    if (canny_gradients.at<uchar>(2, i) >= 255 && canny_gradients.at<uchar>(2, i + 1) <= 0) {
      canny_steps.push_back(ImagePoint(i, image.rows / 2));
    }
  }

  EXPECT_EQ(canny_steps.size(), steps.size());
  for (std::size_t i = 0; i < steps.size(); i++) {
    EXPECT_EQ(canny_steps[i](0), steps[i](0));
  }
}

TEST(StepDetectionTests, oneStepVertical) {
  const int number_steps = 1;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(0, 1), number_steps, size);

  ImagePoint start(image.cols / 2, 0);
  ImagePoint end(image.cols / 2, image.rows);

  const int ref_func_size = 11;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, false);

  EXPECT_EQ(1UL, steps.size());
  EXPECT_EQ(60, steps.front()(0));
  EXPECT_EQ(58, steps.front()(1));
}

TEST(StepDetectionTests, tenStepsVertical) {
  const int number_steps = 10;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(0, 1), number_steps, size);

  ImagePoint start(image.cols / 2, 0);
  ImagePoint end(image.cols / 2, image.rows);

  const int ref_func_size = 3;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, true);

  cv::Mat canny_gradients;
  ImagePoints canny_steps;
  const int thresh_low = 50;
  const int thresh_high = 250;
  const int mask_size = 3;
  cv::Canny(image(cv::Range::all(), cv::Range(image.cols / 2 - 2, image.cols / 2 + 3)),
            canny_gradients,
            thresh_low,
            thresh_high,
            mask_size,
            true);
  for (int i = 1; i < canny_gradients.rows - 1; i++) {
    if (canny_gradients.at<uchar>(i, 2) >= 255 && canny_gradients.at<uchar>(i + 1, 2) <= 0) {
      canny_steps.push_back(ImagePoint(image.cols / 2, i));
    }
  }

  EXPECT_EQ(canny_steps.size(), steps.size());
  for (std::size_t i = 0; i < steps.size(); i++) {
    EXPECT_EQ(canny_steps[i](1), steps[i](1));
  }
}

TEST(StepDetectionTests, OneStepDiagonal) {
  const int number_steps = 1;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 1), number_steps, size);

  ImagePoint start(0, 0);
  ImagePoint end(image.cols, image.rows);

  const int ref_func_size = 11;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, false);

  const unsigned long expected_size = 1;
  const int expected_u = 59;
  const int expected_v = 58;

  EXPECT_EQ(expected_size, steps.size());
  EXPECT_EQ(expected_u, steps.front()(0));
  EXPECT_EQ(expected_v, steps.front()(1));
}

TEST(StepDetectionTests, twoStepsDiagonal) {
  const int number_steps = 2;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 1), number_steps, size);

  ImagePoint start(0, 0);
  ImagePoint end(image.cols, image.rows);

  const int ref_func_size = 11;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, false);

  const unsigned long expected_size = 2UL;
  const int first_exp_u = 29;
  const int first_exp_v = 28;
  const int second_exp_u = 89;
  const int second_exp_v = 88;

  EXPECT_EQ(expected_size, steps.size());
  EXPECT_EQ(first_exp_u, steps.front()(0));
  EXPECT_EQ(first_exp_v, steps.front()(1));
  EXPECT_EQ(second_exp_u, steps.back()(0));
  EXPECT_EQ(second_exp_v, steps.back()(1));
}

TEST(StepDetectionTests, tenStepsDiagonalScannedHorizontal) {
  const int number_steps = 10;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 1), number_steps, size);

  ImagePoint start(0, image.rows / 2);
  ImagePoint end(image.cols, image.rows / 2);

  const int ref_func_size = 3;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, true);

  cv::Mat canny_gradients;
  ImagePoints canny_steps;
  const int thresh_low = 50;
  const int thresh_high = 250;
  const int mask_size = 3;
  cv::Canny(image(cv::Range(image.rows / 2 - 2, image.rows / 2 + 3), cv::Range::all()),
            canny_gradients,
            thresh_low,
            thresh_high,
            mask_size,
            true);
  for (int i = 1; i < canny_gradients.cols - 1; i++) {
    if (canny_gradients.at<uchar>(2, i) >= 255 && canny_gradients.at<uchar>(2, i + 1) <= 0) {
      canny_steps.push_back(ImagePoint(i - 1, image.rows / 2));
    }
  }

  EXPECT_LE(
      std::abs(static_cast<int>(canny_steps.size()) - static_cast<int>(steps.size())), 1);
  for (std::size_t i = 0; i < steps.size(); i++) {
    EXPECT_EQ(canny_steps[i](0), steps[i](0));
  }
}

TEST(StepDetectionTests, tenStepsDiagonalScannedVertical) {
  const int number_steps = 10;
  const int size = 120;
  cv::Mat image = createTestImage(cv::Point(1, 1), number_steps, size);

  ImagePoint start(image.cols / 2, 0);
  ImagePoint end(image.cols / 2, image.rows);

  const int ref_func_size = 3;
  std::vector<float> ref_func = step_detection::createReferenceFunction(ref_func_size);

  ImagePoints steps =
      step_detection::detectStep(image, ScanLine(start, end), ref_func, 1, true);

  cv::Mat canny_gradients;
  ImagePoints canny_steps;
  const int thresh_low = 50;
  const int thresh_high = 250;
  const int mask_size = 3;
  cv::Canny(image(cv::Range::all(), cv::Range(image.cols / 2 - 2, image.cols / 2 + 3)),
            canny_gradients,
            thresh_low,
            thresh_high,
            mask_size,
            true);
  for (int i = 1; i < canny_gradients.rows - 1; i++) {
    if (canny_gradients.at<uchar>(i, 2) >= 255 && canny_gradients.at<uchar>(i + 1, 2) <= 0) {
      canny_steps.push_back(ImagePoint(image.cols / 2, i - 1));
    }
  }

  EXPECT_LE(
      std::abs(static_cast<int>(canny_steps.size()) - static_cast<int>(steps.size())), 1);
  for (std::size_t i = 0; i < steps.size(); i++) {
    EXPECT_EQ(canny_steps[i](1), steps[i](1));
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
