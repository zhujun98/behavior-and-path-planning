#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "car.hpp"

using ::testing::ElementsAre;


namespace {

class QuickTest : public testing::Test {
protected:
  void SetUp() override { start_time_ = time(nullptr); }

  void TearDown() override {
    const time_t end_time = time(nullptr);

    EXPECT_TRUE(end_time - start_time_ <= 0.3) << "The test took too long.";
  }

  // The UTC time (in seconds) when the test starts
  time_t start_time_;
};

class TestPathOptimizer : public QuickTest {

protected:
  double time_step_ = 0.02;
  Car car_;

  TestPathOptimizer() : car_(Map("../../data/highway_map.csv", time_step_)) {}
};

TEST_F(TestPathOptimizer, startUp) {
  double ps0 = 100;
  double pd0 = 7;
  car_.updateParameters({0, 0, 0, 0, ps0, pd0}); // start at lane 2, but not centered

  auto path_sd = PathOptimizer::startUp(&car_);
  std::vector<double> path_s = path_sd.first;
  std::vector<double> path_d = path_sd.second;

  // test the transverse position has not been changed
  ASSERT_DOUBLE_EQ(path_d.back(), pd0);
}

TEST_F(TestPathOptimizer, keepLane) {
  double max_speed = car_.getMaxSpeed();
  double ps0 = 100;
  double pd0 = 7;
  car_.updateParameters({0, 0, 0, 0, ps0, pd0}); // start at lane 2, but not centered

  std::vector<double> path_s;
  std::vector<double> path_d;

    // mock a path which the speed of the end state is 10 m/s
  for (auto i = 0; i < 5; ++i) {
    path_s.push_back(ps0 + i * time_step_ * 10.0);
    path_d.push_back(pd0);
  }
  car_.extendPath({path_s, path_d});

  // keepLane estimates the dynamics from the path
  auto path_sd = PathOptimizer::keepLane(&car_);
  path_s = path_sd.first;
  path_d = path_sd.second;
  ASSERT_TRUE(!path_s.empty());

  // test the car will finally move to the lane center
  ASSERT_NEAR(path_d.back(), car_.getCurrentLaneCenter(), 1e-3);

  std::size_t n = path_s.size();
  double vs_f = (path_s[n - 1] - path_s[n - 2]) / time_step_;
  double as_f = (path_s[n - 1] + path_s[n - 3] - 2 * path_s[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(max_speed, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value

  // mock a path which the speed of the end state is max_speed
  for (auto i=0; i<5; ++i) {
    path_s.push_back(ps0 + i * time_step_ * max_speed);
    path_d.push_back(pd0);
  }
  car_.extendPath({path_s, path_d});

  path_sd = PathOptimizer::keepLane(&car_);
  path_s = path_sd.first;
  path_d = path_sd.second;
  ASSERT_TRUE(!path_s.empty());

  // test the car will stay at the lane center
  ASSERT_NEAR(path_d.back(), car_.getCurrentLaneCenter(), 1e-3);

  n = path_s.size();
  vs_f = (path_s[n - 1] - path_s[n - 2]) / time_step_;
  as_f = (path_s[n - 1] + path_s[n - 3] - 2 * path_s[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(max_speed, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value
}

TEST_F(TestPathOptimizer, changeLane) {
  double max_speed = car_.getMaxSpeed();
  double ps0 = 100;
  double pd0 = 6;
  car_.updateParameters({0, 0, 0, 0, 100, 6}); // start at the center of lane 2

  std::vector<double> path_s;
  std::vector<double> path_d;

  // mock a path
  for (auto i = 0; i < 5; ++i) {
    path_s.push_back(ps0 + i * time_step_ * max_speed);
    path_d.push_back(pd0);
  }
  car_.extendPath({path_s, path_d});

  car_.setTargetLaneId(1);
  // changeLane estimates the dynamics from the path
  auto path_sd = PathOptimizer::changeLane(&car_);
  path_s = path_sd.first;
  path_d = path_sd.second;
  ASSERT_TRUE(!path_s.empty());

  // test lane has been changed
  ASSERT_NEAR(path_d.back(), car_.getTargetLaneCenter(), 1e-3);

  std::size_t n = path_s.size();
  double vs_f = (path_s[n - 1] - path_s[n - 2]) / time_step_;
  double as_f = (path_s[n - 1] + path_s[n - 3] - 2 * path_s[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(max_speed, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value
}

}
