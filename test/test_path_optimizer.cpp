#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "path_optimizer.hpp"

using ::testing::ElementsAre;


namespace {

class QuickTest : public testing::Test {
protected:
  void SetUp() override { start_time_ = time(nullptr); }

  void TearDown() override {
    const time_t end_time = time(nullptr);

    EXPECT_TRUE(end_time - start_time_ <= 0.1) << "The test took too long.";
  }

  // The UTC time (in seconds) when the test starts
  time_t start_time_;
};

class TestPathOptimizer : public QuickTest {

protected:
  double speed_limit_ = 20;
  double time_step_ = 0.02;
  PathOptimizer opt_{speed_limit_, 10.0, 10.0, time_step_};

  TestPathOptimizer() = default;
};

TEST_F(TestPathOptimizer, startUp) {
  double ps0 = 100;
  double pd0 = 7;

  auto path_sd = opt_.startUp({{ps0, 0, 0}, {pd0, 0, 0}}); // start at lane 2, but not centered
  std::vector<double> path_s = path_sd.first;
  std::vector<double> path_d = path_sd.second;

  // test the transverse position has not been changed
  ASSERT_DOUBLE_EQ(path_d.back(), pd0);
}

TEST_F(TestPathOptimizer, keepLane1) {
  double ps = 100;
  double vs = 10; // started at a low speed
  double pd = 7; // started at lane 2, but not centered
  double vd = 0;
  double pd_f = 6; // lane center

  dynamics dyn_front_car {{1e6, 0, 0}, {0, 0, 0}}; // no front car

  auto path_sd = opt_.keepLane({{ps, vs, 0}, {pd, vd, 0}}, dyn_front_car, pd_f);
  ASSERT_TRUE(!path_sd.first.empty()); // path can be found

  // test the car will finally move to the lane center
  ASSERT_NEAR(path_sd.second.back(), pd_f, 1e-3);

  std::size_t n = path_sd.first.size();
  double vs_f = (path_sd.first[n - 1] - path_sd.first[n - 2]) / time_step_;
  double as_f = (path_sd.first[n - 1] + path_sd.first[n - 3] - 2 * path_sd.first[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(speed_limit_, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value
}

TEST_F(TestPathOptimizer, keepLane2) {
  double ps = 100;
  double vs = speed_limit_;
  double pd = 6; // started at lane 2 and centered
  double vd = 0;
  double pd_f = 6; // lane center

  dynamics dyn_front_car {{1e6, 0, 0}, {0, 0, 0}}; // no front car

  auto path_sd = opt_.keepLane({{ps, vs, 0}, {pd, vd, 0}}, dyn_front_car, pd_f);
  ASSERT_TRUE(!path_sd.first.empty()); // path can be found

  // test the car will stay at the lane center
  ASSERT_NEAR(path_sd.second.back(), pd_f, 1e-3);

  std::size_t n = path_sd.first.size();
  double vs_f = (path_sd.first[n - 1] - path_sd.first[n - 2]) / time_step_;
  double as_f = (path_sd.first[n - 1] + path_sd.first[n - 3] - 2 * path_sd.first[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(speed_limit_, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value
}

TEST_F(TestPathOptimizer, changeLane) {
  double ps = 100;
  double vs = speed_limit_;
  double pd = 6; // start at the center of lane 2
  double vd = 0;
  double pd_f = 2; // the center of lane 1

  std::vector<double> path_s;
  std::vector<double> path_d;

  dynamics dyn_front_car {{1e6, 0, 0}, {0, 0, 0}};
  auto path_sd = opt_.changeLane({{ps, vs, 0}, {pd, vd, 0}}, dyn_front_car, pd_f);
  path_s = path_sd.first;
  path_d = path_sd.second;
  ASSERT_TRUE(!path_s.empty());

  // test lane has been changed
  ASSERT_NEAR(path_d.back(), pd_f, 1e-3);

  std::size_t n = path_s.size();
  double vs_f = (path_s[n - 1] - path_s[n - 2]) / time_step_;
  double as_f = (path_s[n - 1] + path_s[n - 3] - 2 * path_s[n - 2]) / time_step_ / time_step_;
  ASSERT_NEAR(speed_limit_, vs_f, 1e-3);
  ASSERT_NEAR(0, as_f, 0.2); // as_f is an estimation so that it differs from the JMT value
}

}
