//
// Created by jun on 12/12/18.
//
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "map.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"

using ::testing::ElementsAre;


namespace {

class TestTrajectory : public testing::Test {

protected:
  // cwd is build/tests
  std::string filename = "../../data/highway_map.csv";
  Map map_;

  TestTrajectory() : map_(filename) {}
};


TEST_F(TestTrajectory, TestClosestWaypoint) {
  int i = 0;
  ASSERT_EQ(i, closestWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, map_.x, map_.y));
  i = 28;
  ASSERT_EQ(28, closestWaypoint(map_.x[i] + 1.0, map_.y[i] + 1.0, map_.x, map_.y));
  i = 180;
  ASSERT_EQ(180, closestWaypoint(map_.x[i] - 1.0, map_.y[i] - 1.0, map_.x, map_.y));
}


TEST_F(TestTrajectory, TestNextWaypoint) {
  int i = 0;
  ASSERT_EQ(i, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, 0.0, map_.x, map_.y));
  ASSERT_EQ(i+1, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, -pi(), map_.x, map_.y));

  i = 28;
  ASSERT_EQ(i, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, 0.0, map_.x, map_.y));
  ASSERT_EQ(i+1, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, -pi(), map_.x, map_.y));

  i = 180;
  ASSERT_EQ(i, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, 0.0, map_.x, map_.y));
  ASSERT_EQ(0, nextWaypoint(map_.x[i] - 1.0, map_.y[i] + 1.0, -pi(), map_.x, map_.y));
}


TEST_F(TestTrajectory, TestCartesianToFrenet) {
  int i = 0;
  position p = cartesianToFrenet(map_.x[i], map_.y[i], 0, map_.max_s, map_.x, map_.y);
  ASSERT_NEAR(map_.s[i], p.first, 1e-4);
  ASSERT_NEAR(0.0, p.second, 1e-4);

  i = 180;
  p = cartesianToFrenet(map_.x[i], map_.y[i], 0, map_.max_s, map_.x, map_.y);
  ASSERT_NEAR(map_.s[i], p.first, 1e-4);
  ASSERT_NEAR(0.0, p.second, 1e-4);
}


TEST_F(TestTrajectory, TestFrenetToCartisan) {
  int i = 0;
  position p = frenetToCartesian(map_.s[i], 0.0, map_.s, map_.max_s, map_.x, map_.y);
  ASSERT_NEAR(map_.x[i], p.first, 1e-6);
  ASSERT_NEAR(map_.y[i], p.second, 1e-6);

  i = 10;
  p = frenetToCartesian(map_.s[i], 0.0, map_.s, map_.max_s, map_.x, map_.y);
  ASSERT_NEAR(map_.x[i], p.first, 1e-6);
  ASSERT_NEAR(map_.y[i], p.second, 1e-6);

  i = 180;
  p = frenetToCartesian(map_.s[i], 0.0, map_.s, map_.max_s, map_.x, map_.y);
  ASSERT_NEAR(map_.x[i], p.first, 1e-6);
  ASSERT_NEAR(map_.y[i], p.second, 1e-6);
}

using state = std::vector<double>;

TEST(TestJerkMinimizingTrajectory, TestCoefficients) {
  state state0_0 = {0, 10, 0};
  state state1_0 = {10, 10, 0};
  polynomial_coeff p_0 = jerkMinimizingTrajectory(state0_0, state1_0, 1);
  polynomial_coeff gt_0 = {0, 10, 0, 0, 0, 0};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_0[i], p_0[i], 1e-12);

  state state0_1 = {0, 10, 0};
  state state1_1 = {20, 15, 20};
  polynomial_coeff p_1 = jerkMinimizingTrajectory(state0_1, state1_1, 2);
  polynomial_coeff gt_1 = {0, 10, 0, 0, -0.625, 0.3125};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_1[i], p_1[i], 1e-12);

  state state0_2 = {5, 10, 2};
  state state1_2 = {-30, -20, -4};
  polynomial_coeff p_2 = jerkMinimizingTrajectory(state0_2, state1_2, 5);
  polynomial_coeff gt_2 = {5, 10, 1, -3.0, 0.64, -0.0432};
  for (int i = 0; i < 6; ++i) ASSERT_NEAR(gt_2[i], p_2[i], 1e-12);
}


TEST(TestJerkMinimizingTrajectory, TestEvaluation) {
  state state0 = {5, 10, 2};
  state state1 = {-30, -20, -4};
  polynomial_coeff p = jerkMinimizingTrajectory(state0, state1, 5);

  ASSERT_NEAR(5, evalTrajectory(p, 0), 1e-12);
  ASSERT_NEAR(-30, evalTrajectory(p, 5), 1e-12);

  ASSERT_NEAR(10, evalVelocity(p, 0), 1e-12);
  ASSERT_NEAR(-20, evalVelocity(p, 5), 1e-12);

  ASSERT_NEAR(2, evalAcceleration(p, 0), 1e-12);
  ASSERT_NEAR(-4, evalAcceleration(p, 5), 1e-12);
}

}
