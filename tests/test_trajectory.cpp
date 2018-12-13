//
// Created by jun on 12/12/18.
//
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "map.hpp"
#include "utilities.hpp"
#include "trajectory.hpp"


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

  i = 10;
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

}
