#include <cmath>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "map.hpp"
#include "utilities.hpp"

namespace {

class TestMap : public testing::Test {

protected:
  Map map_;

  // cwd is build/test
  TestMap() : map_("data/highway_map.csv") {
  }
};


TEST_F(TestMap, Construction) {
  ASSERT_EQ(181, map_.size());
  ASSERT_EQ(3u, map_.nLanes());
}

TEST_F(TestMap, GetLaneId) {
  ASSERT_EQ(0, map_.getLaneId(-1));
  ASSERT_EQ(1, map_.getLaneId(3.));
  ASSERT_EQ(3, map_.getLaneId(8.));
  ASSERT_EQ(4, map_.getLaneId(14.));
}

TEST_F(TestMap, LaneCenter) {
  ASSERT_EQ(map_.laneWidth() / 2., map_.getLaneCenter(0));
  ASSERT_EQ(map_.laneWidth() / 2., map_.getLaneCenter(1));
  ASSERT_EQ(map_.laneWidth() * 5 / 2., map_.getLaneCenter(3));
  ASSERT_EQ(map_.laneWidth() * 5 / 2., map_.getLaneCenter(10));
}

TEST_F(TestMap, ClosestWaypoint) {
  ASSERT_EQ(0, map_.closestWaypoint(783.6001, 1136.571));
  ASSERT_EQ(28, map_.closestWaypoint(1599.067, 1162.989));
  ASSERT_EQ(180, map_.closestWaypoint(752.2067, 1135.417));
}

TEST_F(TestMap, NextWaypoint) {
  int i = 0;
  ASSERT_EQ(0, map_.nextWaypoint(783.6001, 1136.571, 0.0));
  ASSERT_EQ(1, map_.nextWaypoint(783.6001, 1136.571, -pi()));

  ASSERT_EQ(29, map_.nextWaypoint(1599.067, 1162.989, 0.0));
  ASSERT_EQ(28, map_.nextWaypoint(1599.067, 1162.989, -pi()));

  ASSERT_EQ(180, map_.nextWaypoint(752.2067, 1135.417, 0.0));
  ASSERT_EQ(0, map_.nextWaypoint(752.2067, 1135.417, -pi()));
}


//TEST_F(TestMap, CartesianToFrenet) {
//  position p = map_.cartesianToFrenet(784.6001, 1135.571, 0);
//  ASSERT_NEAR(0, p.first, 1e-4);
//  ASSERT_NEAR(0.0, p.second, 1e-4);
//  p = map_.cartesianToFrenet(753.2067, 1136.417, 0);
//  ASSERT_NEAR(6914.14925765991, p.first, 1e-4);
//  ASSERT_NEAR(0.0, p.second, 1e-4);
//}

TEST_F(TestMap, FrenetToCartisan) {
  position p = map_.frenetToCartesian(0, 0.0);
  ASSERT_NEAR(784.6001, p.first, 1e-6);
  ASSERT_NEAR(1135.571, p.second, 1e-6);

  p = map_.frenetToCartesian(302.548864364624, 0.0);
  ASSERT_NEAR(1079.219, p.first, 1e-6);
  ASSERT_NEAR(1180.179, p.second, 1e-6);

  p = map_.frenetToCartesian(6914.14925765991, 0.0);
  ASSERT_NEAR(753.2067, p.first, 1e-6);
  ASSERT_NEAR(1136.417, p.second, 1e-6);
}
}