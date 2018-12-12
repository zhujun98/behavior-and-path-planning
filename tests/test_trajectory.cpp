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
  trajectory traj = std::make_pair(map_.x, map_.y);
  ASSERT_EQ(0, closestWaypoint(std::make_pair(784.0, 1135.0), traj));
  ASSERT_EQ(28, closestWaypoint(std::make_pair(1590.0, 1162.0), traj));
  ASSERT_EQ(180, closestWaypoint(std::make_pair(740.0, 1140.0), traj));
}


TEST_F(TestTrajectory, TestNextWaypoint) {
  trajectory traj = std::make_pair(map_.x, map_.y);
  ASSERT_EQ(0, nextWaypoint(std::make_pair(784.0, 1135.0), 0., traj));
  ASSERT_EQ(28, nextWaypoint(std::make_pair(1590.0, 1162.0), 0., traj));
  ASSERT_EQ(180, nextWaypoint(std::make_pair(740.0, 1140.0), 0., traj));

  ASSERT_EQ(1, nextWaypoint(std::make_pair(784.0, 1135.0), -pi(), traj));
  ASSERT_EQ(29, nextWaypoint(std::make_pair(1590.0, 1162.0), -pi(), traj));
  ASSERT_EQ(0, nextWaypoint(std::make_pair(740.0, 1140.0), -pi(), traj));
}

}
