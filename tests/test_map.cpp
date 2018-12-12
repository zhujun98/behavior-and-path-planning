//
// Created by jun on 12/12/18.
//
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "map.hpp"


namespace {

class TestMap : public testing::Test {

protected:
  // TODO: make it relative path
  std::string filename = "/home/jun/Projects/behavior-and-path-planning/tests/highway_map.csv";
  Map map_;

  TestMap() : map_(filename) {};

};


TEST_F(TestMap, testConstruction) {
  ASSERT_EQ(181, map_.x.size());
  ASSERT_EQ(181, map_.s.size());
  ASSERT_EQ(784.6001, map_.x.front());
  ASSERT_EQ(753.2067, map_.x.back());
  ASSERT_EQ(-0.02359831, map_.dx.front());
  ASSERT_EQ(-0.107399, map_.dx.back());
}

TEST_F(TestMap, testGetLaneId) {
  ASSERT_EQ(0, map_.getLaneId(-1));
  ASSERT_EQ(1, map_.getLaneId(4.));
  ASSERT_EQ(3, map_.getLaneId(12.));
  ASSERT_EQ(4, map_.getLaneId(14.));
}

TEST_F(TestMap, testGetLaneCenter) {
  ASSERT_EQ(map_.lane_width / 2., map_.getLaneCenter(0));
  ASSERT_EQ(map_.lane_width / 2., map_.getLaneCenter(1));
  ASSERT_EQ(map_.lane_width * 5 / 2., map_.getLaneCenter(3));
  ASSERT_EQ(map_.lane_width * 5 / 2., map_.getLaneCenter(10));
}

}