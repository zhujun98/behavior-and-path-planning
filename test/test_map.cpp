#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "map.hpp"


namespace {

class TestMap : public testing::Test {

protected:
  // cwd is build/test
  std::string filename = "../../data/highway_map.csv";
  Map map_;

  TestMap() : map_(filename) {}
};


TEST_F(TestMap, Construction) {
  ASSERT_EQ(181, map_.x.size());
  ASSERT_EQ(181, map_.s.size());
  ASSERT_EQ(784.6001, map_.x.front());
  ASSERT_EQ(753.2067, map_.x.back());
  ASSERT_EQ(-0.02359831, map_.dx.front());
  ASSERT_EQ(-0.107399, map_.dx.back());
}

TEST_F(TestMap, GetLaneId) {
  ASSERT_EQ(0, map_.getLaneId(-1));
  ASSERT_EQ(1, map_.getLaneId(3.));
  ASSERT_EQ(3, map_.getLaneId(8.));
  ASSERT_EQ(4, map_.getLaneId(14.));
}

TEST_F(TestMap, LaneCenter) {
  ASSERT_EQ(map_.lane_width / 2., map_.getLaneCenter(0));
  ASSERT_EQ(map_.lane_width / 2., map_.getLaneCenter(1));
  ASSERT_EQ(map_.lane_width * 5 / 2., map_.getLaneCenter(3));
  ASSERT_EQ(map_.lane_width * 5 / 2., map_.getLaneCenter(10));
}

}