//
// Created by jun on 12/12/18.
//
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "car.hpp"


namespace {

class TestCar : public testing::Test {

protected:
  Car car_;

  TestCar() : car_(Map("../../data/highway_map.csv")) {}
};


TEST_F(TestCar, UpdateParameters) {
  car_.updateParameters({0, 0, 0, 0, 0, -1}); // x, y, vx, vy, s, d
  // result is affected only by d
  ASSERT_EQ(0, car_.getCurrentLaneId());

  car_.updateParameters({0, 0, 0, 0, 0, 3});
  ASSERT_EQ(1, car_.getCurrentLaneId());

  car_.updateParameters({0, 0, 0, 0, 0, 8});
  ASSERT_EQ(3, car_.getCurrentLaneId());

  car_.updateParameters({0, 0, 0, 0, 0, 12.1});
  ASSERT_EQ(4, car_.getCurrentLaneId());
}

TEST_F(TestCar, FollowTraffic) {
  car_.updateParameters({0, 0, 0, 0, 0, 0}); // x, y, vx, vy, s, d
  car_.followTraffic();
  auto path_xy = car_.getPathXY();

  car_.updateParameters({1, 1, 10, 5, 1.4, 0}); // x, y, vx, vy, s, d
  car_.followTraffic();

  path_xy = car_.getPathXY();
}

}