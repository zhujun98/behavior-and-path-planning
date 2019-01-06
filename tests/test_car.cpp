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
  std::string filename = "../../data/highway_map.csv";

  Car car_;

  TestCar() : car_(Map(filename)) {}
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

}