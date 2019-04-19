#include <vector>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "car.hpp"

using ::testing::ElementsAre;


namespace {

class TestCar : public testing::Test {

protected:
  Car car_;

  TestCar() : car_(0, 0, 0, 0) {
    car_.loadMap("data/highway_map.csv");
  }
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

TEST_F(TestCar, updateSurroudingVehicles) {
  double inf_dist = 1e6;

  // move the car to s = 12 m, d = 2 m
  car_.updateParameters({0, 0, 0, 0, 12, 2});
  // [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  std::vector<std::vector<double>> sensor_fusion = {
    {1, 0, 0, 2, 0, 1, 2},
    {2, 0, 0, 0, 3, 25, 6},
    {3, 0, 0, 0, 0, 100, -2},
  };
  car_.updateClosestVehicles(sensor_fusion);

  auto front_vehicles = car_.getClosestFrontVehicles();
  auto rear_vehicles = car_.getClosestRearVehicles();
  ASSERT_THAT(front_vehicles[1].first, ElementsAre(inf_dist, 0, 0));
  ASSERT_THAT(front_vehicles[1].second, ElementsAre(0, 0, 0));
  ASSERT_THAT(rear_vehicles[1].first, ElementsAre(-11, 2, 0));
  ASSERT_THAT(rear_vehicles[1].second, ElementsAre(2, 0, 0));
  ASSERT_THAT(front_vehicles[2].first, ElementsAre(13, 3, 0));
  ASSERT_THAT(front_vehicles[2].second, ElementsAre(6, 0, 0));
  ASSERT_THAT(rear_vehicles[2].first, ElementsAre(-inf_dist, 0, 0));
  ASSERT_THAT(rear_vehicles[2].second, ElementsAre(0, 0, 0));
  ASSERT_THAT(front_vehicles[3].first, ElementsAre(inf_dist, 0, 0));
  ASSERT_THAT(front_vehicles[3].second, ElementsAre(0, 0, 0));
  ASSERT_THAT(rear_vehicles[3].first, ElementsAre(-inf_dist, 0, 0));
  ASSERT_THAT(rear_vehicles[3].second, ElementsAre(0, 0, 0));

  // move to a new position
  car_.updateParameters({0, 0, 0, 0, 121, 6});
  sensor_fusion.emplace_back(std::initializer_list<double>{5, 0, 0, 0, 0, 140, 6});
  sensor_fusion.emplace_back(std::initializer_list<double>{4, 0, 0, 5, 0, 130, 7});
  sensor_fusion.emplace_back(std::initializer_list<double>{6, 0, 0, 0, 0, 122, 3.5});
  car_.updateClosestVehicles(sensor_fusion);

  front_vehicles = car_.getClosestFrontVehicles();
  rear_vehicles = car_.getClosestRearVehicles();
  ASSERT_THAT(front_vehicles[1].first, ElementsAre(1, 0, 0));
  ASSERT_THAT(front_vehicles[1].second, ElementsAre(3.5, 0, 0));
  ASSERT_THAT(rear_vehicles[1].first, ElementsAre(-120, 2, 0));
  ASSERT_THAT(rear_vehicles[1].second, ElementsAre(2, 0, 0));
  ASSERT_THAT(front_vehicles[2].first, ElementsAre(9, 5, 0));
  ASSERT_THAT(front_vehicles[2].second, ElementsAre(7, 0, 0));
  ASSERT_THAT(rear_vehicles[2].first, ElementsAre(-96, 3, 0));
  ASSERT_THAT(rear_vehicles[2].second, ElementsAre(6, 0, 0));
  ASSERT_THAT(front_vehicles[3].first, ElementsAre(inf_dist, 0, 0));
  ASSERT_THAT(front_vehicles[3].second, ElementsAre(0, 0, 0));
  ASSERT_THAT(rear_vehicles[3].first, ElementsAre(-inf_dist, 0, 0));
  ASSERT_THAT(rear_vehicles[3].second, ElementsAre(0, 0, 0));
}

}