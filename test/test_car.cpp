#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "car.hpp"


using ::testing::ElementsAre;


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

TEST_F(TestCar, updateClosestVehicles) {
  // [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  std::vector<std::vector<double>> sensor_fusion = {
    {1, 0, 0, 2, 0, 1, 2},
    {2, 0, 0, 3, 0, 10, 2},
    {3, 0, 0, 4, 0, 20, 2},
    {4, 0, 0, 0, 2, 5, 6},
    {5, 0, 0, 0, 3, 25, 6},
    {6, 0, 0, 0, 4, 30, 10}
  };

  // move the car to s = 12 m, d = 2 m (lane 1)
  car_.updateParameters({0, 0, 0, 0, 12, 2});
  car_.updateClosestVehicles(sensor_fusion);
  auto closest_front_vehicles = car_.getClosestFrontVehicles();
  auto closest_rear_vehicles = car_.getClosestRearVehicles();
  ASSERT_THAT(closest_front_vehicles[1].first, ElementsAre(20, 4, 0));
  ASSERT_THAT(closest_rear_vehicles[1].first, ElementsAre(10, 3, 0));
  ASSERT_THAT(closest_front_vehicles[2].first, ElementsAre(25, 3, 0));
  ASSERT_THAT(closest_rear_vehicles[2].first, ElementsAre(5, 2, 0));
  ASSERT_THAT(closest_front_vehicles[3].first, ElementsAre(30, 4, 0));
  ASSERT_THAT(closest_rear_vehicles[3].first, ElementsAre());

  // move to a new position
  car_.updateParameters({0, 0, 0, 0, 121, 6});
  car_.updateClosestVehicles(sensor_fusion);
  closest_front_vehicles = car_.getClosestFrontVehicles();
  closest_rear_vehicles = car_.getClosestRearVehicles();
  ASSERT_THAT(closest_front_vehicles[1].first, ElementsAre());
  ASSERT_THAT(closest_rear_vehicles[1].first, ElementsAre());
  ASSERT_THAT(closest_front_vehicles[2].first, ElementsAre());
  ASSERT_THAT(closest_rear_vehicles[2].first, ElementsAre(25, 3, 0));
  ASSERT_THAT(closest_front_vehicles[3].first, ElementsAre());
  ASSERT_THAT(closest_rear_vehicles[3].first, ElementsAre(30, 4, 0));
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