//
// Created by jun on 7/16/17.
//
#include "traffic.h"
#include "vehicle.h"


Traffic::Traffic(Ego& ego, const Map& map) {
  ego_car_ = &ego;
  map_ = &map;
}

Traffic::~Traffic() {}

void Traffic::update(const std::vector<double>& localization,
                     const std::vector<std::vector<double>>& sensor_fusion) {

  ego_car_->update(localization);
  ego_car_->setLaneID(map_->compute_lane_id(ego_car_->getPd()));

  other_cars_.clear();
  for ( const auto& i : sensor_fusion ) {
    Vehicle car;
    car.update(i);
    car.setLaneID(map_->compute_lane_id(car.getPd()));

    other_cars_.push_back(car);
  }
}

bool Traffic::checkCollision(double t) {
  ;
}

void Traffic::printout() {
  std::cout << ego_car_->getLaneID() << ", ";
  for ( auto &it : other_cars_ ) {
    std::cout << it.getLaneID() << ", ";
  }
  std::cout << std::endl;
}
