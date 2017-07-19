//
// Created by jun on 7/16/17.
//

#include "traffic.h"
#include "vehicle.h"


Traffic::Traffic() {}

Traffic::~Traffic() {}

void Traffic::update_state(const std::map<int, std::vector<double>>& sensor_fusion) {
  for ( auto i : sensor_fusion ) {
    Car car;

    car.update_state(i.second);
    cars_.erase(i.first);
    cars_.insert(std::make_pair(i.first, car));
  }
}

bool Traffic::checkCollision(Ego& ego) {
  ;
}

void Traffic::printout() {
  for ( auto it = cars_.begin(); it != cars_.end(); ++it ) {
    std::cout << it->first << ": ";
    it->second.printout();
  }
  std::cout << std::endl;
}