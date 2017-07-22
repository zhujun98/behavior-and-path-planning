//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <cmath>

#include "vehicle.h"


double kPI = std::atan(1)*4;

/*
 * Vehicle class
 */

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::update(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  // remove the way points which has been passed (processed) and keep
  // up to 10 way points which has not been reached.
  if ( !path_s_.empty() ) {
    auto it_s = std::lower_bound(path_s_.begin(), path_s_.end(), ps_);
    long j = std::distance(path_s_.begin(), it_s);
    auto it_d = std::next(path_d_.begin(), j);

    if ( std::distance(it_s, path_s_.end()) > 5 ) {
      std::vector<double> unprocessed_path_s(it_s, std::next(it_s, 5));
      std::vector<double> unprocessed_path_d(it_d, std::next(it_d, 5));
      path_s_.clear();
      path_d_.clear();
      for ( auto v : unprocessed_path_s ) { path_s_.push_back(v); }
      for ( auto v : unprocessed_path_d ) { path_d_.push_back(v); }
    } else {
      std::vector<double> unprocessed_path_s(it_s, path_s_.end());
      std::vector<double> unprocessed_path_d(it_d, path_d_.end());
      path_s_.clear();
      path_d_.clear();
      for ( auto v : unprocessed_path_s ) { path_s_.push_back(v); }
      for ( auto v : unprocessed_path_d ) { path_d_.push_back(v); }
    }
  }
}

/*
 * Ego class
 */

Ego::Ego() {

  max_acceleration_ = 10;
  max_speed_ = 22.2;
}

Ego::~Ego() {}

VehicleBehavior Ego::getBehavior() { return behavior_; }
void Ego::setBehavior(VehicleBehavior value) { behavior_ = value; }

vehicle_traj Ego::getPath() { return std::make_pair(path_s_, path_d_); }



