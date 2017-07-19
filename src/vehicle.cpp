//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <cmath>

#include "vehicle.h"
#include "utilities.h"


double kPI = std::atan(1)*4;

/*
 * Vehicle base class
 */

VehicleBase::VehicleBase() {}

VehicleBase::~VehicleBase() {}


/*
 * Car class
 */

Car::Car() {}

Car::~Car() {}

void Car::update_state(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  speed_ = localization[2];
  yaw_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];
}

void Car::printout() {
  std::cout << "px = " << px_
            << ", py = " << py_
            << ", speed = " << speed_
            << ", yaw = " << yaw_
            << std::endl;
}

/*
 * Ego class
 */

Ego::Ego() {
  time_step_ = 0.02;

  max_acceleration_ = 10;
  max_speed_ = 22.2;

  predicted_points_ = 50;
  state_ = KL;

  is_initialized_ = false;
}

Ego::~Ego() {}


void Ego::update_state(const std::vector<double>& localization,
                       const vehicle_traj& continued_path) {
  px_ = localization[0];
  py_ = localization[1];
  speed_ = localization[2];
  yaw_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  path_s_ = continued_path.first;
  path_d_ = continued_path.second;

  is_initialized_ = true;
}

vehicle_traj Ego::plan_path() {
  if ( is_initialized_ ) {
    switch (state_) {
      case KL: keep_lane();
      case LCL: change_lane("L");
      case LCR: change_lane("R");
      case PLCL: prep_lane_change("L");
      case PLCR: prep_lane_change("R");
    }

    return std::make_pair(path_s_, path_d_);
  }

  return {};
}

void Ego::keep_lane() {
  double last_x;
  double last_y;

  size_t n = path_s_.size();
  while ( n < predicted_points_ ) {
    last_x = path_s_[n-1];
    last_y = path_d_[n-1];

    // We do not correct the offset from the lane center
    double forward_distance = max_speed_*time_step_;
    path_s_.push_back(last_x + forward_distance);
    path_d_.push_back(last_y);

    ++n;
  }
}

void Ego::change_lane(std::string direction) {
  ;
}

void Ego::prep_lane_change(std::string direction) {
  ;
}

void Ego::printout() {
  std::cout << "px = " << px_
            << "py = " << py_
            << "speed = " << speed_
            << "yaw = " << yaw_
            << std::endl;
}


