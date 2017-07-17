//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <cmath>

#include "car.h"
#include "utilities.h"


double kPI = std::atan(1)*4;


Car::Car() {
  is_initialized_ = false;

  speed_ = 0.0;

  acceleration_ = 0.0;
  time_step_ = 0.02;

  max_acceleration_ = 10;
  max_speed_ = 22.2;

  predicted_points_ = 50;
  state_ = KL;
}

Car::~Car() {}

car_traj Car::advance(car_traj continued_path,
                  std::map<int, std::vector<double>> sensor_fusion) {
  traffic_.update_state(sensor_fusion);
  update_state(continued_path);
  realize_state();

  return std::make_pair(path_x_, path_y_);
}

void Car::update_state(car_traj continued_path) {
  path_x_.clear();
  path_y_.clear();
  path_x_ = continued_path.first;
  path_y_ = continued_path.second;
}

void Car::realize_state() {

  switch (state_) {
    case KL: realize_keep_lane();
    case LCL: realize_lane_change("L");
    case LCR: realize_lane_change("R");
    case PLCL: realize_prep_lane_change("L");
    case PLCR: realize_prep_lane_change("R");
  }
}

void Car::realize_keep_lane() {
  double last_x;
  double last_y;
  double yaw;

  size_t n = path_x_.size();
  while ( n < predicted_points_ ) {
    last_x = path_x_[n-1];
    last_y = path_y_[n-1];

    double forward_distance = max_speed_*time_step_;
    if ( path_x_.size() < 2 ) {
      yaw = 0.0;
    } else {
      yaw = std::atan2(path_y_[n-2] - path_y_[n-1], path_x_[n-2] - path_x_[n-1]) - kPI;
    }
    path_x_.push_back(last_x + forward_distance*std::cos(yaw));
    path_y_.push_back(last_y + forward_distance*std::sin(yaw));
    n = path_x_.size();
  }
}

void Car::realize_prep_lane_change(std::string direction) {
  ;
}

void Car::realize_lane_change(std::string direction) {
  ;
}
