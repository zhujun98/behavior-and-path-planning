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

  lane_id_ = 2;
}

Ego::~Ego() {}


void Ego::update_state(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  ps_ = localization[2];
  pd_ = localization[3];
  speed_ = localization[4];
  yaw_ = localization[5];

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

vehicle_traj Ego::plan_path() {
  switch (state_) {
    case KL:
      keep_lane();
    case LCL:
      change_lane("L");
    case LCR:
      change_lane("R");
    case PLCL:
      prep_lane_change("L");
    case PLCR:
      prep_lane_change("R");
  }
  std::cout << ps_ << std::endl;
  print1DContainer(path_s_);
  std::cout << pd_ << std::endl;
  print1DContainer(path_d_);
  return std::make_pair(path_s_, path_d_);
}

void Ego::keep_lane() {
  double last_s;
  double last_d;

  PathPlanner planner;

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( path_s_.empty() ) {
    ps0 = ps_;
    pd0 = pd_;
  } else {
    ps0 = *std::next(path_s_.end(), -1);
    pd0 = *std::next(path_d_.end(), -1);
  }

  vs0 = max_speed_;
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = max_speed_;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = time_step_*predicted_points_;
  ps1 = ps0 + max_speed_*duration;
  pd1 = (lane_id_ - 0.5)*4.0;

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = planner.jerk_minimizing_trajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = planner.jerk_minimizing_trajectory(state0_d, state1_d, duration);
  double t = 0.0;
  while ( path_s_.size() < predicted_points_ ) {
    t += time_step_;
    path_s_.push_back(planner.eval_trajectory(coeff_s, t));
    path_d_.push_back(planner.eval_trajectory(coeff_d, t));
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


