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
}

void Vehicle::printout() const {
  std::cout << px_ << ", " << py_ << ", "
            << vx_ << ", " << vy_ << ", "
            << ps_ << ", " << pd_ << ", " << std::endl;
}

int Vehicle::getLaneID() { return lane_id_; }

void Vehicle::setLaneID(int value) { lane_id_ = value; }

double Vehicle::getPd() { return pd_; }

/*
 * Ego class
 */

Ego::Ego() {

  max_acceleration_ = 10;
  max_speed_ = 21.;

  behavior_ = KL;

  lane_change_timer_ = 0;

}

Ego::~Ego() {}

void Ego::update(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  truncatePath();
}

void Ego::truncatePath() {
  if ( !path_s_.empty() ) {

    // first remove processed way points

    auto unprocessed_s_begin = std::lower_bound(path_s_.begin(),
                                                path_s_.end(), ps_);
    int n_removed = std::distance(path_s_.begin(), unprocessed_s_begin);
    auto unprocessed_d_begin = std::next(path_d_.begin(), n_removed);

    path_s_.erase(path_s_.begin(), unprocessed_s_begin);
    path_d_.erase(path_d_.begin(), unprocessed_d_begin);

  }
}

VehicleBehavior Ego::getBehavior() { return behavior_; }

void Ego::setBehavior(VehicleBehavior value) { behavior_ = value; }

vehicle_traj Ego::getPath() { return std::make_pair(path_s_, path_d_); }

int Ego::getTargetLaneID() { return target_lane_id_; }

void Ego::setTargetLaneID(int value) { target_lane_id_ = value; }

double Ego::getLaneChangeTimer() { return lane_change_timer_; }

void Ego::setLaneChangeTimer(double value) { lane_change_timer_ = value; }

double Ego::getMaxSpeed() { return max_speed_; }


