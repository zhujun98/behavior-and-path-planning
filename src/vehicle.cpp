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

Vehicle::Vehicle() {
  is_initialized_ = false;
}

Vehicle::~Vehicle() {}

int Vehicle::getLaneID() { return lane_id_; }

void Vehicle::setLaneID(int value) { lane_id_ = value; }

void Vehicle::update(const std::vector<double>& localization, const Map& map) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  lane_id_ = map.compute_lane_id(pd_);

  if ( !is_initialized_) {
    target_lane_id_ = lane_id_;
    is_initialized_ = true;
  }
}

void Vehicle::printout() const {
  std::cout << px_ << ", " << py_ << ", "
            << vx_ << ", " << vy_ << ", "
            << ps_ << ", " << pd_ << ", "
            << "Lane ID: " << lane_id_ << std::endl;
}

/*
 * Ego class
 */

Ego::Ego() {
  is_active_ = false;

  max_acceleration_ = 10;
  max_speed_ = 21.;

  behavior_ = KL;
}

Ego::~Ego() {}


int Ego::getTargetLaneID() { return target_lane_id_; }

void Ego::setTargetLaneID(int value) { target_lane_id_ = value; }

VehicleBehavior Ego::getBehavior() { return behavior_; }

void Ego::setBehavior(VehicleBehavior value) { behavior_ = value; }

vehicle_traj Ego::getPath() { return std::make_pair(path_s_, path_d_); }



