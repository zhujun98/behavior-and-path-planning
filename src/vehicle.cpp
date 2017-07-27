//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <cmath>

#include "vehicle.h"
#include "map.h"


double kPI = std::atan(1)*4;

/*
 * Vehicle class
 */

Vehicle::Vehicle(const Map& map) {
  is_initialized_ = false;

  map_ = &map;
}

Vehicle::~Vehicle() {}

void Vehicle::updateParameters(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  speed_ = localization[2];
  yaw_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  is_initialized_ = true;
}

void Vehicle::printout() const {
  std::cout << "Lane ID = " << map_->computerLaneID(pd_) << ", "
            << "px = " << px_ << ", " << "py = " << py_ << ", "
            << "speed = " << speed_ << ", " << "yaw = " << yaw_ << ", "
            << "ps = " << ps_ << ", " << "pd = " << pd_ << ", " << std::endl;
}

int Vehicle::getLaneID() const { return map_->computerLaneID(pd_); }

double Vehicle::getPx() const { return px_; }

double Vehicle::getPy() const { return py_; }

double Vehicle::getSpeed() const { return speed_; }

double Vehicle::getYaw() const { return yaw_; }

double Vehicle::getPs() const { return ps_; }

double Vehicle::getPd() const { return pd_; }
