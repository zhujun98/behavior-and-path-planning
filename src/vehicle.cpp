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
  std::cout << "Lane ID = " << lane_id_ << ", "
            << "px = " << px_ << ", " << "py = " << py_ << ", "
            << "vx = " << vx_ << ", " << "vy = " << vy_ << ", "
            << "ps = " << ps_ << ", " << "pd = " << pd_ << ", " << std::endl;
}

int Vehicle::getLaneID() const { return lane_id_; }

void Vehicle::setLaneID(int value) { lane_id_ = value; }

double Vehicle::getVx() const { return vx_; }
double Vehicle::getVy() const { return vy_; }
double Vehicle::getPs() const { return ps_; }
double Vehicle::getPd() const { return pd_; }
