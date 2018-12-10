//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <cmath>

#include "vehicle.h"
#include "map.h"
#include "parameters.h"


Vehicle::Vehicle(const Map& map) :
    is_initialized_(false),
    time_step_(kTIME_STEP),
    history_length_(5),
    map_(&map) {}

Vehicle::~Vehicle() = default;

void Vehicle::updateParameters(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  speed_ = localization[2];
  yaw_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  if ( hps_.size() >= history_length_ ) {
    hps_.pop_front();
    hpd_.pop_front();
  }
  hps_.push_back(ps_);
  hpd_.push_back(pd_);

  lane_id_ = map_->computerLaneID(pd_);

  /*
   * The following data are not consistent with the returned data from
   * the simulator. The time interval of the returned data is changing!
   */

  // update velocity in Frenet coordinate system

  if ( hps_.size() > 1 ) {
    vs_ = (hps_.back() - hps_.front()) / ((hps_.size() - 1) * time_step_);
    vd_ = (hpd_.back() - hpd_.front()) / ((hpd_.size() - 1) * time_step_);

    if ( hvs_.size() >= history_length_ ) {
      hvs_.pop_front();
      hvd_.pop_front();
    }
    hvs_.push_back(vs_);
    hvd_.push_back(vd_);
  }

  // update acceleration in Frenet coordinate system

  if ( hvs_.size() > 1 ) {
    as_ = (hvs_.back() - hvs_.front()) / ((hvs_.size() - 1) * time_step_);
    ad_ = (hvd_.back() - hvd_.front()) / ((hvd_.size() - 1) * time_step_);
  }

//  std::cout << "Compare speed: " << speed_ << " "
//            << std::sqrt(vs_*vs_ + vd_*vd_) << std::endl;
}


void Vehicle::printout() const {
  std::cout << "Lane ID = " << map_->computerLaneID(pd_) << ", "
            << "px = " << px_ << ", " << "py = " << py_ << ", "
            << "speed = " << speed_ << ", " << "yaw = " << yaw_ << ", "
            << "ps = " << ps_ << ", " << "pd = " << pd_ << ", " << std::endl;
}

double Vehicle::getTimeStep() const { return time_step_; }

int Vehicle::getLaneID() const { return lane_id_; }

double Vehicle::getPx() const { return px_; }

double Vehicle::getPy() const { return py_; }

double Vehicle::getSpeed() const { return speed_; }

double Vehicle::getYaw() const { return yaw_; }

double Vehicle::getPs() const { return ps_; }

double Vehicle::getPd() const { return pd_; }

double Vehicle::getVs() const { return vs_; }

double Vehicle::getVd() const { return vd_; }

double Vehicle::getAs() const { return as_; }

double Vehicle::getAd() const { return ad_; }
