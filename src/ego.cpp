//
// Created by jun on 7/25/17.
//

#include "ego.h"
#include "map.h"
#include "vehicle.h"
#include "ego_state.h"
#include "utilities.h"
#include "ego_state_keep_lane.h"


Ego::Ego() {
  // initialize state
  state_ = new EgoStateKeepLane();
  state_->onEnter(*this);

  time_step_ = 0.02; // 20 ms
  prediction_pts_ = 100; // No. of prediction points

  max_acceleration_ = 10;
  max_speed_ = 21.;
}

Ego::~Ego() {
  delete state_;
}

void Ego::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion,
                 const Map& map) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];
  lane_id_ = map.compute_lane_id(pd_);

  updateUnprocessedPath();

  EgoState* state = state_->onUpdate(*this, sensor_fusion, map);
  if ( state != NULL ) {
    state_->onExit(*this);
    delete state_;
    state_ = state;
    state_->onEnter(*this);
  }
}

void Ego::updateUnprocessedPath() {
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

void Ego::truncatePath(unsigned int n_keep) {
  while ( path_s_.size() > n_keep ) {
    path_s_.pop_back();
    path_d_.pop_back();
  }
}

void Ego::extendPath(std::vector<double> coeff_s, std::vector<double> coeff_d) {

  double t = 0.0;
  while ( path_s_.size() < prediction_pts_ ) {
    t += time_step_;
    path_s_.push_back(evalTrajectory(coeff_s, t));
    path_d_.push_back(evalTrajectory(coeff_d, t));
  }
}

std::pair<std::vector<double>, std::vector<double>>
Ego::getPath() const { return std::make_pair(path_s_, path_d_); }

double Ego::getMaxSpeed() const { return max_speed_; }

double Ego::getMaxAcceleration() const { return max_acceleration_; }

double Ego::getMaxSteering() const { return max_steering_; }

double Ego::getTimeStep() const { return time_step_; }

unsigned int Ego::getPredictionPts() const { return prediction_pts_; }
