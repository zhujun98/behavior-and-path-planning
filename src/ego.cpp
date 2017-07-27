//
// Created by jun on 7/25/17.
//

#include "ego.h"
#include "map.h"
#include "ego_state.h"
#include "utilities.h"
#include "ego_state_constant_speed.h"


Ego::Ego() {
  time_step_ = 0.02; // in s
  prediction_pts_ = 100; // No. of prediction points

  max_acceleration_ = 10;  // in m/s^2
  max_speed_ = 22;  // in m/s

  safe_ds_in_seconds_ = 1;  // in s

  // initialize state
  state_ = new EgoStateConstantSpeed();
  state_->onEnter(*this);
}

Ego::~Ego() {
  delete state_;
}

void Ego::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion,
                 const Map& map) {
  updateParameters(localization, map);
  updateSurroundings(sensor_fusion, map);
  updateUnprocessedPath();

  EgoState* state = state_->onUpdate(*this, map);
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

void Ego::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion,
                             const Map& map) {
  surroundings_.center.clear();
  surroundings_.left.clear();
  surroundings_.right.clear();

  for ( const auto& v : sensor_fusion ) {
    double lane_id = map.compute_lane_id(v[5]);
    if ( lane_id == lane_id_ ) {
      surroundings_.center.push_back(v);
    } else if ( lane_id == lane_id_ - 1 ) {
      surroundings_.left.push_back(v);
    } else if ( lane_id == lane_id_ + 1 ) {
      surroundings_.right.push_back(v);
    }
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

double Ego::getSafeDsInSeconds() const { return safe_ds_in_seconds_; }

Surroundings const* Ego::getSurroundings() const { return &surroundings_; }