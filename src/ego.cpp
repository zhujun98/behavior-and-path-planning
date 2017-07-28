//
// Created by jun on 7/25/17.
//
#include "ego.h"
#include "map.h"
#include "ego_state.h"
#include "utilities.h"


Ego::Ego(const Map& map) : Vehicle(map) {
  time_step_ = 0.02; // in s
  prediction_pts_ = 100; // No. of prediction points

  max_acceleration_ = 10;  // in m/s^2
  max_speed_ = 22;  // in m/s

  min_safe_distance_ = 15; // in m

  target_speed_ = max_speed_;

  state_ = EgoStateFactory::createState(FT);
//  state_ = EgoStateFactory::createState(CS);
  state_->onEnter(*this);
}

Ego::~Ego() {
  delete state_;
}

void Ego::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateSurroundings(sensor_fusion);
  updateUnprocessedPath();

  EgoState* state = state_->checkTransition(*this);
  if ( state != NULL ) {
    truncatePath(5);  // remove most of the planned path from the last state

    state_->onExit(*this);
    delete state_;
    state_ = state;
    state_->onEnter(*this);
  } else {
    state_->onUpdate(*this);
  }
}

void Ego::updateUnprocessedPath() {
  if ( !path_s_.empty() ) {

    // remove processed way points
    auto unprocessed_s_begin = std::lower_bound(path_s_.begin(), path_s_.end(), ps_);

    int n_removed = std::distance(path_s_.begin(), unprocessed_s_begin);
    auto unprocessed_d_begin = std::next(path_d_.begin(), n_removed);

    path_s_.erase(path_s_.begin(), unprocessed_s_begin);
    path_d_.erase(path_d_.begin(), unprocessed_d_begin);

  }
}

void Ego::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion) {
  surroundings_.center.clear();
  surroundings_.left.clear();
  surroundings_.right.clear();

  int ego_lane_id = map_->computerLaneID(pd_);
  for ( const auto& v : sensor_fusion ) {
    int lane_id = map_->computerLaneID(v[5]);
    if ( lane_id == ego_lane_id ) {
      surroundings_.center.push_back(v);
    } else if ( lane_id == ego_lane_id - 1 ) {
      surroundings_.left.push_back(v);
    } else if ( lane_id == ego_lane_id + 1 ) {
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

  // Reset path when the car is about to finish a lap!
  // At this moment, the car should sit in between the first way point
  // and the last way point.
  double max_s = map_->getMaxS();
  if ( path_s_[0] > max_s ) {
    for ( auto &v : path_s_ ) { v -= max_s; }
  }
}

std::vector<double> const* Ego::getPathS() const { return &path_s_; }

std::vector<double> const* Ego::getPathD() const { return &path_d_; }

double Ego::getMaxSpeed() const { return max_speed_; }

double Ego::getMaxAcceleration() const { return max_acceleration_; }

double Ego::getMaxSteering() const { return max_steering_; }

double Ego::getTimeStep() const { return time_step_; }

unsigned int Ego::getPredictionPts() const { return prediction_pts_; }

double Ego::getMinSafeDistance() const { return min_safe_distance_; }

Surroundings const* Ego::getSurroundings() const { return &surroundings_; }

Map const* Ego::getMap() const { return map_; }

int Ego::getTargetLaneID() const { return target_lane_id_; }

void Ego::setTargetLaneID(int value) {
  if ( value >= 1 && value <= 3 ) { target_lane_id_ = value; }
}

double Ego::getTargetSpeed() const { return target_speed_; }

void Ego::setTargetSpeed(double value) {
  if ( value > max_speed_ ) {
    target_speed_ = max_speed_;
  } else if ( value < 0 ) {
    target_speed_ = 0;
  } else {
    target_speed_ = value;
  }
}