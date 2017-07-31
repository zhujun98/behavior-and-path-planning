//
// Created by jun on 7/25/17.
//
#include "ego.h"
#include "map.h"
#include "ego_states/ego_state.h"
#include "utilities.h"


Ego::Ego(const Map& map) : Vehicle(map) {

  max_speed_ = 22;  // in m/s
  max_acceleration_ = 10; // in m/s^2
  max_jerk_ = 10; // in m/s^3
  min_safe_distance_ = 15; // in m

  max_evaluation_time_ = 3; // in s
  max_evaluation_distance_ = 100; // in m

  target_speed_ = max_speed_*0.95;

  state_ = EgoStateFactory::createState(FT);
  state_->onEnter(*this);

  // ugly?!
  for ( size_t i = 0; i < map_->getNoLanes(); ++i ) {
    std::vector<std::vector<double>> place_holder;
    surroundings_.push_back(place_holder);
  }

  ticker_ = 0;
}

Ego::~Ego() { delete state_; }

void Ego::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateSurroundings(sensor_fusion);
  updateUnprocessedPath();

  // do nothing until collecting enough data
  if ( hvs_.size() < history_length_ ) { return; }

  EgoState* state = state_->checkTransition(*this);
  if ( state != NULL ) {
    state_->onExit(*this);
    delete state_;
    state_ = state;
    state_->onEnter(*this);
  }
  state_->onUpdate(*this);

  ++ticker_;
}

void Ego::updateUnprocessedPath() {
  if ( !path_s_.empty() ) {

    // important to start another lap
    if ( path_s_.back() > map_->getMaxS() && ps_ < path_s_.front() ) {
      ps_ += map_->getMaxS();
    }

    auto unprocessed_s_begin = std::lower_bound(path_s_.begin(), path_s_.end(), ps_);
    // remove processed way points
    int n_removed = std::distance(path_s_.begin(), unprocessed_s_begin);
    auto unprocessed_d_begin = std::next(path_d_.begin(), n_removed);

    path_s_.erase(path_s_.begin(), unprocessed_s_begin);
    path_d_.erase(path_d_.begin(), unprocessed_d_begin);

  }
}

void Ego::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion) {

  for ( auto &v : surroundings_ ) { v.clear(); }

  for ( const auto& v : sensor_fusion ) {
    int lane_id = map_->computerLaneID(v[5]);
    if (lane_id > 0 && lane_id <= map_->getNoLanes()) {
      surroundings_[lane_id - 1].push_back(v);
    }
  }
}

std::vector<double> Ego::getClosestVehicle(int lane_id, int direction) const {

  if ( surroundings_.size() < map_->getNoLanes() ) { return {}; }

  if ( lane_id < 1 || lane_id > surroundings_.size() ) { return {}; }

  // 1 for front, -1 for rear
  if ( direction != 1 && direction != -1 ) { return {};}

  // get the distance and the speed of the front car
  // Ignore the vehicle beyond the max_evaluation_distance_
  double min_ds = speed_*max_evaluation_time_;
  if (min_ds > max_evaluation_distance_ ) { min_ds = max_evaluation_distance_; }
  double ps;
  double vs;
  bool is_found = false;

  // lane ID starts from 1
  for ( auto &v : surroundings_[lane_id - 1] ) {
    double ds = v[4] - ps_;
    if ( ds*direction > 0 && ds*direction < min_ds ) {
      min_ds = ds*direction;
      ps = v[4];
      vs = v[2];
      is_found = true;
    }
  }

  std::vector<double> state;
  if ( is_found ) {
    state.push_back(ps);
    state.push_back(vs);
    state.push_back(0); // currently assume 0 acceleration
  }

  return state;
}

void Ego::truncatePath(unsigned int n_keep) {
  while ( path_s_.size() > n_keep ) {
    path_s_.pop_back();
    path_d_.pop_back();
  }
}

void Ego::extendPath(vehicle_trajectory new_path) {

  // the last point of the old path and the first point of the new
  // path are the same.
  for ( int i=1; i < new_path.first.size(); ++i ) {
    path_s_.push_back(new_path.first[i]);
    path_d_.push_back(new_path.second[i]);
  }

  // Reset path when the car just finishes a lap!
  double max_s = map_->getMaxS();
  if ( path_s_[0] >= max_s ) {
    for ( auto &v : path_s_ ) { v -= max_s; }
  }
}

void Ego::printTraffic() {
  std::cout << "No. of vehicles on the lane (from left to right): ";
  for ( auto &v : surroundings_ ) {
    std::cout << v.size() << ", ";
  }
  std::cout << std::endl;

  // lane ID starts from 1
  for ( int i = 1; i <= surroundings_.size(); ++i ) {
    auto front_car = getClosestVehicle(i, 1);
    if ( !front_car.empty() ) {
      std::cout << "Lane ID " << i << "Closest front car: ds = "
                << front_car[0] - ps_ << ", v = " << front_car[1] << ", ";
    }

    auto rear_car = getClosestVehicle(i, -1);
    if ( !rear_car.empty() ) {
      std::cout << "Closest rear car: ds = "
                << rear_car[0] - ps_ << ", v = " << rear_car[1] << ", ";
    }
    std::cout << std::endl;
  }
}

std::vector<double> const* Ego::getPathS() const { return &path_s_; }

std::vector<double> const* Ego::getPathD() const { return &path_d_; }

double Ego::getMaxSpeed() const { return max_speed_; }

double Ego::getMaxAcceleration() const { return max_acceleration_; }

double Ego::getMaxJerk() const { return max_jerk_; }

double Ego::getMaxSteering() const { return max_steering_; }

double Ego::getMinSafeDistance() const { return min_safe_distance_; }

double Ego::getMaxEvaluationDistance() const { return max_evaluation_distance_; }

typedef std::vector<std::vector<std::vector<double>>> Surroundings;
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

long Ego::getTicker() const { return ticker_; }