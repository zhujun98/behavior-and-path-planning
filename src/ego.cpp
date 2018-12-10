//
// Created by jun on 7/25/17.
//
#include <queue>

#include "ego.hpp"
#include "ego_states.hpp"
#include "utilities.hpp"


Ego::Ego(const Map& map) :
  is_initialized_(false),
  time_step_(0.02),
  max_speed_(mph2mps(50)),
  max_acceleration_ (10),
  max_jerk_(10),
  min_safe_distance_coeff_(0.5),
  max_evaluation_distance_(100),
  state_(EgoStateFactory::createState(FT)),
  map_(map),
  surroundings_(map_.n_lanes + 2)
{
  state_->onEnter(*this);
}

Ego::~Ego() { delete state_; }

void Ego::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateSurroundings(sensor_fusion);
  updateUnprocessedPath();

  // Do nothing until collecting enough data. For calculation of the
  // derivative.
  if ( hvs_.size() < history_length_ ) { return; }

  EgoState* state = state_->checkTransition(*this);
  if (state != nullptr) {
    state_->onExit(*this);
    delete state_;
    state_ = state;
    state_->onEnter(*this);
  }
  state_->onUpdate(*this);
}


void Ego::updateParameters(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];

  if (hps_.size() >= history_length_) {
    hps_.pop_front();
    hpd_.pop_front();
  }
  hps_.push_back(ps_);
  hpd_.push_back(pd_);

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
}


void Ego::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion) {
  for (auto& v : surroundings_) v.clear();

  for (auto& v : sensor_fusion) {
    surroundings_[map_.getLaneId(v[5])].push_back(v);
  }
}


void Ego::updateUnprocessedPath() {
  if ( !path_.first.empty() ) {

    // important to start another lap
    if ( path_.first.back() > map_.max_s && ps_ < path_.first.front() ) {
      ps_ += map_.max_s;
    }

    auto unprocessed_s_begin = std::lower_bound(path_.first.begin(), path_.first.end(), ps_);
    // remove processed way points
    long n_removed = std::distance(path_.first.begin(), unprocessed_s_begin);
    auto unprocessed_d_begin = std::next(path_.second.begin(), n_removed);

    path_.first.erase(path_.first.begin(), unprocessed_s_begin);
    path_.second.erase(path_.second.begin(), unprocessed_d_begin);
  }
}

std::vector<std::vector<double>> Ego::getVehiclesInLane(uint8_t lane_id) const {
  return surroundings_[lane_id];
}

std::pair<std::vector<double>, std::vector<double>> Ego::getClosestVehicles(uint8_t lane_id) const {
  using car_in_lane = std::pair<double, car_state>;
  std::priority_queue<car_in_lane, std::vector<car_in_lane>, std::greater<car_in_lane>> front_cars;
  std::priority_queue<car_in_lane> rear_cars;

  for (auto &v : surroundings_[lane_id]) {
    double ds = v[4] - ps_;
    if (ds > 0) front_cars.emplace(ds, v);
    else rear_cars.emplace(ds, v);
  }

  return {front_cars.top().second, rear_cars.top().second};
}

void Ego::truncatePath(unsigned int n_keep) {
  if (path_.first.size() > n_keep) {
    path_.first.erase(path_.first.begin() + n_keep, path_.first.end());
    path_.second.erase(path_.second.begin() + n_keep, path_.second.end());
  }
}

void Ego::extendPath(trajectory path) {
  if (!path_.first.empty()) {
    if (path_.first.back() != path.first[0] || path_.second.back() != path.second[0]) {
      throw std::runtime_error("The first point of new path is not the last point of the old path.");
    }
  }

  path_.first.insert(path_.first.end(), path.first.begin(), path.first.end());
  path_.second.insert(path_.second.end(), path.second.begin(), path.second.end());

  // re-calculate s value when the car just finishes a lap!
  double max_s = map_.max_s;
  if (path_.first[0] >= max_s)
    for (auto &s : path_.first) s -= max_s;
}


Ego::trajectory Ego::getPath() { return path_; }


Ego::car_state Ego::getCurrentState() const {
  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  // we need at least 3 points to calculate acceleration
//  if (path_.first.size() < 3)
//    return std::make_pair(std::vector<double>{ps_, vs_, as_}, std::vector<double>{pd_, vd_, ad_});
//
//  auto it_s = path_.first.rbegin();
//  auto it_d = path_.second.rbegin();
//  ps0 = *it_s;
//  pd0 = *it_d;
//  std::advance(it_s, 1);
//  std::advance(it_d, 1);
//  double ps0_1 = *it_s;
//  double pd0_1 = *it_d;
//  std::advance(it_s, 1);
//  std::advance(it_d, 1);
//  double ps0_2 = *it_s;
//  double pd0_2 = *it_d;
//  vs0 = (ps0 - ps0_1) / time_step_;
//  vd0 = (pd0 - pd0_1) / time_step_;
//  double vs0_1 = (ps0_1 - ps0_2) / time_step_;
//  double vd0_1 = (pd0_1 - pd0_2) / time_step_;
//  as0 = (vs0 - vs0_1) / time_step_;
//  ad0 = (vd0 - vd0_1) / time_step_;

  return {ps0, vs0, as0, pd0, vd0, ad0};
}

double Ego::getMaxSpeed() const { return max_speed_; }
double Ego::getMaxAcceleration() const { return max_acceleration_; }
double Ego::getMaxJerk() const { return max_jerk_; }
double Ego::getMaxSteering() const { return max_steering_; }

double Ego::getMinSafeDistance(double speed) const { return min_safe_distance_coeff_*speed + 5; }
double Ego::getMaxEvaluationDistance() const { return max_evaluation_distance_; }

const Ego::surroundings& Ego::getSurroundings() const { return surroundings_; }

void Ego::setTargetLaneId(uint8_t value) {
  if ( value >= 1 && value <= 3 ) { target_lane_id_ = value; }
}


void Ego::info() const {
  std::cout << "Lane ID = " << getCurrentLaneId() << ", "
            << "px = " << px_ << ", " << "py = " << py_ << ", "
            << "vx = " << vx_ << ", " << "vy = " << vy_ << ", "
            << "ps = " << ps_ << ", " << "pd = " << pd_ << ", " << std::endl;

  std::cout << "No. of vehicles on the lane (from left to right): ";
  for (auto &v : surroundings_) std::cout << v.size() << ",\n";

  for (auto i = 0; i < surroundings_.size(); ++i) {
    auto closest_cars = getClosestVehicles(i);

    auto front_car = closest_cars.first;
    if ( !front_car.empty() ) {
      std::cout << "Lane ID " << i << "Closest front car: ds = "
                << front_car[0] - ps_ << ", v = " << front_car[1] << ", ";
    }

    auto rear_car = closest_cars.second;
    if ( !rear_car.empty() ) {
      std::cout << "Closest rear car: ds = "
                << rear_car[0] - ps_ << ", v = " << rear_car[1] << ", ";
    }
    std::cout << std::endl;
  }
}

uint8_t Ego::getCurrentLaneId() const { return map_.getLaneId(pd_); }
uint8_t Ego::getTargetLaneId() const { return target_lane_id_; }

bool Ego::isAroundOrigin() const { return (ps_ < 30 || map_.max_s - ps_ < 30); }
