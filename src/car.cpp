//
// Created by jun on 7/25/17.
//
#include <queue>

#include "car.hpp"
#include "car_states.hpp"


Car::Car(const Map& map) :
  is_initialized_(false),
  time_step_(0.02),
  map_(map),
  state_(CarStateFactory::createState(States::ON)),
  surroundings_(map_.n_lanes + 2)
{
  state_->onEnter(*this);
}

Car::~Car() { delete state_; }

void Car::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateSurroundings(sensor_fusion);
  updateUnprocessedPath();

  CarState* state = state_->checkTransition(*this);
  if (state != nullptr) {
    state_->onExit(*this);
    delete state_;
    state_ = state;
    state_->onEnter(*this);
  }
  state_->onUpdate(*this);
}


void Car::updateParameters(const std::vector<double>& localization) {
  px_ = localization[0];
  py_ = localization[1];
  vx_ = localization[2];
  vy_ = localization[3];
  ps_ = localization[4];
  pd_ = localization[5];
}


void Car::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion) {
  for (auto& v : surroundings_) v.clear();
  for (auto& v : sensor_fusion) surroundings_[map_.getLaneId(v[6])].push_back(v);
}


void Car::updateUnprocessedPath() {
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

std::pair<std::vector<double>, std::vector<double>> Car::getClosestVehicles(uint16_t lane_id) const {
  using car_in_lane = std::pair<double, std::vector<double>>;
  std::priority_queue<car_in_lane, std::vector<car_in_lane>, std::greater<car_in_lane>> front_cars;
  std::priority_queue<car_in_lane> rear_cars;

  for (auto &v : surroundings_[lane_id]) {
    double ds = v[4] - ps_;
    if (ds > 0) front_cars.emplace(ds, v);
    else rear_cars.emplace(ds, v);
  }

  return {front_cars.top().second, rear_cars.top().second};
}

void Car::truncatePath(unsigned int n_keep) {
  if (path_.first.size() > n_keep) {
    path_.first.erase(path_.first.begin() + n_keep, path_.first.end());
    path_.second.erase(path_.second.begin() + n_keep, path_.second.end());
  }
}

void Car::extendPath(trajectory path) {
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

void Car::followTraffic() {
//  truncatePath(5);
//
//  auto state0 = getInitialState();
//  double ps0 = state0.first[0];
//  double vs0 = state0.first[1];
//
//  // Calculate the maximum reachable final speed. The maximum jerk is
//  // not considered here. So this could be an over-estimation.
//  double vs1 = vs0 + prediction_time*car.getMaxAcceleration();
//  if ( vs1 > car.getTargetSpeed() ) { vs1 = car.getTargetSpeed(); }
//
//  // If there is vehicle in front of the car car, then find the maximum
//  // acceleration that can make a safe distance between the car car
//  // and the front car.
//  if ( !front_vehicle.empty() ) {
//    double ps_front = front_vehicle[0];
//    double vs_front = front_vehicle[1];
//
//    double acc = car.getMaxAcceleration();
//    int n_steps = 20;
//    double acc_step = 2*car.getMaxAcceleration()/n_steps;
//    // The maximum deceleration is actually 3*max_acceleration_.
//    // In practice, the number should be the maximum deceleration that
//    // the car can physically achieve.
//    for ( int i=0; i<=2*n_steps; ++i ) {
//      vs1 = vs0 + prediction_time*acc;
//      if ( vs1 > car.getTargetSpeed() ) { vs1 = car.getTargetSpeed(); }
//      if ( vs1 < 0 ) { vs1 = 0; }
//
//      double distance = ps_front - ps0 - car.getMinSafeDistance(vs1) +
//                        (vs_front - 0.5*(vs0 + vs1))*prediction_time;
//      if ( distance > 0 ) { break; }
//
//      acc -= acc_step;
//    }
//  }
//
//  double ds1 = 0.5*(vs0 + vs1)*prediction_time;
//  double pd1 = (car.getLaneID() - 0.5) * car.getMap()->getLaneWidth();
//
//  PathPlanner planner(car.getTargetSpeed(), car.getMaxAcceleration(), car.getMaxJerk());
//
//  planner.setDsBoundary(ds1*0.8, ds1*1.2);
//  planner.setVsBoundary(vs1*0.8, vs1*1.2);
//
//  planner.setPdBoundary(pd1, pd1);
//
//  vehicle_trajectory new_path = planner.plan(state0, prediction_time);
//
//  car.extendPath(new_path);
}

void Car::shiftLaneLeft() {

}

void Car::shiftLaneRight() {

}

Car::trajectory Car::getPath() { return path_; }

double Car::getMaxSpeed() const { return max_speed_; }
double Car::getMaxAcceleration() const { return max_acceleration_; }
double Car::getMaxJerk() const { return max_jerk_; }
double Car::getMaxSteering() const { return max_steering_; }

void Car::setTargetLaneId(uint8_t value) {
  if (value >= 1 && value <= map_.n_lanes)
    target_lane_id_ = value;
  else std::cerr << "Invalid target lane ID: " << value << std::endl;
}

void Car::info() const {
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

uint16_t Car::getCurrentLaneId() const { return map_.getLaneId(pd_); }
uint16_t Car::getTargetLaneId() const { return target_lane_id_; }

bool Car::isAroundOrigin() const { return (ps_ < 30 || map_.max_s - ps_ < 30); }
