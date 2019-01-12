//
// Created by jun on 7/25/17.
//
#include <queue>

#include "car.hpp"
#include "car_states.hpp"
#include "trajectory.hpp"
#include "jmt.hpp"


Car::Car(const Map& map) :
  is_initialized_(false),
  map_(map),
  time_step_(0.02),
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
  double px = localization[0];
  double py = localization[1];
  double vx = localization[2];
  double vy = localization[3];
  double ps = localization[4];
  double pd = localization[5];

  if (is_initialized_) {
    ax_ = (vx - vx_) / time_step_;
    ay_ = (vy - vy_) / time_step_;

    double vs = (ps - ps_) / time_step_;
    double vd = (pd - pd_) / time_step_;
    as_ = (vs - vs_) / time_step_;
    ad_ = (vd - vd_) / time_step_;
    vs_ = vs;
    vd_ = vd;
  } else {
    ax_ = 0;
    ay_ = 0;

    vs_ = std::sqrt(vx*vx + vy*vy); // estimation
    vd_ = 0;
    as_ = 0;
    ad_ = 0;

    is_initialized_ = true;
  }

  px_ = px;
  py_ = py;
  vx_ = vx;
  vy_ = vy;

  ps_ = ps;
  pd_ = pd;
}


void Car::updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion) {
  for (auto& v : surroundings_) v.clear();
  for (auto& v : sensor_fusion) surroundings_[map_.getLaneId(v[6])].push_back(v);
}


void Car::updateUnprocessedPath() {
  if (!path_s_.empty()) {
    // the path includes the starting point
    if (path_s_.back() > map_.max_s && path_s_.front() > ps_) ps_ += map_.max_s;

    auto unprocessed_s0 = std::lower_bound(path_s_.begin(), path_s_.end(), ps_);
    auto unprocessed_d0 = std::next(path_d_.begin(), std::distance(path_s_.begin(), unprocessed_s0));

    path_s_.erase(path_s_.begin(), unprocessed_s0);
    path_d_.erase(path_d_.begin(), unprocessed_d0);
  }
}

void Car::truncatePath(unsigned int n_keep) {
  if (path_s_.size() > n_keep) {
    path_s_.erase(path_s_.begin() + n_keep, path_s_.end());
    path_d_.erase(path_d_.begin() + n_keep, path_d_.end());
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

Car::trajectory Car::estimateFinalState() const {
  double ps, vs, as;
  double pd, vd, ad;

  if (path_s_.empty() || path_s_.size() < 2) {
    ps = ps_;
    pd = pd_;
    vs = vs_;
    vd = vd_;
    as = as_;
    ad = ad_;
  } else if (path_s_.size() == 2) {
    ps = ps_;
    pd = pd_;
    vs = (path_s_[1] - path_s_[0]) / time_step_;
    vd = (path_d_[1] - path_d_[0]) / time_step_;
    as = (vs - vs_) / time_step_;
    ad = (vd - vd_) / time_step_;
  } else {
    std::size_t n = path_s_.size();

    ps = path_s_[n-1];
    pd = path_d_[n-1];
    vs = (ps - path_s_[n-2]) / time_step_;
    vd = (pd - path_d_[n-2]) / time_step_;
    as = (ps + path_s_[n-3] - 2*path_s_[n-2]) / time_step_;
    ad = (pd + path_d_[n-3] - 2*path_d_[n-2]) / time_step_;
  }

  return {{ps, vs, as}, {pd, vd, ad}};
}

void Car::followTraffic() {
  truncatePath(5);

  auto state0 = estimateFinalState();
  std::vector<double> state_s0 = state0.first;
  std::vector<double> state_d0 = state0.second;

  double delta_t = 1.0; // prediction time

  double ps_f = ps_ + 2 * delta_t * max_speed_;
  double vs_f = max_speed_;
  double as_f = 0;

  double pd_f = 0;
  double vd_f = 0;
  double ad_f = 0;

  std::vector<double> state_s1 = {ps_f, vs_f, as_f};
  std::vector<double> state_d1 = {pd_f, vd_f, ad_f};

  polynomial_coeff coeff_s = jerkMinimizingTrajectory(state_s0, state_s1, delta_t);
  polynomial_coeff coeff_d = jerkMinimizingTrajectory(state_d0, state_d1, delta_t);

  double t = 0;
  while (t < delta_t) {
    t += time_step_;
    double ps = evalTrajectory(coeff_s, t);
    double pd = evalTrajectory(coeff_d, t);
    path_s_.push_back(ps);
    path_d_.push_back(pd);
  }
}

void Car::shiftLaneLeft() {

}

void Car::shiftLaneRight() {

}

Car::trajectory Car::getPathXY() const {
  std::vector<double> path_x;
  std::vector<double> path_y;
  for (auto i = 0; i < path_s_.size(); ++i) {
    position pxy = frenetToCartesian(path_s_[i], path_d_[i], map_.s, map_.max_s, map_.x, map_.y);
    path_x.push_back(pxy.first);
    path_y.push_back(pxy.second);
  }

  return {path_x, path_y};
}

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
