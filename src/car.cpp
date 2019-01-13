#include "car.hpp"
#include "car_states.hpp"
#include "trajectory.hpp"
#include "jmt.hpp"


/*
 * PathOptimizer
 */

PathOptimizer::PathOptimizer() = default;

PathOptimizer::~PathOptimizer() = default;

PathOptimizer::trajectory PathOptimizer::keepLane(Car* car) {

  auto dynamics0 = car->estimateFinalDynamics();
  std::vector<double> dynamics_s0 = dynamics0.first;
  std::vector<double> dynamics_d0 = dynamics0.second;

  double delta_t = 1.0;

  double ps_f = dynamics_s0[0] + 2 * delta_t * car->max_speed_;
  double vs_f = car->max_speed_;
  double as_f = 0;

  double pd_f = car->getCurrentLaneCenter();
  double vd_f = 0;
  double ad_f = 0;

  std::vector<double> dynamics_s1 = {ps_f, vs_f, as_f};
  std::vector<double> dynamics_d1 = {pd_f, vd_f, ad_f};

  polynomial_coeff coeff_s = jerkMinimizingTrajectory(dynamics_s0, dynamics_s1, delta_t);
  polynomial_coeff coeff_d = jerkMinimizingTrajectory(dynamics_d0, dynamics_d1, delta_t);

  double t = 0;
  double time_step = car->time_step_;
  std::vector<double> path_s;
  std::vector<double> path_d;
  while (t < delta_t) {
    t += time_step;
    double ps = evalTrajectory(coeff_s, t);
    double pd = evalTrajectory(coeff_d, t);
    path_s.push_back(ps);
    path_d.push_back(pd);
  }

  return {path_s, path_d};
}

PathOptimizer::trajectory PathOptimizer::changeLaneLeft(Car* car) {
  std::vector<double> path_s;
  std::vector<double> path_d;

  return {path_s, path_d};
}

PathOptimizer::trajectory PathOptimizer::changeLaneRight(Car* car) {
  std::vector<double> path_s;
  std::vector<double> path_d;

  return {path_s, path_d};
}

/*
 * Car
 */

Car::Car(const Map& map) :
  is_initialized_(false),
  time_step_(0.02),
  map_(map),
  state_(CarStateFactory::createState(States::ON))
{
  state_->onEnter(*this);
}

Car::~Car() { delete state_; }

void Car::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateClosestVehicles(sensor_fusion);
  updateUnprocessedPath();

  info();

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


void Car::updateClosestVehicles(const std::vector<std::vector<double>>& sensor_fusion) {
  closest_front_cars_.clear();
  closest_rear_cars_.clear();

  // sensor_fusion: [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  for (auto& v : sensor_fusion) {
    if (std::abs(v[5] - ps_) > max_tracking_distance) continue;

    uint16_t lane_id = map_.getLaneId(v[6]);
    double vs = std::sqrt(v[3]*v[3] + v[4]*v[4]); // an estimation
    if (v[5] > ps_) {
      // front vehicle
      if (closest_front_cars_.find(lane_id) != closest_front_cars_.end()) {
        double min_s = closest_front_cars_[lane_id].first[0];
        if (v[5] < min_s) {
          closest_front_cars_[lane_id] = {{v[5], vs, 0}, {0, 0, 0}};
        }
      } else {
        closest_front_cars_[lane_id] = {{v[5], vs, 0}, {0, 0, 0}};
      }
    } else {
      // rear vehicle
      if (closest_rear_cars_.find(lane_id) != closest_rear_cars_.end()) {
        double max_s = closest_rear_cars_[lane_id].first[0];
        if (v[5] > max_s) {
          closest_rear_cars_[lane_id] = {{v[5], vs, 0}, {0, 0, 0}};
        }
      } else {
        closest_rear_cars_[lane_id] = {{v[5], vs, 0}, {0, 0, 0}};
      }
    }
  }
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

Car::dynamics Car::estimateFinalDynamics() const {
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

void Car::planPath() {
  truncatePath(5);

  opt_paths_.clear();
  opt_paths_.push_back(path_optimizer_.keepLane(this));
  opt_paths_.push_back(path_optimizer_.changeLaneLeft(this));
  opt_paths_.push_back(path_optimizer_.changeLaneRight(this));

  auto new_path_s = opt_paths_[0].first;
  auto new_path_d = opt_paths_[0].second;

  path_s_.insert(path_s_.end(), new_path_s.begin(), new_path_s.end());
  path_d_.insert(path_d_.end(), new_path_d.begin(), new_path_d.end());
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

void Car::info() const {
  std::cout << "Car is running at lane " << getCurrentLaneId() << "\n"
            << "px = " << px_ << ", " << "py = " << py_ << ", "
            << "vx = " << vx_ << ", " << "vy = " << vy_ << "\n"
            << "ps = " << ps_ << ", " << "pd = " << pd_ << ", "
            << "vs = " << vs_ << ", " << "vd = " << vd_ << "\n";

  for (uint16_t i=1; i<=map_.n_lanes; ++i) {
    std::cout << "Lane " << i << ": ";
    if (closest_front_cars_.find(i) != closest_front_cars_.end())
      std::cout << "closest front car has ds = " << closest_front_cars_.at(i).first[0] - ps_ << ", ";
    if (closest_rear_cars_.find(i) != closest_rear_cars_.end())
      std::cout << "closest rear car has ds = " << closest_rear_cars_.at(i).first[0] - ps_;
    std::cout << "\n";
  }

  std::cout << std::endl;
}

std::map<uint16_t, Car::dynamics> Car::getClosestFrontVehicles() const { return closest_front_cars_; }
std::map<uint16_t, Car::dynamics> Car::getClosestRearVehicles() const { return closest_rear_cars_; }

uint16_t Car::getCurrentLaneId() const { return map_.getLaneId(pd_); }
double Car::getCurrentLaneCenter() const { return map_.getLaneCenter(map_.getLaneId(pd_)); }

uint16_t Car::getTargetLaneId() const { return target_lane_id_; }
