#include "car.hpp"
#include "trajectory.hpp"
#include "jmt.hpp"


/*
 * PathOptimizer
 */

PathOptimizer::PathOptimizer() = default;

PathOptimizer::~PathOptimizer() = default;

bool PathOptimizer::validatePath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d,
                                 double delta_t, double time_step,
                                 double speed_limit, double acceleration_limit, double jerk_limit) {
  double v_sqr_limit = speed_limit * speed_limit;
  double a_sqr_limit = acceleration_limit * acceleration_limit;
  double j_sqr_limit = jerk_limit * jerk_limit;

  double t = 0;
  while (t < delta_t) {
    t += time_step;

    double vs = evalVelocity(coeff_s, t);
    if (vs < 0) {
//      std::cout << "vs cannot be negative: " << vs << std::endl;
      return false; // why there could be path with negative speed?
    }
    double vd = evalVelocity(coeff_d, t);
    double v_sqr = vs*vs + vd*vd;
    if (v_sqr > v_sqr_limit) {
//      std::cout << "violate the speed limit: " << vs << ", " << vd << std::endl;
      return false;
    }

    double as = evalAcceleration(coeff_s, t);
    double ad = evalAcceleration(coeff_d, t);
    double a_sqr = as*as + ad*ad;
    if (a_sqr > a_sqr_limit) {
//      std::cout << "violate the acceleration limit: " << as << ", " << ad << std::endl;
      return false;
    }

    double js = evalJerk(coeff_s, t);
    double jd = evalJerk(coeff_d, t);
    double j_sqr = js*js + jd*jd;
    if (j_sqr > j_sqr_limit) {
//      std::cout << "violate the jerk limit: " << js << ", " << jd << std::endl;
      return false;
    }
  }

  return true;
}

trajectory
PathOptimizer::computeJmtPath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d,
                              double delta_t, double time_step) {
  double t = 0;
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

trajectory  PathOptimizer::startUp(Car* car) {
  auto dynamics0 = car->estimateFinalDynamics();
  std::vector<double> dynamics_s0 = dynamics0.first;
  std::vector<double> dynamics_d0 = dynamics0.second;

  double time_step = car->time_step_;
  std::vector<double> path_s;
  std::vector<double> path_d;

  // At very low speed, we apply a simple dynamics model:
  // 1. The car is accelerated as much as allowed along the s direction;
  // 2. The car does not move in the d direction.
  double delta_t = 2.0;

  double t = 0;
  double ps_f = dynamics_s0[0];
  double vs_f = dynamics_s0[1];
  double as_f = dynamics_s0[2];
  double pd_f = dynamics_d0[0];

  while (t < delta_t) {
    t += time_step;

    if (as_f < car->max_acceleration_) {
      as_f += time_step * car->max_jerk_;
      if (as_f > car->max_acceleration_) as_f = car->max_acceleration_;
    }

    ps_f += 0.5 * time_step * (2 * vs_f + time_step * as_f);
    path_s.push_back(ps_f);
    vs_f += time_step * as_f;

    path_d.push_back(pd_f);
  }

  return {path_s, path_d};
}

trajectory PathOptimizer::keepLane(Car* car) {

  double safe_distance = 20; // safe distance (in meter) after a front car
  double min_dist_no_car = 30; // min delta s of a path if there is no front car

  auto dyn = car->estimateFinalDynamics();
  std::vector<double> dyn_s = dyn.first;
  std::vector<double> dyn_d = dyn.second;

  double ps_f = dyn_s[0] + min_dist_no_car;
  double vs_f = car->max_speed_;
  uint16_t lane_id = car->getCurrentLaneId();
  if (car->closest_front_cars_.find(lane_id) != car->closest_front_cars_.end()) {
    double ps_front_car = car->closest_front_cars_[lane_id].first[0];
    double vs_front_car = car->closest_front_cars_[lane_id].first[1];
    if (ps_front_car < dyn_s[0] + safe_distance) {
      ps_f = car->closest_front_cars_[lane_id].first[0];
      vs_f = std::min(vs_f, vs_front_car * (ps_front_car - dyn_s[0]) / safe_distance);
    }
  }

  double pd_f = car->getCurrentLaneCenter();
  double vd_f = 0;

  return searchOptimizedJMT(dyn_s, dyn_d, ps_f, vs_f, pd_f, vd_f, car->time_step_, 1.0, 5.0,
                            car->max_speed_, car->max_acceleration_, car->max_jerk_);
}

trajectory PathOptimizer::changeLane(Car* car) {
  double safe_distance = 20; // safe distance (in meter) after a front car
  double min_dist_no_car = 30; // min delta s of a path if there is no front car

  auto dyn = car->estimateFinalDynamics();
  std::vector<double> dyn_s = dyn.first;
  std::vector<double> dyn_d = dyn.second;

  double ps_f = dyn_s[0] + min_dist_no_car;
  double vs_f = car->max_speed_;
  uint16_t lane_id = car->getTargetLaneId();
  if (car->closest_front_cars_.find(lane_id) != car->closest_front_cars_.end()) {
    double ps_front_car = car->closest_front_cars_[lane_id].first[0];
    double vs_front_car = car->closest_front_cars_[lane_id].first[1];
    if (ps_front_car < dyn_s[0] + safe_distance) {
      ps_f = car->closest_front_cars_[lane_id].first[0];
      vs_f = std::min(vs_f, vs_front_car * (ps_front_car - dyn_s[0]) / safe_distance);
    }
  }

  double pd_f = car->getTargetLaneCenter();
  double vd_f = 0;

  return searchOptimizedJMT(dyn_s, dyn_d, ps_f, vs_f, pd_f, vd_f, car->time_step_, 1.0, 5.0,
                            car->max_speed_, car->max_acceleration_, car->max_jerk_);
}

trajectory
PathOptimizer::searchOptimizedJMT(const std::vector<double>& dyn_s, const std::vector<double>& dyn_d,
                                  double ps_f, double vs_f, double pd_f, double vd_f,
                                  double time_step, double dist_step, double time_limit,
                                  double speed_limit, double acc_limit, double jerk_limit) {
  polynomial_coeff coeff_s;
  polynomial_coeff coeff_d;

  // (for simplicity) final accelerations and transverse accelerations are always 0
  double as_f = 0;
  double ad_f = 0;

  double delta_t = 0;

  bool valid = false;
  while (true) {
    // Increase the distance and reset time if no valid path is found.
    if (delta_t > time_limit) {
      delta_t = 0;
      ps_f += dist_step;
    }

    delta_t += 10 * time_step; // use a course time grid for searching

    std::vector<double> dyn_s_f = {ps_f, vs_f, as_f};
    std::vector<double> dyn_d_f = {pd_f, vd_f, ad_f};

    coeff_s = jerkMinimizingTrajectory(dyn_s, dyn_s_f, delta_t);
    coeff_d = jerkMinimizingTrajectory(dyn_d, dyn_d_f, delta_t);

    // For a given ps_f, the algorithm guarantees that is a valid path exists, the first
    // valid path found takes the shortest time.
    valid = validatePath(coeff_s, coeff_d, delta_t, time_step, speed_limit, acc_limit, jerk_limit);
    if (valid) return computeJmtPath(coeff_s, coeff_d, delta_t, time_step);

    // reach the maximum possible distance and failed to find a path, return an empty path?
    if (ps_f - dyn_s[0] > time_limit * speed_limit) return {{}, {}};
  }


}

/*
 * Car
 */

class Car::State {

protected:
  uint16_t tick_ = 0;

  State() = default;

public:

  virtual ~State() = default;

  // Check the validity of the transition states one-by-one. If valid.
  // return the next state.
  virtual State* getNextState(Car& car) = 0;

  // action when entering a state
  virtual void onEnter(Car& car) = 0;

  // action when updating a state
  virtual void onUpdate(Car& car) = 0;

  // action when exiting a state
  virtual void onExit(Car& car) = 0;

};


class Car::StateStartUp : public Car::State {

public:

  StateStartUp() = default;

  ~StateStartUp() override = default;

  State* getNextState(Car &car) override {
    if (car.getCurrentSpeed() > 10)
      return createState(States::KL);
    else return nullptr;
  }

  void onEnter(Car& car) override {
    std::cout << "Enter state: *** ON ***" << std::endl;
  }

  void onUpdate(Car &car) override {
    if (tick_ == 0) car.startUp();
    ++tick_;
    if (tick_ == 5) tick_ = 0;
  }

  void onExit(Car& car) override {
    std::cout << "Exit state: *** ON ***" << std::endl;
  }
};


class Car::StateKeepLane : public Car::State {

public:

  StateKeepLane() = default;

  ~StateKeepLane() override = default;

  State*getNextState(Car &car) override {
    auto opt_id = car.getOptimizedLaneId();
    if (opt_id != car.getCurrentLaneId()) {
      car.setTargetLaneId(opt_id);
      return createState(States::CL);
    }

    return nullptr;
  }

  void onEnter(Car& car) override {
    std::cout << "Enter state: *** KEEP LANE ***" << std::endl;
  }

  void onUpdate(Car &car) override {
    if (tick_ == 0) car.keepLane();
    ++tick_;
    if (tick_ == 5) tick_ = 0;
  }

  void onExit(Car& car) override {
    std::cout << "Exit state: *** KEEP LANE ***" << std::endl;
  }
};


class Car::StateChangeLane : public Car::State {

  uint16_t nf_ = 0; // number of failures of finding lane change path
  uint16_t max_attempt_ = 5;

public:

  StateChangeLane() = default;

  ~StateChangeLane() override = default;

  State* getNextState(Car &car) override {
    if (nf_ >= max_attempt_ || car.getCurrentLaneId() == car.getTargetLaneId())
      return createState(States::KL);
    return nullptr;
  }

  void onEnter(Car& car) override {
    std::cout << "Enter state: *** CHANGE LANE *** from Lane-" << car.getCurrentLaneId()
              << " to Lane-" << car.getTargetLaneId() << std::endl;
  }

  void onUpdate(Car& car) override {
    if (!car.changeLane()) {
      ++nf_;
      std::cout << "Failed to find a path! Number of attempts: " << nf_ << "\n";
    } else nf_ = 0;
  }

  void onExit(Car& car) override {
    std::cout << "Exit state: *** CHANGE LANE *** " << std::endl;
  }
};


Car::State* Car::createState(States name) {
  switch(name) {
    case States::CL:
      return new Car::StateChangeLane;
    case States::KL:
      return new Car::StateKeepLane;
    case States::ST:
      return new Car::StateStartUp;
    default:
      throw std::invalid_argument("Unknown state!");
  }
}


Car::Car(const Map& map, double time_step) :
  is_initialized_(false),
  time_step_(time_step),
  map_(map),
  state_(createState(States::ST))
{
  state_->onEnter(*this);
}

Car::~Car() = default;

void Car::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {
  updateParameters(localization);
  updateClosestVehicles(sensor_fusion);
  updateUnprocessedPath();

//  info();

  state_->onUpdate(*this);
  std::unique_ptr<State> state(state_->getNextState(*this));
  if (state != nullptr) {
    state_->onExit(*this);
    state_ = std::move(state);
    state_->onEnter(*this);
  }
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

void Car::extendPath(trajectory&& traj) {
  auto path_s = traj.first;
  auto path_d = traj.second;

  //  std::cout << "Old path: \n";
  //  for (auto i=0; i<path_s_.size(); ++i)
  //    std::cout << path_s_[i] << ", " << path_d_[i] << "\n";
  //  std::cout << std::endl;

  path_s_.insert(path_s_.end(), path_s.begin(), path_s.end());
  path_d_.insert(path_d_.end(), path_d.begin(), path_d.end());

  //  std::cout << "New path: \n";
  //  for (auto i=0; i<path_s_.size(); ++i)
  //    std::cout << path_s_[i] << ", " << path_d_[i] << "\n";
  //  std::cout << std::endl;
}

dynamics Car::estimateFinalDynamics() const {
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

bool Car::startUp() {
  truncatePath(5);
  extendPath(PathOptimizer::startUp(this));
  return true;
}

bool Car::keepLane() {
  truncatePath(5);

  auto new_path = PathOptimizer::keepLane(this);
  if (new_path.first.empty()) return false;
  extendPath(std::move(new_path));
  return true;
}

bool Car::changeLane() {
  truncatePath(5);

  auto new_path = PathOptimizer::changeLane(this);
  if (checkAllCollisions(new_path)) {
    extendPath(PathOptimizer::keepLane(this));
    return false;
  }

  extendPath(std::move(new_path));
  return true;
}

trajectory Car::getPathXY() const {
  std::vector<double> path_x;
  std::vector<double> path_y;
  for (std::size_t i = 0; i < path_s_.size(); ++i) {
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

uint16_t Car::getOptimizedLaneId() const {
  // Lane change will not be considered if the distance to the front
  // vehicle is large.
  double max_dist_at_lane_change = 50;
  double prediction_time = 5;

  // This function does not take care of whether it is feasible to change
  // lane in order to reach the optimized lane.
  uint16_t n_lanes = map_.n_lanes;
  uint16_t current_id = getCurrentLaneId();
  double opt_dist; // distance to the front car

  if (closest_front_cars_.find(current_id) == closest_front_cars_.end())
    // If there is no car in the current lane, we stay.
    return current_id;
  else {
    auto dyn = closest_front_cars_.at(current_id);

    if (dyn.first[0] - ps_ > max_dist_at_lane_change) return current_id;

    // assume the car moves at a constant speed
    opt_dist = dyn.first[0] + prediction_time * dyn.first[1];
  }

  uint16_t opt_id = current_id;
  // from the left lane to the right lane
  for (uint16_t i=1; i<=n_lanes; ++i) {
    if (i == current_id) continue;

    if (closest_front_cars_.find(i) == closest_front_cars_.end()) {
      // prefer to overtake via the left lane
      opt_id = i;
      break;
    } else {
      auto dyn = closest_front_cars_.at(i);
      auto dist = dyn.first[0] + prediction_time * dyn.first[1];
      if (dist > opt_dist) {
        opt_id = i;
        opt_dist = dist;
      }
    }
  }

  std::cout << "Optimized land ID is: " << opt_id << std::endl;

  // only allow to change to the next lane
  if (opt_id - current_id > 1) opt_id = current_id + 1u;
  if (current_id - opt_id > 1) opt_id = current_id - 1u;
  return opt_id;
}

bool Car::checkCollision(const trajectory& path, const dynamics& dyn) const {

  double safe_dist = 10; // safe distance between vehicles (in m)

  double ps = dyn.first[0];
  double vs = dyn.first[1];
  double pd = dyn.second[0];
  double vd = dyn.second[1];

  auto path_s = path.first;
  auto path_d = path.second;
  for (uint16_t i=0; i<path_s.size(); ++i) {
    ps += vs * time_step_;
    pd += vd * time_step_;

    // two vehicles are in different lanes
    if (std::abs(pd - path_d[i]) >= map_.lane_width) continue;

    if (distance(path_s[i], path_d[i], ps, pd) < safe_dist) return true;
  }

  return false;
}

bool Car::checkAllCollisions(const trajectory& path) const {
  uint16_t current_id = getCurrentLaneId();
  uint16_t target_id = getTargetLaneId();

  // check the front vehicle in the current line
  if (closest_front_cars_.find(current_id) != closest_front_cars_.end()) {
    dynamics dyn = closest_front_cars_.at(current_id);
    if (checkCollision(path, dyn)) return true;
  }

  // check the front and rear vehicles in the target line
  if (target_id != current_id) {
    if (closest_front_cars_.find(target_id) != closest_front_cars_.end()) {
      dynamics dyn = closest_front_cars_.at(target_id);
      if (checkCollision(path, dyn)) return true;
    }
    if (closest_rear_cars_.find(target_id) != closest_rear_cars_.end()) {
      dynamics dyn = closest_rear_cars_.at(target_id);
      if (checkCollision(path, dyn)) return true;
    }
  }

  return false;
}

std::map<uint16_t, dynamics> Car::getClosestFrontVehicles() const { return closest_front_cars_; }
std::map<uint16_t, dynamics> Car::getClosestRearVehicles() const { return closest_rear_cars_; }

uint16_t Car::getCurrentLaneId() const { return map_.getLaneId(pd_); }
double Car::getCurrentLaneCenter() const { return map_.getLaneCenter(map_.getLaneId(pd_)); }

void Car::setTargetLaneId(uint16_t id) { target_lane_id_ = id; }
uint16_t Car::getTargetLaneId() const { return target_lane_id_; }
double Car::getTargetLaneCenter() const { return map_.getLaneCenter(target_lane_id_); }

double Car::getCurrentSpeed() const { return vs_; }
double Car::getMaxSpeed() const { return max_speed_; }
