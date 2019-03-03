#include <cmath>
#include <vector>

#include <boost/log/trivial.hpp>

#include "car.hpp"
#include "path_optimizer.hpp"


class Car::State {

protected:
  uint16_t tick_ = 0;
  uint16_t max_tick_ = 5; // determine the frequency of planning new path

  State() = default;

public:

  virtual ~State() = default;

  // Return a pointer to the next state if a certain condition is met. Otherwise nullptr.
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
    if (car.vs_ > 10)
      return createState(States::KL);
    else return nullptr;
  }

  void onEnter(Car& car) override {
    BOOST_LOG_TRIVIAL(info) << "Enter state: *** ON ***";
  }

  void onUpdate(Car &car) override {
    if (tick_++ == 0) car.startUp();
    if (tick_ == max_tick_) tick_ = 0;
  }

  void onExit(Car& car) override {
    BOOST_LOG_TRIVIAL(info) << "Exit state: *** ON ***";
  }
};


class Car::StateKeepLane : public Car::State {

public:

  StateKeepLane() = default;

  ~StateKeepLane() override = default;

  State*getNextState(Car &car) override {
    auto opt_id = car.getOptimizedLaneId();
    if (opt_id != car.getCurrentLaneId()) {
      BOOST_LOG_TRIVIAL(debug) << "Optimized lane ID is: " << opt_id;
      car.setTargetLaneId(opt_id);
      return createState(States::CL);
    }

    return nullptr;
  }

  void onEnter(Car& car) override {
    BOOST_LOG_TRIVIAL(info) << "Enter state: *** KEEP LANE ***";
    car.setTargetLaneId(car.getCurrentLaneId());
  }

  void onUpdate(Car &car) override {
    if (tick_++ == 0) car.keepLane();
    if (tick_ == max_tick_) tick_ = 0;
  }

  void onExit(Car& car) override {
    BOOST_LOG_TRIVIAL(info) << "Exit state: *** KEEP LANE ***";
  }
};


class Car::StateChangeLane : public Car::State {

  uint16_t nf_ = 0; // number of failures of finding lane change path
  uint16_t max_attempt_ = 3;

public:

  StateChangeLane() = default;

  ~StateChangeLane() override = default;

  State* getNextState(Car &car) override {
    if (nf_ >= max_attempt_ || car.getCurrentLaneId() == car.getTargetLaneId())
      return createState(States::KL);
    return nullptr;
  }

  void onEnter(Car& car) override {
    BOOST_LOG_TRIVIAL(info)
      << "Enter state: *** CHANGE LANE *** from Lane-" << car.getCurrentLaneId()
      << " to Lane-" << car.getTargetLaneId() << std::endl;
  }

  void onUpdate(Car& car) override {
    if (tick_++ == 0) {
      if (!car.changeLane()) {
        ++nf_;
        BOOST_LOG_TRIVIAL(warning) << "Failed to find a path! Number of attempts: " << nf_ << "\n";
      } else nf_ = 0;
    }
    if (tick_ == max_tick_) tick_ = 0;
  }

  void onExit(Car& car) override {
    BOOST_LOG_TRIVIAL(info) << "Exit state: *** CHANGE LANE *** ";
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


Car::Car(double time_step, double speed_limit, double acc_limit, double jerk_limit) :
  is_initialized_(false),
  time_step_(time_step),
  speed_limit_(speed_limit),
  acc_limit_(acc_limit),
  jerk_limit_(jerk_limit),
  state_(createState(States::ST))
{
  state_->onEnter(*this);
}

Car::~Car() = default;

void Car::loadMap(const std::string &file_path) {
  map_ = std::make_shared<Map>(file_path);

  path_opt_.reset(new PathOptimizer(map_, time_step_, speed_limit_, acc_limit_, jerk_limit_));

  // initializing closest vehicles
  for (auto i = 1; i <= map_->nLanes(); ++i) {
    closest_front_cars_[i] = {{inf_dist_, 0, 0}, {0, 0, 0}};
    closest_rear_cars_[i] = {{-inf_dist_, 0, 0}, {0, 0, 0}};
  }
}

void Car::update(const std::vector<double>& localization,
                 const std::vector<std::vector<double>>& sensor_fusion) {

  if (not map_) return;

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
    double dx = px - px_;
    double dy = py - py_;
    // estimate dt
    double dt;
    if (std::abs(dx) > std::abs(dy)) dt = dx / vx;
    else dt = dy / vy;

    ax_ = (vx - vx_) / dt;
    ay_ = (vy - vy_) / dt;

    double vs = (ps - ps_) / dt;
    double vd = (pd - pd_) / dt;
    as_ = (vs - vs_) / dt;
    ad_ = (vd - vd_) / dt;
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
  for (auto& v : closest_front_cars_) {
    auto& dyn_s = v.second.first;
    dyn_s[0] = inf_dist_;
    dyn_s[1] = 0;
    dyn_s[2] = 0;

    auto& dyn_d = v.second.second;
    dyn_d[0] = 0;
    dyn_d[1] = 0;
    dyn_d[2] = 0;
  }
  for (auto& v : closest_rear_cars_) {
    auto& dyn_s = v.second.first;
    dyn_s[0] = - inf_dist_;
    dyn_s[1] = 0;
    dyn_s[2] = 0;

    auto& dyn_d = v.second.second;
    dyn_d[0] = 0;
    dyn_d[1] = 0;
    dyn_d[2] = 0;
  }

  // sensor_fusion: [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  for (auto& v : sensor_fusion) {
    // TODO: Fix when a new lap starts
    double d_ps = v[5] - ps_;

    uint16_t id = map_->getLaneId(v[6]);
    // Sensor fusion could return data with d outside of the lanes.
    if (id == 0 || id > map_->nLanes()) continue;

    double vs = std::sqrt(v[3]*v[3] + v[4]*v[4]); // an estimation
    if (d_ps > 0) {
      // front vehicle
      dynamics& dyn = closest_front_cars_[id];
      if (d_ps < dyn.first[0]) dyn = {{d_ps, vs, 0}, {v[6], 0, 0}};
    } else {
      // rear vehicle
      dynamics& dyn = closest_rear_cars_[id];
      if (d_ps > dyn.first[0]) dyn = {{d_ps, vs, 0}, {v[6], 0, 0}};
    }
  }
}


void Car::updateUnprocessedPath() {
  if (!path_s_.empty()) {
    // the path includes the starting point
    // TODO: check
    if (path_s_.back() > map_->maxS() && path_s_.front() > ps_) ps_ +=  map_->maxS();

    auto it_s = std::lower_bound(path_s_.begin(), path_s_.end(), ps_);
    auto it_d = std::next(path_d_.begin(), std::distance(path_s_.begin(), it_s));

    // it_s and it_d will be not removed
    path_s_.erase(path_s_.begin(), it_s);
    path_d_.erase(path_d_.begin(), it_d);
  }
}

void Car::truncatePath(unsigned int n_keep) {
  if (path_s_.size() > n_keep) {
    path_s_.erase(path_s_.begin() + n_keep, path_s_.end());
    path_d_.erase(path_d_.begin() + n_keep, path_d_.end());
  }
}

void Car::extendPath(trajectory&& traj) {
//  std::cout << "Old path: \n";
//  for (auto i=0; i<path_s_.size(); ++i)
//    std::cout << path_s_[i] << ", " << path_d_[i] << "\n";
//  std::cout << std::endl;

  path_s_.insert(path_s_.end(), traj.first.begin(), traj.first.end());
  path_d_.insert(path_d_.end(), traj.second.begin(), traj.second.end());

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
  extendPath(path_opt_->startUp(estimateFinalDynamics()));
  return true;
}

bool Car::keepLane() {
  truncatePath(5);

  auto new_path = path_opt_->keepLane(estimateFinalDynamics(),
                                      closest_front_cars_.at(getCurrentLaneId()));

  if (new_path.first.empty()) {
    BOOST_LOG_TRIVIAL(error) << "Failed to find a path for keepLane!";
    return false;
  }
  extendPath(std::move(new_path));
  return true;
}

bool Car::changeLane() {
  truncatePath(5);

  auto new_path = path_opt_->changeLane(estimateFinalDynamics(),
                                        closest_front_cars_.at(getTargetLaneId()),
                                        target_lane_id_);

  if (new_path.first.empty()) {
    BOOST_LOG_TRIVIAL(error) << "Failed to find a path for changeLane!";
    return false;
  }
  extendPath(std::move(new_path));
  return true;
}

trajectory Car::getPathXY() const {
  std::vector<double> path_x;
  std::vector<double> path_y;
  for (std::size_t i = 0; i < path_s_.size(); ++i) {
    position pxy = map_->frenetToCartesian(path_s_[i], path_d_[i]);
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

  for (uint16_t i=1; i<=map_->nLanes(); ++i) {
    std::cout << "Lane " << i << ": ";
    std::cout << closest_front_cars_.at(i).first[0] << " m to the closest front car, ";
    std::cout << closest_rear_cars_.at(i).first[0] << " m to the closest rear car.\n";
  }

  std::cout << std::endl;
}

uint16_t Car::getOptimizedLaneId() const {
  double prediction_time = 4;

  uint16_t n_lanes = map_->nLanes();
  uint16_t current_id = getCurrentLaneId();

  const auto& front_dyn = closest_front_cars_.at(current_id);
  // Do not change lane if the front car is far away or very close or significantly faster
  if (front_dyn.first[0] > 3.0 * vs_ || front_dyn.first[1] > 1.2 * vs_)
    return current_id;

  double opt_dist = 0; // farthest distance to the front car
  uint16_t opt_id = current_id; // lane id with the farthest distance to the front car

  // bonus for the current lane
  double bonus = front_dyn.first[1];

  // from the left lane to the right lane (prefer to overtake via the left lane)
  for (uint16_t i=1; i<=n_lanes; ++i) {
    const auto& dyn = closest_front_cars_.at(i);

    // assume the car moves at a constant speed
    auto dist = dyn.first[0] + prediction_time * dyn.first[1];

    if (i == current_id) dist += bonus;

    if (dist > opt_dist) {
      opt_dist = dist;
      opt_id = i;
    }
  }

  // only allow to change to the next lane
  if (opt_id - current_id > 1) opt_id = current_id + 1u;
  if (current_id - opt_id > 1) opt_id = current_id - 1u;

  if (opt_id != current_id) {
    auto& rear_car = closest_rear_cars_.at(opt_id);
    double d_vs = rear_car.first[1] - vs_;
    // rear car is fast and too close
    if (rear_car.first[0] > -3.0 || 2.0 * d_vs + rear_car.first[0] > 0) return current_id;
  }

  return opt_id;
}

const cars_on_road& Car::getClosestFrontVehicles() const { return closest_front_cars_; }
const cars_on_road& Car::getClosestRearVehicles() const { return closest_rear_cars_; }

uint16_t Car::getCurrentLaneId() const { return map_->getLaneId(pd_); }

void Car::setTargetLaneId(uint16_t id) { target_lane_id_ = id; }
uint16_t Car::getTargetLaneId() const { return target_lane_id_; }
