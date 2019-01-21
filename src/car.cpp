#include "car.hpp"
#include "path_optimizer.hpp"
#include "trajectory.hpp"
#include "map.hpp"


double Car::inf_dist = 1.0e6;

class Car::State {

protected:
  uint16_t tick_ = 0;

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


Car::Car(const std::string& file_path, double time_step) :
  is_initialized_(false),
  time_step_(time_step),
  map_(new Map(file_path)),
  state_(createState(States::ST)),
  path_opt_(new PathOptimizer(mph2mps(47.5), 9.5, 9.5, time_step))
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

    for (auto i = 1; i <= map_->n_lanes; ++i) {
      closest_front_cars_[i] = {{inf_dist, 0, 0}, {0, 0, 0}};
      closest_rear_cars_[i] = {{-inf_dist, 0, 0}, {0, 0, 0}};
    }

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
    dyn_s[0] = inf_dist;
    dyn_s[1] = 0;
    dyn_s[2] = 0;

    auto& dyn_d = v.second.second;
    dyn_d[0] = 0;
    dyn_d[1] = 0;
    dyn_d[2] = 0;
  }
  for (auto& v : closest_rear_cars_) {
    auto& dyn_s = v.second.first;
    dyn_s[0] = - inf_dist;
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

    if (std::abs(d_ps) > max_tracking_dist_) continue;

    uint16_t id = map_->getLaneId(v[6]);
    // Sensor fusion could return data with d outside of the lanes.
    if (id == 0 || id > map_->n_lanes) continue;

    double vs = std::sqrt(v[3]*v[3] + v[4]*v[4]); // an estimation
    if (d_ps > 0) {
      // front vehicle
      dynamics& dyn = closest_front_cars_[id];
      if (d_ps < dyn.first[0]) dyn = {{d_ps, vs, 0}, {0, 0, 0}};
    } else {
      // rear vehicle
      dynamics& dyn = closest_rear_cars_[id];
      if (d_ps > dyn.first[0]) dyn = {{d_ps, vs, 0}, {0, 0, 0}};
    }
  }
}


void Car::updateUnprocessedPath() {
  if (!path_s_.empty()) {
    // the path includes the starting point
    // TODO: check
    if (path_s_.back() > map_->max_s && path_s_.front() > ps_) ps_ += map_->max_s;

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
                                      closest_front_cars_.at(getCurrentLaneId()),
                                      getCurrentLaneCenter());

  if (new_path.first.empty()) {
    std::cerr << "Failed to find a path for keepLane! \n";
    return false;
  }
  extendPath(std::move(new_path));
  return true;
}

bool Car::changeLane() {
  truncatePath(5);

  auto new_path = path_opt_->keepLane(estimateFinalDynamics(),
                                      closest_front_cars_.at(getTargetLaneId()),
                                      getTargetLaneCenter());

  if (new_path.first.empty() || checkAllCollisions(new_path)) {
    new_path = path_opt_->keepLane(estimateFinalDynamics(),
                                   closest_front_cars_.at(getCurrentLaneId()),
                                   getCurrentLaneCenter());

    return false;
  }

  extendPath(std::move(new_path));
  return true;
}

trajectory Car::getPathXY() const {
  std::vector<double> path_x;
  std::vector<double> path_y;
  for (std::size_t i = 0; i < path_s_.size(); ++i) {
    position pxy = frenetToCartesian(path_s_[i], path_d_[i], map_->s, map_->max_s, map_->x, map_->y);
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

  for (uint16_t i=1; i<=map_->n_lanes; ++i) {
    std::cout << "Lane " << i << ": ";
    std::cout << closest_front_cars_.at(i).first[0] << " m to the closest front car, ";
    std::cout << closest_rear_cars_.at(i).first[0] << " m to the closest rear car.\n";
  }

  std::cout << std::endl;
}

uint16_t Car::getOptimizedLaneId() const {
  double prediction_time = 4;

  // This function does not take care of whether it is feasible to change
  // lane in order to reach the optimized lane.
  uint16_t n_lanes = map_->n_lanes;
  uint16_t current_id = getCurrentLaneId();

  const auto& front_dyn = closest_front_cars_.at(current_id);
  // Do not change lane if the front car is far away or significantly faster
  if (front_dyn.first[0] > 2.0 * vs_ || front_dyn.first[1] > 1.2 * vs_)
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

  for (uint16_t i=0; i<path.first.size(); ++i) {
    ps += vs * time_step_;
    pd += vd * time_step_;

    // two vehicles are in different lanes
    if (std::abs(pd - path.second[i]) >= map_->lane_width) continue;

    if (distance(path.first[i], path.second[i], ps, pd) < safe_dist) return true;
  }

  return false;
}

bool Car::checkAllCollisions(const trajectory& path) const {
  uint16_t current_id = getCurrentLaneId();
  uint16_t target_id = getTargetLaneId();

  // check the front vehicle in the current line
  if (checkCollision(path, closest_front_cars_.at(current_id))) return true;

  // check the front and rear vehicles in the target line
  if (target_id != current_id) {
    if (checkCollision(path, closest_front_cars_.at(target_id))) return true;
    if (checkCollision(path, closest_rear_cars_.at(target_id))) return true;
  }

  return false;
}

const std::map<uint16_t, dynamics>& Car::getClosestFrontVehicles() const { return closest_front_cars_; }
const std::map<uint16_t, dynamics>& Car::getClosestRearVehicles() const { return closest_rear_cars_; }

uint16_t Car::getCurrentLaneId() const { return map_->getLaneId(pd_); }
double Car::getCurrentLaneCenter() const { return map_->getLaneCenter(map_->getLaneId(pd_)); }

void Car::setTargetLaneId(uint16_t id) { target_lane_id_ = id; }
uint16_t Car::getTargetLaneId() const { return target_lane_id_; }
double Car::getTargetLaneCenter() const { return map_->getLaneCenter(target_lane_id_); }

double Car::getCurrentSpeed() const { return vs_; }
