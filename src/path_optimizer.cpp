#include <cmath>

#include "path_optimizer.hpp"
#include "map.hpp"
#include "jmt.hpp"


PathOptimizer::PathOptimizer(std::shared_ptr<Map> map,
                             double time_step, double speed_limit, double acc_limit, double jerk_limit) :
  map_(std::move(map)),
  time_step_(time_step),
  speed_limit_(speed_limit),
  acc_limit_(acc_limit),
  jerk_limit_(jerk_limit)
{
}

PathOptimizer::~PathOptimizer() = default;

bool PathOptimizer::validatePath(const dynamics& dyn, const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d,
                                 double delta_t) {
  double v_sqr_limit = speed_limit_ * speed_limit_;
  double a_sqr_limit = acc_limit_ * acc_limit_;
  double j_sqr_limit = jerk_limit_ * jerk_limit_;

  double ps = dyn.first[0];
  double vs = dyn.first[1];
  double as = dyn.first[2];
  double pd = dyn.second[0];
  double vd = dyn.second[1];
  double ad = dyn.second[2];

  double ps0 = ps; //

  double t = 0;
  while (t < delta_t) {
    t += time_step_;

    double ps1 = evalTrajectory(coeff_s, t);
    double pd1 = evalTrajectory(coeff_d, t);

    uint16_t current_lane_id = map_->getLaneId(pd1);

    double vs1 = (ps1 - ps) / time_step_;
    if (vs < 0) {
//      std::cout << "vs cannot be negative: " << vs << std::endl;
      return false; // why there could be path with negative speed?
    }
    double vd1 = (pd1 - pd) / time_step_;;
    double v_sqr = vs1 * vs1 + vd1 * vd1;
    if (v_sqr > v_sqr_limit) {
//      std::cout << "violate the speed limit: " << vs << ", " << vd << std::endl;
      return false;
    }

    double as1 = (vs1 - vs) / time_step_;
    double ad1 = (vd1 - vd) / time_step_;
    double a_sqr = as1 * as1 + ad1 * ad1;
    if (a_sqr > a_sqr_limit) {
//      std::cout << "violate the acceleration limit: " << as << ", " << ad << std::endl;
      return false;
    }

    double js = (as1 - as) / time_step_;
    double jd = (ad1 - ad) / time_step_;
    double j_sqr = js * js + jd * jd;
    if (j_sqr > j_sqr_limit) {
//      std::cout << "violate the jerk limit: " << js << ", " << jd << std::endl;
      return false;
    }

    ps = ps1;
    pd = pd1;
    vs = vs1;
    vd = vd1;
    as = as1;
    ad = ad1;
  }

  return true;
}

trajectory
PathOptimizer::computeJmtPath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d, double delta_t) {
  double t = 0;
  std::vector<double> path_s;
  std::vector<double> path_d;
  while (t < delta_t) {
    t += time_step_;
    double ps = evalTrajectory(coeff_s, t);
    double pd = evalTrajectory(coeff_d, t);
    path_s.push_back(ps);
    path_d.push_back(pd);
  }

  return {path_s, path_d};
}

trajectory PathOptimizer::startUp(dynamics&& dyn) {
  std::vector<double> path_s;
  std::vector<double> path_d;

  // At very low speed, we apply a simple dynamics model:
  // 1. The car is accelerated as much as allowed along the s direction;
  // 2. The car does not move in the d direction.
  double delta_t = 2.0;

  double t = 0;
  double ps_f = dyn.first[0];
  double vs_f = dyn.first[1];
  double as_f = dyn.first[2];
  double pd_f = dyn.second[0];

  while (t < delta_t) {
    t += time_step_;

    if (as_f < acc_limit_) {
      as_f += time_step_ * jerk_limit_;
      if (as_f > acc_limit_) as_f = acc_limit_;
    }

    ps_f += 0.5 * time_step_ * (2 * vs_f + time_step_ * as_f);
    path_s.push_back(ps_f);
    vs_f += time_step_ * as_f;

    path_d.push_back(pd_f);
  }

  return {path_s, path_d};
}

trajectory PathOptimizer::keepLane(dynamics&& dyn, const dynamics& front_vehicle) {
  // determine when to start slowing down if there is a slow front car
  double safe_factor = 1.5;
  // determine how close it will follow the front car in case of traffic jam.
  double safe_dist = 10;

  double d_ps = safe_factor * dyn.first[1];
  double vs_f = speed_limit_;
  if (front_vehicle.first[0] <= d_ps) {
    d_ps = front_vehicle.first[0];
    vs_f = front_vehicle.first[1];
    if (d_ps < safe_dist) vs_f *= d_ps / safe_dist;
  }

  double ps_f = dyn.first[0] + d_ps;

  double pd_f = map_->getLaneCenter(map_->getLaneId(dyn.second[0]));
  double vd_f = 0;

  // (for simplicity) final longitudinal and transverse accelerations are always 0
  return searchOptimizedJMT({dyn.first, dyn.second}, {{ps_f, vs_f, 0}, {pd_f, vd_f, 0}});
}

trajectory PathOptimizer::changeLane(dynamics&& dyn, const dynamics& front_vehicle, uint16_t target_lane_id) {
  double d_ps = dyn.first[1];
  double vs_f = speed_limit_;
  if (front_vehicle.first[0] < d_ps) {
    d_ps = front_vehicle.first[0];
    vs_f = front_vehicle.first[1];
  }

  double ps_f = dyn.first[0] + d_ps;

  double pd_f = map_->getLaneCenter(target_lane_id);
  double vd_f = 0;

  // (for simplicity) final longitudinal and transverse accelerations are always 0
  return searchOptimizedJMT({dyn.first, dyn.second}, {{ps_f, vs_f, 0}, {pd_f, vd_f, 0}});
}

trajectory PathOptimizer::searchOptimizedJMT(dynamics&& dyn, dynamics&& dyn_f) {
  polynomial_coeff coeff_s;
  polynomial_coeff coeff_d;

  double delta_t = 0;
  while (true) {
    // Increase the distance and reset time if no valid path is found.
    if (delta_t > time_limit_) {
      delta_t = 0;
      dyn_f.first[0] += dist_step_;

      // reach the maximum possible distance and failed to find a path, return an empty path?
      if (dyn_f.first[0] - dyn.first[0] > time_limit_ * speed_limit_) return {{}, {}};
    }

    delta_t += 10 * time_step_; // use a course time grid for searching
    if (delta_t * speed_limit_ < dyn_f.first[0] - dyn.first[0]) continue;

    coeff_s = jerkMinimizingTrajectory(dyn.first, dyn_f.first, delta_t);
    coeff_d = jerkMinimizingTrajectory(dyn.second, dyn_f.second, delta_t);

    // For a given ps_f, the algorithm guarantees that is a valid path exists, the first
    // valid path found takes the shortest time.
    if (validatePath(dyn, coeff_s, coeff_d, delta_t))
      return computeJmtPath(coeff_s, coeff_d, delta_t);
  }
}
