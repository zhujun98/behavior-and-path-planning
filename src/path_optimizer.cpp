//
// Created by jun on 1/20/19.
//

#include "path_optimizer.hpp"
#include "jmt.hpp"


PathOptimizer::PathOptimizer(double speed_limit, double acc_limit, double jerk_limit, double time_step) :
  speed_limit_(speed_limit),
  acc_limit_(acc_limit),
  jerk_limit_(jerk_limit),
  time_step_(time_step)
{
}

PathOptimizer::~PathOptimizer() = default;

bool PathOptimizer::validatePath(const polynomial_coeff& coeff_s, const polynomial_coeff& coeff_d, double delta_t) {
  double v_sqr_limit = speed_limit_ * speed_limit_;
  double a_sqr_limit = acc_limit_ * acc_limit_;
  double j_sqr_limit = jerk_limit_ * jerk_limit_;

  double t = 0;
  while (t < delta_t) {
    t += time_step_;

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

trajectory  PathOptimizer::startUp(dynamics&& dyn) {
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

trajectory
PathOptimizer::keepLane(dynamics&& dyn, const dynamics& closest_front_car, double pd_f) {
  // the default value assumes that the front car is far away (it takes more than 2 seconds to reach it)
  double d_ps = 2.0 * dyn.first[1];
  double vs_f = speed_limit_;
  if (closest_front_car.first[0] <= d_ps) {
    d_ps = closest_front_car.first[0];
    vs_f = closest_front_car.first[1];
  }

  double ps_f = dyn.first[0] + d_ps;
  double vd_f = 0;

  return searchOptimizedJMT({dyn.first, dyn.second}, ps_f, vs_f, pd_f, vd_f, 1.0, 4.0);
}

trajectory
PathOptimizer::changeLane(dynamics&& dyn, const dynamics& closest_front_car, double pd_f) {
  double d_ps = dyn.first[1];
  double vs_f = speed_limit_;
  if (closest_front_car.first[0] < d_ps) {
    d_ps = closest_front_car.first[0];
    vs_f = closest_front_car.first[1];
  }

  double ps_f = dyn.first[0] + d_ps;
  double vd_f = 0;

  // change lane should take place as quickly as possible
  return searchOptimizedJMT({dyn.first, dyn.second}, ps_f, vs_f, pd_f, vd_f, 1.0, 4.0);
}

trajectory
PathOptimizer::searchOptimizedJMT(dynamics&& dyn, double ps_f, double vs_f, double pd_f, double vd_f,
                                  double dist_step, double time_limit) {

  polynomial_coeff coeff_s;
  polynomial_coeff coeff_d;

  // (for simplicity) final accelerations and transverse accelerations are always 0
  double as_f = 0;
  double ad_f = 0;

  double delta_t = 0;

  while (true) {
    // Increase the distance and reset time if no valid path is found.
    if (delta_t > time_limit) {
      delta_t = 0;
      ps_f += dist_step;

      // reach the maximum possible distance and failed to find a path, return an empty path?
      if (ps_f - dyn.first[0] > time_limit * speed_limit_) return {{}, {}};
    }

    delta_t += 10 * time_step_; // use a course time grid for searching
    if (delta_t * speed_limit_ < ps_f - dyn.first[0]) continue;

    std::vector<double> dyn_s_f = {ps_f, vs_f, as_f};
    std::vector<double> dyn_d_f = {pd_f, vd_f, ad_f};

    coeff_s = jerkMinimizingTrajectory(dyn.first, dyn_s_f, delta_t);
    coeff_d = jerkMinimizingTrajectory(dyn.second, dyn_d_f, delta_t);

    // For a given ps_f, the algorithm guarantees that is a valid path exists, the first
    // valid path found takes the shortest time.
    if (validatePath(coeff_s, coeff_d, delta_t)) return computeJmtPath(coeff_s, coeff_d, delta_t);
  }
}