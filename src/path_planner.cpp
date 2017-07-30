//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>
#include <assert.h>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

#include "path_planner.h"
#include "utilities.h"


const double INF_D = std::numeric_limits<double>::max();

PathPlanner::PathPlanner(double max_speed, double max_acceleration, double max_jerk) {
  time_step_ = 0.02; // in s

  max_speed_ = max_speed;
  max_acceleration_ = max_acceleration;
  max_jerk_ = max_jerk;

  search_steps_ = 5;

  goal_ds_upper_ = 0;
  goal_ds_lower_ = 0;

  goal_vs_upper_ = max_speed_;
  goal_vs_lower_ = max_speed_;

  goal_as_upper_ = 0;
  goal_as_lower_ = 0;

  goal_pd_upper_ = 0;
  goal_pd_lower_ = 0;

  goal_vd_upper_ = 0;
  goal_vd_lower_ = 0;

  goal_ad_upper_ = 0;
  goal_ad_lower_ = 0;
}

PathPlanner::~PathPlanner() {}

std::vector<double> PathPlanner::analyzePath(const path_coefficients& coefficients, double duration) {
  std::vector<double> costs;

  double t = 0;
  double max_velocity_sqr = 0;
  double max_as = 0;
  double max_ad = 0;
  double max_js = 0;
  double max_jd = 0;
  double min_vs = 0;

  double count = 0;
  while ( t < duration ) {
    double vs = evalVelocity(coefficients.first, t);
    if ( vs < min_vs ) { min_vs = vs; }
    double vd = evalVelocity(coefficients.second, t);
    double v_sqr = vs*vs + vd*vd;
    if ( v_sqr > max_velocity_sqr ) { max_velocity_sqr = v_sqr; }

    double abs_as = std::abs(evalAcceleration(coefficients.first, t));
    if ( abs_as > max_as ) { max_as = abs_as; }

    double abs_ad = std::abs(evalAcceleration(coefficients.second, t));
    if ( abs_ad > max_ad ) { max_ad = abs_ad; }

    double abs_js = std::abs(evalJerk(coefficients.first, t));
    if ( abs_js > max_js ) { max_js = abs_js; }

    double abs_jd = std::abs(evalJerk(coefficients.second, t));
    if ( abs_jd > max_jd ) { max_jd = abs_jd; }

    t += time_step_;
    ++count;
  }

  /*
   * Forbid to go backward
   */
  if ( min_vs < 0 ) {
    costs.push_back(1000000);
  } else {
    costs.push_back(0);
  }

  /*
   * Heavily penalize when exceeding maximum speed, acceleration and jerk
   */

  double max_speed = std::sqrt(max_velocity_sqr);
  if ( max_speed > max_speed_ ) {
    costs.push_back(max_speed*10);
  } else {
    costs.push_back(0);
  }

  if ( max_as > max_acceleration_ ) {
    costs.push_back(max_as*5);
  }  else {
    costs.push_back(0);
  }

  if ( max_ad > max_acceleration_ ) {
    costs.push_back(max_ad*5);
  }  else {
    costs.push_back(0);
  }

  if ( max_js > max_jerk_ ) {
    costs.push_back(max_js*5);
  }  else {
    costs.push_back(0);
  }

  if ( max_jd > max_jerk_ ) {
    costs.push_back(max_jd*5);
  }  else {
    costs.push_back(0);
  }

  /*
   * Awarding average speed
   */
  double distance = evalTrajectory(coefficients.first, duration) -
                    evalTrajectory(coefficients.first, 0);
  costs.push_back(-distance/duration);

  return costs;
}

vehicle_trajectory PathPlanner::plan(const vehicle_state& state0, double duration) {

  double min_cost = INF_D;
  std::vector<double> best_costs;
  path_coefficients best_coefficients;

  double ds_step = (goal_ds_upper_ - goal_ds_lower_) / search_steps_;
  double vs_step = (goal_vs_upper_ - goal_vs_lower_) / search_steps_;

  double goal_as = goal_as_lower_;
  double goal_pd = goal_pd_lower_;
  double goal_vd = goal_vd_lower_;
  double goal_ad = goal_ad_lower_;

  // optimization using brute force
  double goal_vs = goal_vs_lower_;
  for ( int i = 0; i <= search_steps_; ++i) {
    double goal_ds = goal_ds_lower_;
    for ( int j = 0; j <= search_steps_; ++j ) {
      vehicle_state state1 = {{goal_ds + state0.first[0], goal_vs, goal_as},
                              {goal_pd, goal_vd, goal_ad}};

      std::vector<double> coeff_s = jerkMinimizingTrajectory(
          state0.first, state1.first, duration);
      std::vector<double> coeff_d = jerkMinimizingTrajectory(
          state0.second, state1.second, duration);

      // empty coefficients result in an all zero trajectory, which could
      // have the best cost.
      if ( coeff_s.empty() || coeff_d.empty() ) { continue; }

      path_coefficients coefficients = std::make_pair(coeff_s, coeff_d);

      std::vector<double> costs = analyzePath(coefficients, duration);
      double total_cost = std::accumulate(costs.begin(), costs.end(), 0);
      if ( total_cost < min_cost ) {
        min_cost = total_cost;
        best_costs = costs;
        best_coefficients = coefficients;
      }
      goal_ds += ds_step;
    }
    goal_vs += vs_step;
  }
  double t = 0.0;
  std::vector<double> best_path_s;
  std::vector<double> best_path_d;
  while ( best_path_s.size() <= duration / time_step_  ) {
    best_path_s.push_back(evalTrajectory(best_coefficients.first, t));
    best_path_d.push_back(evalTrajectory(best_coefficients.second, t));

    t += time_step_;
  }

//  std::cout << "Best costs: ";
//  print1DContainer(best_costs);

  return std::make_pair(best_path_s, best_path_d);
}

std::vector<double>
PathPlanner::jerkMinimizingTrajectory(const std::vector<double>& state0,
                                      const std::vector<double>& state1,
                                      double dt) {
  assert (state0.size() == 3);
  assert (state1.size() == 3);

  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt3*dt;
  double dt5 = dt4*dt;

  Eigen::Matrix3d a;
  a <<  dt3,    dt4,    dt5,
      3*dt2,  4*dt3,  5*dt4,
      6*dt,  12*dt2, 20*dt3;

  Eigen::Vector3d b;
  b << state1[0] - (state0[0] + dt*state0[1] + 0.5*dt2*state0[2]),
      state1[1] - (state0[1] + dt*state0[2]),
      state1[2] -  state0[2];

  Eigen::VectorXd solution = a.colPivHouseholderQr().solve(b);

  std::vector<double> result(6);

  result[0] = state0[0];
  result[1] = state0[1];
  result[2] = 0.5*state0[2];
  result[3] = solution[0];
  result[4] = solution[1];
  result[5] = solution[2];

  return result;
}

double PathPlanner::evalPolynomialDeriv(const std::vector<double>& p, double t, int deriv) const {
  assert( deriv >= 0 );

  if ( deriv > p.size() ) { return 0; }

  double result = 0.0;
  double t_power = 1;
  for ( int i = deriv; i < p.size(); ++i ) {
    int multiplier = 1;
    for ( int j = i; j > i - deriv; --j ) {
      multiplier *= j;
    }
    result += multiplier*p[i]*t_power;
    t_power *= t;
  }

  return result;
}

double PathPlanner::evalTrajectory(const std::vector<double>& p, double t) const {
  return evalPolynomialDeriv(p, t, 0);
}

double PathPlanner::evalVelocity(const std::vector<double>& p, double t) const {
  return evalPolynomialDeriv(p, t, 1);
}

double PathPlanner::evalAcceleration(const std::vector<double>& p, double t) const {
  return evalPolynomialDeriv(p, t, 2);
}

double PathPlanner::evalJerk(const std::vector<double>& p, double t) const {
  return evalPolynomialDeriv(p, t, 3);
}

double PathPlanner::setDsBoundary(double lower, double upper) {
  if ( upper < lower ) { upper = lower; }
  goal_ds_lower_ = lower;
  goal_ds_upper_ = upper;
}

double PathPlanner::setVsBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_vs_lower_ = lower;
  goal_vs_upper_ = upper;
}

double PathPlanner::setAsBoundary(double lower, double upper) {
  if ( lower > max_acceleration_ ) { lower = max_acceleration_; }
  if ( upper > max_acceleration_ ) { upper = max_acceleration_; }
  if ( upper < lower ) { upper = lower; }
  goal_as_lower_ = lower;
  goal_as_upper_ = upper;
}

double PathPlanner::setPdBoundary(double lower, double upper) {
  if ( upper < lower ) { upper = lower; }
  goal_pd_lower_ = lower;
  goal_pd_upper_ = upper;
}

double PathPlanner::setVdBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_vd_lower_ = lower;
  goal_vd_upper_ = upper;
}

double PathPlanner::setAdBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_ad_lower_ = lower;
  goal_ad_upper_ = upper;
}