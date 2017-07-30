//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>
#include <assert.h>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

#include "path_planner.h"


PathPlanner::PathPlanner(double max_speed, double max_acceleration, double max_jerk) {
  time_step_ = 0.02; // in s

  max_speed_ = max_speed;
  max_acceleration_ = max_acceleration;
  max_jerk_ = max_jerk;
}

PathPlanner::~PathPlanner() {}


double PathPlanner::analyzePath(const path_coefficients& coefficients, double duration) {
  double cost = 0;

  double t = 0;
  double max_velocity_sqr = 0;
  double ave_velocity_sqr = 0;
  double max_acceleration_sqr = 0;
  double ave_acceleration_sqr = 0;
  double max_jerk_sqr = 0;
  double ave_jerk_sqr = 0;
  double count = 0;
  while ( t < duration ) {
    double vs = evalVelocity(coefficients.first, t);
    double vd = evalVelocity(coefficients.second, t);
    double v_sqr = vs*vs + vd*vd;
    ave_velocity_sqr += v_sqr;
    if ( v_sqr > max_velocity_sqr ) { max_velocity_sqr = v_sqr; }

    double as = evalAcceleration(coefficients.first, t);
    double ad = evalAcceleration(coefficients.second, t);
    double a_sqr = as*as + ad*ad;
    ave_acceleration_sqr += a_sqr;
    if ( a_sqr > max_acceleration_sqr ) { max_acceleration_sqr = a_sqr; }

    double js = evalJerk(coefficients.first, t);
    double jd = evalJerk(coefficients.second, t);
    double j_sqr = js*js + jd*jd;
    ave_jerk_sqr += j_sqr;
    if ( j_sqr > max_jerk_sqr ) { max_jerk_sqr = j_sqr; }

    t += time_step_;
    ++count;
  }

  double max_speed = std::sqrt(max_velocity_sqr);
  double ave_speed = std::sqrt(ave_velocity_sqr / count);
  if ( max_speed > 0.95*max_speed_ ) {
    cost += max_speed*10;
  } else {
    cost -= ave_speed;
  }

  double max_acceleration = std::sqrt(max_acceleration_sqr);
  double ave_acceleration = std::sqrt(ave_acceleration_sqr / count);
  if ( max_acceleration > 0.95*max_acceleration_ ) {
    cost += max_acceleration*10;
  } else {
    cost += ave_acceleration;
  }

  double max_jerk = std::sqrt(max_jerk_sqr);
  double ave_jerk = std::sqrt(ave_jerk_sqr / count);
  if ( max_jerk > 0.95*max_jerk_ ) {
    cost += max_jerk*10;
  } else {
    cost += ave_jerk;
  }

  std::cout << "cost: " << cost
            << "max speed: " << max_speed
            << "max acceleration: " << max_acceleration
            << "max jerk: " << max_jerk << std::endl;

  return cost;
}

vehicle_trajectory PathPlanner::plan(const vehicle_state& state0,
                                     const vehicle_state& state1,
                                     double duration) {
  double min_cost = 1000;
  path_coefficients best_coefficients;

  int count = 0;
  vehicle_state state1_jittered = state1;

  while ( count < 10 ) {
    state1_jittered.first[0] = state1.first[0] + 0.5*(count - 5);
    std::vector<double> coeff_s = jerkMinimizingTrajectory(
        state0.first, state1_jittered.first, duration);
    std::vector<double> coeff_d = jerkMinimizingTrajectory(
        state0.second, state1_jittered.second, duration);

    path_coefficients coefficients = std::make_pair(coeff_s, coeff_d);

    double cost = analyzePath(coefficients, duration);

    if ( cost < min_cost ) {
      min_cost = cost;
      best_coefficients = coefficients;
    }
    ++count;
  }

  double t = 0.0;
  std::vector<double> best_path_s;
  std::vector<double> best_path_d;
  while ( best_path_s.size() <= duration / time_step_  ) {
    best_path_s.push_back(evalTrajectory(best_coefficients.first, t));
    best_path_d.push_back(evalTrajectory(best_coefficients.second, t));

    t += time_step_;
  }

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

void PathPlanner::setTimeStep(double value) { time_step_ = value; }