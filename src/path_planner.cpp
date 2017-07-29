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


vehicle_trajectory
PathPlanner::plan(vehicle_state state0, vehicle_state state1, double duration) {

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0.first, state1.first, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0.second, state1.second, duration);

  double t = 0.0;
  std::vector<double> path_s;
  std::vector<double> path_d;
  while ( path_s.size() <= duration / time_step_  ) {
    path_s.push_back(evalTrajectory(coeff_s, t));
    path_d.push_back(evalTrajectory(coeff_d, t));

    t += time_step_;
  }

  return std::make_pair(path_s, path_d);
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