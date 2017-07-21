//
// Created by jun on 7/21/17.
//
#include <assert.h>

#include "Eigen-3.3/Eigen/Dense"

#include "path_planner.h"


PathPlanner::PathPlanner() {}

PathPlanner::~PathPlanner() {}

//
// Find the coefficients of a quinted polynomial which minimizes the
// jerk between the initial state and the final state in a given period.
// Note:: for multi-dimensional scenario, one needs to apply this
//        function to different directions separately.
//
// @param state0: initial state [x, dx/dt, d^2(x)/dt^2]
// @param state1: final state [x, dx/dt, d^2(x)/dt^2]
// @param dt: transition time (s) between the initial and final states
//
std::vector<double>
PathPlanner::jerk_minimizing_trajectory(std::vector<double> state0,
                                        std::vector<double> state1,
                                        double dt) const {
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

double PathPlanner::eval_trajectory(std::vector<double> coeff, double t) const {
  double result = 0.0;
  double t_power = 1;
  for ( int i=0; i < coeff.size(); ++i ) {
    result += coeff[i]*t_power;
    t_power *= t;
  }

  return result;
}
