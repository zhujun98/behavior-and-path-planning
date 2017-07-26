//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <assert.h>

#include "Eigen-3.3/Eigen/Dense"


#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

//
// For converting back and forth between radians and degrees.
//
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

//
//
//
template <class T>
inline void print1DContainer(T v) {
  for ( auto i : v ) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;
}

//
// Calculate the distance between two points (x1, y1) and (x2, y2)
//
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

//
// Checks if the SocketIO event has JSON data.
//
inline std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

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
inline std::vector<double>
jerkMinimizingTrajectory(const std::vector<double>& state0,
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

//
// Evaluate the position at time t using the given polynomial coefficients
// y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
//
// @param p: polynomial coefficients
// @param t: time
//
// @return: position at time t
//
inline double evalTrajectory(const std::vector<double>& p, double t) {
  double result = 0.0;
  double t_power = 1;
  for ( int i=0; i < p.size(); ++i ) {
    result += p[i]*t_power;
    t_power *= t;
  }

  return result;
}

#endif //PATH_PLANNING_UTILITIES_H
