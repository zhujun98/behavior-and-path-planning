//
// Created by jun on 12/10/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <exception>

#include "utilities.hpp"
#include "eigen3/Eigen/Dense"
#include "spline/spline.h"

using trajectory = std::pair<std::vector<double>, std::vector<double>>;
using position = std::pair<double, double>;


/**
 * Calculate the index of the closest waypoint.
 */
std::size_t closestWaypoint(const position& p, const trajectory& traj) {
  std::size_t closest_waypoint = 0;
  double closest = std::numeric_limits<double>::max();
  for (std::size_t i=0; i < traj.first.size(); ++i) {
    double dist = distance(p.first, p.second, traj.first[i], traj.second[i]);
    if (dist < closest) {
      closest = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

/**
 * Get the index of the next waypoint.
 */
std::size_t nextWaypoint(const position& p, double yaw, const trajectory& traj) {
  std::size_t closest_wpt = closestWaypoint(p, traj);

  double closest_x = traj.first[closest_wpt];
  double closest_y = traj.second[closest_wpt];

  double closest_yaw = atan2(closest_y - p.second, closest_x - p.first);

  double angle = std::abs(yaw - closest_yaw);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    ++closest_wpt;
    if (closest_wpt == traj.first.size()) closest_wpt = 0;
  }

  return closest_wpt;
}


/**
 * Convert the Cartesian coordinate in a trajectory to the Frenet coordinate.
 */
position cartesianToFrenet(const position& p, double yaw, const trajectory& traj) {

  std::size_t next_wpt = nextWaypoint(p, yaw, traj);
  std::size_t prev_wpt;
  if (next_wpt == 0) prev_wpt = traj.first.size() - 1;
  else prev_wpt = next_wpt - 1;

  // find the projection of x onto n

  double n_x = traj.first[next_wpt] - traj.first[prev_wpt];
  double n_y = traj.second[next_wpt] - traj.second[prev_wpt];
  double x_x = p.first - traj.first[prev_wpt];
  double x_y = p.second - traj.second[prev_wpt];

  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  // calculate the Frenet coordinate d

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - traj.first[prev_wpt];
  double center_y = 2000 - traj.second[prev_wpt];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) frenet_d *= -1;

  // calculate the Frenet coordinate s
  double frenet_s = 0;
  for (std::size_t i = 0; i < next_wpt; ++i)
    frenet_s += distance(traj.first[i], traj.second[i], traj.first[i+1], traj.second[i+1]);

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

/**
 * Convert the Frenet coordinate in a trajectory to the Cartesian coordinate.
 */
position frenetToCartesian(const position& p, const trajectory& traj, const trajectory& traj_cts) {
  int prev_wp = -1;
  while (p.first > traj.first[prev_wp+1] && (prev_wp < traj.first.size() - 1))
    ++prev_wp;

  int wp2 = (prev_wp + 1) % traj.first.size();

  double heading = atan2((traj.second[wp2] - traj.second[prev_wp]), (traj.first[wp2] - traj.first[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (p.first - traj.first[prev_wp]);

  double seg_x = traj.first[prev_wp] + seg_s * std::cos(heading);
  double seg_y = traj.first[prev_wp] + seg_s * std::sin(heading);

  double perp_heading = heading-pi()/2;

  double px = seg_x + p.second * std::cos(perp_heading);
  double py = seg_y + p.second * std::sin(perp_heading);

  return {px, py};
}

/**
 * Find the coefficients of a quinted polynomial which minimizes the
 * jerk between the initial state and the final state in a given period.
 * Note:: for multi-dimensional scenario, one needs to apply this
 *        function to different directions separately.
 *
 * @param state0: initial state [x, dx/dt, d^2(x)/dt^2]
 * @param state1: final state [x, dx/dt, d^2(x)/dt^2]
 * @param duration: transition time (s) between the initial and final states
 *
 * @return
 */
std::vector<double>
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
       6*dt, 12*dt2, 20*dt3;

  Eigen::Vector3d b;
  b << state1[0] - (state0[0] + dt*state0[1] + 0.5*dt2*state0[2]),
       state1[1] - (state0[1] + dt*state0[2]),
       state1[2] -  state0[2];

  Eigen::VectorXd solution = a.colPivHouseholderQr().solve(b);

  return {solution[0], solution[1], solution[2]};
}

/**
 * Evaluate the ith derivative of a polynomial
 * y = p[0] + p[1]*x + p[2]*x + ... + p[n-1]*x^(n-1)
 *
 * @param p: polynomial coefficients
 * @param x: variable
 * @param order: order of derivative
 *
 * @return: the derivative
 */
double evalPolynomialDerivative(const std::vector<double>& p, double t, unsigned order) {
  if (order > p.size()) throw std::invalid_argument("Invalid order!");

  double result = 0.0;
  double t_power = 1;
  for ( auto i = order; i < p.size(); ++i ) {
    int multiplier = 1;
    for (int j = i; j > i - order; --j) multiplier *= j;
    result += multiplier * p[i] * t_power;
    t_power *= t;
  }

  return result;
}


double evalTrajectory(const std::vector<double>& p, double t) {
  return evalPolynomialDerivative(p, t, 0);
}

double evalVelocity(const std::vector<double>& p, double t) {
  return evalPolynomialDerivative(p, t, 1);
}

double evalAcceleration(const std::vector<double>& p, double t) {
  return evalPolynomialDerivative(p, t, 2);
}

double evalJerk(const std::vector<double>& p, double t) {
  return evalPolynomialDerivative(p, t, 3);
}

#endif //PATH_PLANNING_TRAJECTORY_H
