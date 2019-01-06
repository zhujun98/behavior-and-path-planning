//
// Created by jun on 12/10/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <exception>

#include "utilities.hpp"
#include "eigen3/Eigen/Dense"
#include "spline/spline.h"

using position = std::pair<double, double>;
using polynomial_coeff = std::vector<double>;
using trajectory = std::vector<double>;


/**
 * Calculate the index of the closest waypoint.
 */
std::size_t closestWaypoint(double x, double y, const trajectory& traj_x, const trajectory& traj_y) {
  std::size_t closest_wpt = 0;
  double closest = std::numeric_limits<double>::max();
  for (std::size_t i=0; i < traj_x.size(); ++i) {
    double dist = distance(x, y, traj_x[i], traj_y[i]);
    if (dist < closest) {
      closest = dist;
      closest_wpt = i;
    }
  }

  return closest_wpt;
}

/**
 * Get the index of the next waypoint.
 */
std::size_t nextWaypoint(double x, double y, double yaw, const trajectory& traj_x, const trajectory& traj_y) {
  std::size_t closest_wpt = closestWaypoint(x, y, traj_x, traj_y);

  double closest_x = traj_x[closest_wpt];
  double closest_y = traj_y[closest_wpt];

  double closest_yaw = atan2(closest_y - y, closest_x - x);

  double angle = std::abs(yaw - closest_yaw);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    ++closest_wpt;
    if (closest_wpt == traj_x.size()) closest_wpt = 0;
  }

  return closest_wpt;
}


/**
 * Convert the Cartesian coordinate in a trajectory to the Frenet coordinate.
 */
position cartesianToFrenet(double x, double y, double yaw, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y) {

  std::size_t next_wpt = nextWaypoint(x, y, yaw, traj_x, traj_y);
  std::size_t prev_wpt;
  if (next_wpt == 0) prev_wpt = traj_x.size() - 1;
  else prev_wpt = next_wpt - 1;

  // find the projection of x onto n
  double n_x = traj_x[next_wpt] - traj_x[prev_wpt];
  double n_y = traj_y[next_wpt] - traj_y[prev_wpt];
  double x_x = x - traj_x[prev_wpt];
  double x_y = y - traj_y[prev_wpt];

  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  // calculate the Frenet coordinate d
  double d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - traj_x[prev_wpt];
  double center_y = 2000 - traj_y[prev_wpt];
  double center_to_pos = distance(center_x, center_y, x_x, x_y);
  double center_to_ref = distance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) d *= -1;

  // calculate the Frenet coordinate s
  double s = 0;
  for (std::size_t i = 0; i < prev_wpt; ++i)
    s += distance(traj_x[i], traj_y[i], traj_x[i+1], traj_y[i+1]);

  s += distance(0, 0, proj_x, proj_y);
  if (s > max_s) s -= max_s;
  else if (s < 0) s += max_s;

  return {s, d};
}

/**
 * Convert the Frenet coordinate in a trajectory to the Cartesian coordinate.
 */
position frenetToCartesian(double s, double d, const trajectory& traj_s, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y) {
  while (s > max_s) s -= max_s;
  while (s < 0) s += max_s;

  long next_wpt = std::distance(traj_s.begin(), std::lower_bound(traj_s.begin(), traj_s.end(), s));

  trajectory local_traj_s;
  trajectory local_traj_x;
  trajectory local_traj_y;
  // apply interpolation around the next waypoint
  for (int i = -5; i < 5; ++i ) {
    long wpt = next_wpt + i;

    // we either have wpt < 0 or wpt > 0
    if (wpt < 0) {
      wpt += traj_s.size();
      local_traj_s.push_back(traj_s[wpt] - max_s);
    } else if (wpt >= traj_s.size()) {
      wpt -= traj_s.size();
      local_traj_s.push_back(traj_s[wpt] + max_s);
    } else {
      local_traj_s.push_back(traj_s[wpt]);
    }
    local_traj_x.push_back(traj_x[wpt]);
    local_traj_y.push_back(traj_y[wpt]);
  }

  tk::spline spline_sx;
  spline_sx.set_points(local_traj_s, local_traj_x);

  tk::spline spline_sy;
  spline_sy.set_points(local_traj_s, local_traj_y);

  double x = spline_sx(s);
  double y = spline_sy(s);

  double dx = spline_sx(s + 0.1) - spline_sx(s - 0.1);
  double dy = spline_sy(s + 0.1) - spline_sy(s - 0.1);
  double yaw = std::atan2(dy, dx) - pi()/2;

  x += d * std::cos(yaw);
  y += d * std::sin(yaw);

  return {x, y};
}

/**
 * Find the coefficients of a quinted polynomial which minimizes the
 * jerk between the initial state and the final state in a given period,
 * i.e., min(integral(d^3(x)/dt^3)^2))
 *
 *
 * Reference: 10.1109/ROBOT.2010.5509799
 *
 * Note:: for multi-dimensional scenario, one needs to apply this
 *        function to different directions separately.
 *
 * @param state0: initial state [x, dx/dt, d^2(x)/dt^2]
 * @param state1: final state [x, dx/dt, d^2(x)/dt^2]
 * @param duration: transition time (s) between the initial and final states
 *
 * @return a vector of coefficients (p0, p1, p2, p3, p4, p5, p6) for evaluating the polynomial
 *         s(t) = p0 + p1*t + p2*t**2 + p3*t**3 + p4*t**4 + p5*t**5
 */
polynomial_coeff
jerkMinimizingTrajectory(const std::vector<double>& state0, const std::vector<double>& state1, double dt) {
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

  Eigen::VectorXd p = a.colPivHouseholderQr().solve(b);

  return {state0[0], state0[1], state0[2]/2.0, p[0], p[1], p[2]};
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
double evalPolynomialDerivative(const polynomial_coeff& p, double t, unsigned order) {
  if (order > p.size()) throw std::invalid_argument("Invalid order!");

  double result = 0.0;
  double t_power = 1;
  for (std::size_t i = order; i < p.size(); ++i) {
    int multiplier = 1;
    for (std::size_t j = i; j > i - order; --j) multiplier *= j;
    result += multiplier * p[i] * t_power;
    t_power *= t;
  }

  return result;
}


double evalTrajectory(const polynomial_coeff& p, double t) {
  return evalPolynomialDerivative(p, t, 0);
}

double evalVelocity(const polynomial_coeff& p, double t) {
  return evalPolynomialDerivative(p, t, 1);
}

double evalAcceleration(const polynomial_coeff& p, double t) {
  return evalPolynomialDerivative(p, t, 2);
}

#endif //PATH_PLANNING_TRAJECTORY_H
