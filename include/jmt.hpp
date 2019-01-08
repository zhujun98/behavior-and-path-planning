//
// Created by jun on 1/8/19.
//

#ifndef BEHAVIOR_AND_PATH_PLANNING_JMT_H
#define BEHAVIOR_AND_PATH_PLANNING_JMT_H

#include "eigen3/Eigen/Dense"

using polynomial_coeff = std::vector<double>;

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

#endif //BEHAVIOR_AND_PATH_PLANNING_JMT_H
