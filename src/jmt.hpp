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
jerkMinimizingTrajectory(const std::vector<double>& state0, const std::vector<double>& state1, double dt);

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
double evalPolynomialDerivative(const polynomial_coeff& p, double t, unsigned order);

double evalTrajectory(const polynomial_coeff& p, double t);

double evalVelocity(const polynomial_coeff& p, double t);

double evalAcceleration(const polynomial_coeff& p, double t);

double evalJerk(const polynomial_coeff& p, double t);

#endif //BEHAVIOR_AND_PATH_PLANNING_JMT_H

