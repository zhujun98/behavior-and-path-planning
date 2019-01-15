#include "eigen3/Eigen/Dense"

#include "jmt.hpp"


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

double evalJerk(const polynomial_coeff& p, double t) {
  return evalPolynomialDerivative(p, t, 3);
}