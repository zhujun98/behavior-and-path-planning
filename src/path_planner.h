//
// Created by jun on 7/28/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>

typedef std::pair<std::vector<double>, std::vector<double>> path_coefficients;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_state;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_trajectory;


class PathPlanner {
private:
  double time_step_;

  double max_speed_;
  double max_acceleration_;
  double max_jerk_;

  //
  // Evaluate the ith derivative of a polynomial
  // y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
  //
  // @param p: polynomial coefficients
  // @param t: time
  //
  // @return: value at time t
  //
  double evalPolynomialDeriv(const std::vector<double>& p, double t, int deriv) const;

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
  std::vector<double> jerkMinimizingTrajectory(const std::vector<double>& state0,
                                               const std::vector<double>& state1,
                                               double duration) ;
  //
  // Evaluate the position at time t using the given polynomial coefficients
  // y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
  //
  // @param p: polynomial coefficients
  // @param t: time
  //
  // @return: position at time t
  //
  double evalTrajectory(const std::vector<double>& p, double t) const;

  double evalVelocity(const std::vector<double>& p, double t) const;

  double evalAcceleration(const std::vector<double>& p, double t) const;

  double evalJerk(const std::vector<double>& p, double t) const;

  double analyzePath(const vehicle_trajectory& path, double duration);

public:

  // consturctor
  PathPlanner(double max_speed, double max_acceleration, double max_jerk);

  //
  ~PathPlanner();

  // Find the optimized path
  vehicle_trajectory plan(const vehicle_state& state0, const vehicle_state& state1, double duration);

  void setTimeStep(double value);

};


#endif //PATH_PLANNING_PATH_PLANNER_H
