//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>


typedef std::pair<std::vector<double>, std::vector<double>> traj_coefficients;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_state;
typedef std::pair<std::vector<double>, std::vector<double>> vehicle_trajectory;


class PathPlanner {
private:
  double time_step_; // in t

  double max_speed_; // in m/s
  double max_acceleration_; // in m/s^2
  double max_jerk_; // in m/s^3

  double goal_ds_upper_;
  double goal_ds_lower_;

  double goal_vs_upper_;
  double goal_vs_lower_;

  double goal_as_upper_;
  double goal_as_lower_;

  double goal_pd_upper_;
  double goal_pd_lower_;

  double goal_vd_upper_;
  double goal_vd_lower_;

  double goal_ad_upper_;
  double goal_ad_lower_;

  int search_steps_;

  /**
   * Evaluate the ith derivative of a polynomial
   * y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
   *
   * @param p: polynomial coefficients
   * @param t: time
   * @param deriv: order of derivative
   * @return: value at time t
   */
  double evalPolynomialDeriv(const std::vector<double>& p, double t, int deriv) const;

  /**
   * Find the coefficients of a quinted polynomial which minimizes the
   * jerk between the initial state and the final state in a given period.
   * Note:: for multi-dimensional scenario, one needs to apply this
   *        function to different directions separately.
   *
   * @param state0: initial state [x, dx/dt, d^2(x)/dt^2]
   * @param state1: final state [x, dx/dt, d^2(x)/dt^2]
   * @param duration: transition time (s) between the initial and final states
   * @return
   */
  std::vector<double> jerkMinimizingTrajectory(const std::vector<double>& state0,
                                               const std::vector<double>& state1,
                                               double duration) ;

  /**
   * Evaluate the position at time t using the given polynomial coefficients
   * y = p[0] + p[1]*t + p[2]*t + ... + p[n-1]*t^(n-1)
   *
   * @param p: polynomial coefficients
   * @param t: time
   * @return: position at time t
   */
  double evalTrajectory(const std::vector<double>& p, double t) const;

  // Evaluate the velocity at time t using the given polynomial coefficients
  double evalVelocity(const std::vector<double>& p, double t) const;

  // Evaluate the acceleration at time t using the given polynomial coefficients
  double evalAcceleration(const std::vector<double>& p, double t) const;

  // Evaluate the jerk at time t using the given polynomial coefficients
  double evalJerk(const std::vector<double>& p, double t) const;

  /**
   * calculate the cost of a path
   *
   * @param path: path
   * @param duration: duration
   * @returna vector of costs from different terms. (It is convenient for debug)
   */
  std::vector<double> analyzeTrajectory(const vehicle_trajectory& path, double duration);

public:

  PathPlanner(double max_speed, double max_acceleration, double max_jerk);

  ~PathPlanner();

  // Find the optimized path
  vehicle_trajectory plan(const vehicle_state& state0, double duration);

  void setVsBoundary(double lower, double upper);

  void setAsBoundary(double lower, double upper);

  void setDsBoundary(double lower, double upper);

  void setVdBoundary(double lower, double upper);

  void setAdBoundary(double lower, double upper);

  void setPdBoundary(double lower, double upper);
};


#endif //PATH_PLANNING_PATH_PLANNER_H