//
// Created by jun on 7/28/17.
//
#include <iostream>
#include <cmath>
#include <vector>

#include "eigen3/Eigen/Dense"

#include "path_planner.h"
#include "utilities.h"
#include "parameters.h"


PathPlanner::PathPlanner(double max_speed, double max_acceleration, double max_jerk) :
    time_step_(kTIME_STEP),
    max_speed_(0.95*max_speed),
    max_acceleration_(0.95*max_acceleration),
    max_jerk_(0.95*max_jerk),
    search_steps_(100),
    goal_ds_upper_(0),
    goal_ds_lower_(0),
    goal_vs_upper_(0),
    goal_vs_lower_(0),
    goal_as_upper_(0),
    goal_as_lower_(0),
    goal_pd_upper_(0),
    goal_pd_lower_(0),
    goal_vd_upper_(0),
    goal_vd_lower_(0),
    goal_ad_upper_(0),
    goal_ad_lower_(0) {}

PathPlanner::~PathPlanner() {}

vehicle_trajectory PathPlanner::plan(const vehicle_state& state0, double duration) {

  double min_cost = kINF_D;
  std::vector<double> best_result;
  traj_coefficients best_coefficients;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distribution(0, 1);
  for ( int i = 0; i <= search_steps_; ++i) {
    // randomly search the best solution
    // TODO:: use better line search method
    double goal_ps = state0.first[0] + goal_ds_lower_ +
                     distribution(gen)*(goal_ds_upper_ - goal_ds_lower_);
    double goal_vs = goal_vs_lower_ +
                     distribution(gen)*(goal_vs_upper_ - goal_vs_lower_);
    double goal_as = goal_as_lower_ +
                     distribution(gen)*(goal_as_upper_ - goal_as_lower_);
    double goal_pd = goal_pd_lower_ +
                     distribution(gen)*(goal_pd_upper_ - goal_pd_lower_);
    double goal_vd = goal_vd_lower_ +
                     distribution(gen)*(goal_vd_upper_ - goal_vd_lower_);
    double goal_ad = goal_ad_lower_ +
                     distribution(gen)*(goal_ad_upper_ - goal_ad_lower_);

    vehicle_state state1 = {{goal_ps, goal_vs, goal_as},
                            {goal_pd, goal_vd, goal_ad}};

    std::vector<double> coeff_s = jerkMinimizingTrajectory(
        state0.first, state1.first, duration);
    std::vector<double> coeff_d = jerkMinimizingTrajectory(
        state0.second, state1.second, duration);

    // empty coefficients result in an all zero trajectory, which could
    // have the best cost.
    if ( coeff_s.empty() || coeff_d.empty() ) { continue; }

    traj_coefficients coefficients = std::make_pair(coeff_s, coeff_d);

    std::vector<double> result = analyzeTrajectory(coefficients, duration);

    if ( result[0] < min_cost ) {
      min_cost = result[0];
      best_result = result;
      best_coefficients = coefficients;
    }
  }

  double t = 0.0;
  std::vector<double> best_path_s;
  std::vector<double> best_path_d;
  while ( best_path_s.size() <= duration / time_step_  ) {
    best_path_s.push_back(evalTrajectory(best_coefficients.first, t));
    best_path_d.push_back(evalTrajectory(best_coefficients.second, t));

    t += time_step_;
  }

//  std::cout << "cost, min speed (m/s), max speed (m/s), average speed (m/s), "
//      "max acceleration (m/s^2), max jerk (m/s^2)\n";
//  print1DContainer(best_result);

  return std::make_pair(best_path_s, best_path_d);
}

std::vector<double> PathPlanner::analyzeTrajectory(
    const traj_coefficients& coefficients, double duration) {

  double max_velocity_sqr = 0.0;
  double max_acceleration_sqr = 0.0;
  double max_jerk_sqr = 0.0;
  double min_vs = 0;

  double t = 0;
  while (t < duration) {
    double vs = evalVelocity(coefficients.first, t);
    if (vs < min_vs) { min_vs = vs; }
    double vd = evalVelocity(coefficients.second, t);
    double v_sqr = vs*vs + vd*vd;
    if (v_sqr > max_velocity_sqr) { max_velocity_sqr = v_sqr; }

    double as = evalAcceleration(coefficients.first, t);
    double ad = evalAcceleration(coefficients.second, t);
    double a_sqr = as*as + ad*ad;
    if (a_sqr > max_acceleration_sqr) { max_acceleration_sqr = a_sqr; }

    double js = evalJerk(coefficients.first, t);
    double jd = evalJerk(coefficients.second, t);
    double j_sqr = js*js + jd*jd;
    if (j_sqr > max_jerk_sqr) { max_jerk_sqr = j_sqr; }

    t += time_step_;
  }

  double cost = 0;
  std::vector<double> result(6);

  // Forbidden to go backward
  result[1] = min_vs;
  if ( min_vs < 0 ) {
    cost += 100000;
  }

  // max speed
  double max_speed = std::sqrt(max_velocity_sqr);
  result[2] = max_speed;
  if (max_speed > max_speed_) {
    cost += max_speed*10.0;
  }

  // average speed
  double distance = evalTrajectory(coefficients.first, duration) -
                    evalTrajectory(coefficients.first, 0);
  double average_speed = distance/duration;
  result[3] = average_speed;
  cost -= average_speed*5;

  // max acceleration
  double max_acceleration = std::sqrt(max_acceleration_sqr);
  result[4] = max_acceleration;
  if (max_acceleration > max_acceleration_) {
    cost += max_acceleration*10.0;
  }

  // max jerk
  double max_jerk = std::sqrt(max_jerk_sqr);
  result[5] = max_jerk;
  if (max_jerk > max_jerk_) {
    cost += max_jerk*10.0;
  }

  // total cost
  result[0] = cost;

  return result;
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

void PathPlanner::setDsBoundary(double lower, double upper) {
  if ( upper < lower ) { upper = lower; }
  goal_ds_lower_ = lower;
  goal_ds_upper_ = upper;
}

void PathPlanner::setVsBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_vs_lower_ = lower;
  goal_vs_upper_ = upper;
}

void PathPlanner::setAsBoundary(double lower, double upper) {
  if ( lower > max_acceleration_ ) { lower = max_acceleration_; }
  if ( upper > max_acceleration_ ) { upper = max_acceleration_; }
  if ( upper < lower ) { upper = lower; }
  goal_as_lower_ = lower;
  goal_as_upper_ = upper;
}

void PathPlanner::setPdBoundary(double lower, double upper) {
  if ( upper < lower ) { upper = lower; }
  goal_pd_lower_ = lower;
  goal_pd_upper_ = upper;
}

void PathPlanner::setVdBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_vd_lower_ = lower;
  goal_vd_upper_ = upper;
}

void PathPlanner::setAdBoundary(double lower, double upper) {
  if ( lower > max_speed_ ) { lower = max_speed_; }
  if ( upper > max_speed_ ) { upper = max_speed_; }
  if ( upper < lower ) { upper = lower; }
  goal_ad_lower_ = lower;
  goal_ad_upper_ = upper;
}