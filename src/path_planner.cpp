//
// Created by jun on 7/21/17.
//
#include <assert.h>

#include "Eigen-3.3/Eigen/Dense"

#include "path_planner.h"


PathPlanner::PathPlanner(Ego& car) {
  car_ = &car;

  n_path_points_ = 50;
  time_step_ = 0.02;
}

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

void PathPlanner::plan() {
  switch (car_->getBehavior()) {
    case KL:
      keep_lane();
      break;
    case LC:
      change_lane();
      car_->setBehavior(LCING);
      break;
    case PLC:
      break;
    case LCING:
      break;
  }
}

void PathPlanner::generate_path(std::vector<double> state0_s,
                                std::vector<double> state0_d,
                                std::vector<double> state1_s,
                                std::vector<double> state1_d,
                                double duration) {
  std::vector<double> coeff_s = jerk_minimizing_trajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerk_minimizing_trajectory(state0_d, state1_d, duration);
  double t = 0.0;
  while ( car_->path_s_.size() < n_path_points_ ) {
    t += time_step_;
    car_->path_s_.push_back(eval_trajectory(coeff_s, t));
    car_->path_d_.push_back(eval_trajectory(coeff_d, t));
  }
}

void PathPlanner::truncateLastPath() {
  // remove the way points which has been passed (processed) and keep
  // up to 10 way points which has not been reached.
  if ( !car_->path_s_.empty() ) {
    auto it_s = std::lower_bound(car_->path_s_.begin(), car_->path_s_.end(), car_->ps_);
    long j = std::distance(car_->path_s_.begin(), it_s);
    auto it_d = std::next(car_->path_d_.begin(), j);

    if ( std::distance(it_s, car_->path_s_.end()) > 20 ) {
      std::vector<double> unprocessed_path_s(it_s, std::next(it_s, 20));
      std::vector<double> unprocessed_path_d(it_d, std::next(it_d, 20));
      car_->path_s_.clear();
      car_->path_d_.clear();
      for ( auto v : unprocessed_path_s ) { car_->path_s_.push_back(v); }
      for ( auto v : unprocessed_path_d ) { car_->path_d_.push_back(v); }
    } else {
      std::vector<double> unprocessed_path_s(it_s, car_->path_s_.end());
      std::vector<double> unprocessed_path_d(it_d, car_->path_d_.end());
      car_->path_s_.clear();
      car_->path_d_.clear();
      for ( auto v : unprocessed_path_s ) { car_->path_s_.push_back(v); }
      for ( auto v : unprocessed_path_d ) { car_->path_d_.push_back(v); }
    }
  }
}

void PathPlanner::keep_lane() {
  truncateLastPath();
  double last_s;
  double last_d;

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( car_->path_s_.empty() ) {
    ps0 = car_->ps_;
    pd0 = car_->pd_;
  } else {
    ps0 = *std::next(car_->path_s_.end(), -1);
    pd0 = *std::next(car_->path_d_.end(), -1);
  }

  vs0 = car_->max_speed_;
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = car_->max_speed_;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = time_step_* n_path_points_;
  ps1 = ps0 + car_->max_speed_*duration;
  pd1 = (car_->getLaneID() - 0.5)*4.0;

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  generate_path(state0_s, state0_d, state1_s, state1_d, duration);

}

void PathPlanner::change_lane() {
  truncateLastPath();

  double last_s;
  double last_d;

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( car_->path_s_.empty() ) {
    ps0 = car_->ps_;
    pd0 = car_->pd_;
  } else {
    ps0 = *std::next(car_->path_s_.end(), -1);
    pd0 = *std::next(car_->path_d_.end(), -1);
  }

  vs0 = car_->max_speed_;
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = car_->max_speed_;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = time_step_* n_path_points_;
  ps1 = ps0 + car_->max_speed_*duration;
  pd1 = (car_->getTargetLaneID() - 0.5)*4.0;

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  generate_path(state0_s, state0_d, state1_s, state1_d, duration);
}

