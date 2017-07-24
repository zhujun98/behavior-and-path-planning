//
// Created by jun on 7/21/17.
//
#include <assert.h>

#include "Eigen-3.3/Eigen/Dense"

#include "path_planner.h"
#include "vehicle.h"


PathPlanner::PathPlanner(Ego& ego, const Map& map) {
  ego_ = &ego;
  map_ = &map;

  max_prediction_points_ = 200;
  time_step_ = 0.02;
}

PathPlanner::~PathPlanner() {}

std::vector<double>
PathPlanner::jerkMinimizingTrajectory(std::vector<double> state0,
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

double PathPlanner::evalTrajectory(std::vector<double> p, double t) const {
  double result = 0.0;
  double t_power = 1;
  for ( int i=0; i < p.size(); ++i ) {
    result += p[i]*t_power;
    t_power *= t;
  }

  return result;
}

void PathPlanner::plan() {
  switch (ego_->getBehavior()) {
    case KL:
      keepLane();
      break;
    case LC:
      break;
    case PLC:
      prepareLaneChange();
      ego_->setBehavior(LC);
      break;
  }
}

void PathPlanner::extendPath(std::vector<double> coeff_s,
                              std::vector<double> coeff_d) {

  double t = 0.0;
  while ( ego_->path_s_.size() < max_prediction_points_ ) {
    t += time_step_;
    ego_->path_s_.push_back(evalTrajectory(coeff_s, t));
    ego_->path_d_.push_back(evalTrajectory(coeff_d, t));
  }
}

void PathPlanner::keepLane() {

  double last_s;
  double last_d;

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego_->path_s_.empty() ) {
    ps0 = ego_->ps_;
    pd0 = ego_->pd_;
  } else {
    ps0 = *std::next(ego_->path_s_.end(), -1);
    pd0 = *std::next(ego_->path_d_.end(), -1);
  }

  vs0 = ego_->getMaxSpeed();
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = ego_->getMaxSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = time_step_* max_prediction_points_;
  ps1 = ps0 + ego_->getMaxSpeed()*duration;
  pd1 = (ego_->getLaneID() - 0.5)*4.0;

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  extendPath(coeff_s, coeff_d);

}

void PathPlanner::prepareLaneChange() {
  double time_left = ego_->getLaneChangeTimer();

  while ( ego_->path_s_.size() > 5 ) {
    ego_->path_s_.pop_back();
    ego_->path_d_.pop_back();
  }

  double last_s;
  double last_d;

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego_->path_s_.empty() ) {
    ps0 = ego_->ps_;
    pd0 = ego_->pd_;
  } else {
    ps0 = *std::next(ego_->path_s_.end(), -1);
    pd0 = *std::next(ego_->path_d_.end(), -1);
  }

  vs0 = ego_->getMaxSpeed();
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = ego_->getMaxSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double duration = time_step_* max_prediction_points_;

  ps1 = ps0 + vs0*time_left;
  pd1 = (ego_->getTargetLaneID() - 0.5)*4.0;

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  extendPath(coeff_s, coeff_d);
}
