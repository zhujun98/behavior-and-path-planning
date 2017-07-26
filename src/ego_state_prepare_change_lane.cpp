//
// Created by jun on 7/25/17.
//
#include <iostream>

#include "utilities.h"
#include "map.h"
#include "vehicle.h"
#include "ego.h"
#include "ego_state.h"
#include "ego_state_change_lane.h"
#include "ego_state_prepare_change_lane.h"


EgoStatePrepareChangeLane::EgoStatePrepareChangeLane() {}

EgoStatePrepareChangeLane::~EgoStatePrepareChangeLane() {}

void EgoStatePrepareChangeLane::onEnter(Ego& ego) {
  std::cout << "Enter state: *** PREPARE CHANGE LANE ***" << std::endl;
}

EgoState* EgoStatePrepareChangeLane::onUpdate(Ego& ego, const Map& map) {
  int current_lane_id = ego.getLaneID();
  target_lane_id_ = current_lane_id;
  if ( checkSideCollision(ego, map, true) ) {
    target_lane_id_ = current_lane_id - 1;
  } else if ( checkSideCollision(ego, map, false) ) {
    target_lane_id_ = current_lane_id + 1;
  }

  if ( target_lane_id_ != current_lane_id ) {
    ego.truncatePath(5);
    planPath(ego, map);
    return new EgoStateChangeLane(target_lane_id_);
  } else {
    followAhead(ego, map);
    return nullptr;
  }
}

void EgoStatePrepareChangeLane::onExit(Ego& ego) {
  std::cout << "Exist state: *** PREPARE CHANGE LANE ***" << std::endl;
}

void EgoStatePrepareChangeLane::planPath(Ego& ego, const Map& map) {

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego.getPath().first.empty() ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
  } else {
    ps0 = *std::next(ego.getPath().first.end(), -1);
    pd0 = *std::next(ego.getPath().second.end(), -1);
  }

  vs0 = ego.getMaxSpeed(); //std::sqrt(ego.getVx()*ego.getVx() + ego.getVy()*ego.getVy());
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = ego.getMaxSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  double duration = ego.getTimeStep() * ego.getPredictionPts();

  ps1 = ps0 + 0.5*(vs0 + vs1)*duration;
  pd1 = (target_lane_id_ - 0.5) * map.getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}

void EgoStatePrepareChangeLane::followAhead(Ego& ego, const Map& map) {
  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego.getPath().first.empty() ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
  } else {
    ps0 = *std::next(ego.getPath().first.end(), -1);
    pd0 = *std::next(ego.getPath().second.end(), -1);
  }

  vs0 = ego.getMaxSpeed();
  double min_ds = 1000;
  for ( auto &v : ego.getSurroundings()->center ) {
    double ds = v[4] - ego.getPs();
    if ( ds > 0 && ds < min_ds ) {
      min_ds = ds;
      vs0 = std::sqrt(v[2]*v[2] + v[3]*v[3]);
    }
  }

  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = vs0;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = ego.getTimeStep() * ego.getPredictionPts();
  ps1 = ps0 + 0.5*(vs0 + vs1)*duration;
  pd1 = (ego.getLaneID() - 0.5) * map.getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}

bool EgoStatePrepareChangeLane::checkSideCollision(const Ego& ego, const Map&map, bool left) {

  int current_lane_id = ego.getLaneID();
  std::vector<std::vector<double>> cars_in_lane;
  if ( left ) {
    if ( current_lane_id <= 1 ) { return false; }
    cars_in_lane = ego.getSurroundings()->left;

  } else {
    if ( current_lane_id >= 3 ) { return false; }
    cars_in_lane = ego.getSurroundings()->right;
  }

  bool is_safe = true;
  for ( const auto&v : cars_in_lane ) {
    double ds = v[4] - ego.getPs();
    double speed = std::sqrt(v[2] * v[2] + v[3] * v[3]);

    if (ds >= 0 && ds < ego.getMaxSpeed() * ego.getSafeDsInSeconds()) {
      is_safe = false;
      break;
    } else if (ds < 0 && -1.0 * ds < speed * ego.getSafeDsInSeconds()) {
      is_safe = false;
      break;
    }
  }

  return is_safe;

}
