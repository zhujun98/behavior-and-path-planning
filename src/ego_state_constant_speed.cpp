//
// Created by jun on 7/24/17.
//
#include <iostream>
#include <vector>

#include "map.h"
#include "vehicle.h"
#include "ego.h"
#include "ego_state.h"
#include "ego_state_constant_speed.h"
#include "ego_state_prepare_change_lane.h"
#include "utilities.h"


EgoStateConstantSpeed::EgoStateConstantSpeed() {
  target_speed_ = 0;
}
EgoStateConstantSpeed::EgoStateConstantSpeed(double speed) {
  target_speed_ = speed;
}

EgoStateConstantSpeed::~EgoStateConstantSpeed() {}

void EgoStateConstantSpeed::onEnter(Ego& ego) {
  if ( target_speed_ > ego.getMaxSpeed() || target_speed_ <= 0) {
    target_speed_ = ego.getMaxSpeed();
  }

  std::cout << "Enter state: *** CONSTANT SPEED *** " << target_speed_*2.25
            << " MPH" << std::endl;
}

EgoState* EgoStateConstantSpeed::onUpdate(Ego& ego) {
  if ( checkCollision(ego) ) {
    return new EgoStatePrepareChangeLane();
  } else {
    planPath(ego);
    return nullptr;
  }
}

void EgoStateConstantSpeed::onExit(Ego& ego) {
  std::cout << "Exit state: *** CONSTANT SPEED ***" << std::endl;
}

void EgoStateConstantSpeed::planPath(Ego& ego) {

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego.getPath().first.empty() ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
  } else {
    ps0 = ego.getPath().first.back();
    pd0 = ego.getPath().second.back();
  }

  vs0 = target_speed_;
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = target_speed_;
  vd1 = 0;
  as1 = 0;
  ad1 = 0;

  double duration = ego.getTimeStep() * ego.getPredictionPts();
  ps1 = ps0 + 0.5*(vs0 + vs1)*duration;
  pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state0_s = {ps0, vs0, as0};
  std::vector<double> state0_d = {pd0, vd0, ad0};
  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, duration);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, duration);

  ego.extendPath(coeff_s, coeff_d);
}

bool EgoStateConstantSpeed::checkCollision(const Ego& ego) {

  double prediction_time = 1; // in s
  double min_ds = 1000; // initialize with a large number
  double dv = 0; // speed difference

  // we only check the collision with the front car
  for ( auto &v : ego.getSurroundings()->center ) {
    double ds = v[4] - ego.getPs();
    if ( ds > 0 && ds < min_ds ) {
      min_ds = ds;
      dv = ego.getSpeed() - v[2];
    }
  }

  double safe_distance = ego.getMinSafeDistance();
  if ( dv > 0 ) { safe_distance += prediction_time*dv; }

  return ( min_ds <= safe_distance );
}
