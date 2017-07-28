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
#include "utilities.h"
#include "ego_transition_state.h"


EgoStateConstantSpeed::EgoStateConstantSpeed() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CS_TO_FT));
}

EgoStateConstantSpeed::~EgoStateConstantSpeed() {}

void EgoStateConstantSpeed::onEnter(Ego& ego) {
  ego.truncatePath(5);
  std::cout << "Enter state: *** CONSTANT SPEED *** " << ego.getTargetSpeed()*2.25
            << " MPH" << std::endl;
}

void EgoStateConstantSpeed::onUpdate(Ego &ego) {
  planPath(ego);
}

void EgoStateConstantSpeed::onExit(Ego& ego) {
  std::cout << "Exit state: *** CONSTANT SPEED ***" << std::endl;
}

void EgoStateConstantSpeed::planPath(Ego& ego) {

  auto state0_sd = getState0(ego);
  std::vector<double> state0_s = state0_sd.first;
  std::vector<double> state0_d = state0_sd.second;
  double ps0 = state0_s[0];
  double vs0 = state0_s[1];

  double ps1, vs1, as1;
  double pd1, vd1, ad1;
  double prediction_time = 2.0; // in s

  vs1 = ego.getTargetSpeed();
  vd1 = 0;
  as1 = 0;
  ad1 = 0;
  ps1 = ps0 + 0.5*(vs0 + vs1)*prediction_time;
  pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  std::vector<double> state1_s = {ps1, vs1, as1};
  std::vector<double> state1_d = {pd1, vd1, ad1};

  std::vector<double> coeff_s = jerkMinimizingTrajectory(state0_s, state1_s, prediction_time);
  std::vector<double> coeff_d = jerkMinimizingTrajectory(state0_d, state1_d, prediction_time);

  ego.extendPath(coeff_s, coeff_d, prediction_time);
}
