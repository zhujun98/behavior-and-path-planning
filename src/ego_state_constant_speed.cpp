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

  double ps0, vs0, as0;
  double pd0, vd0, ad0;

  double ps1, vs1, as1;
  double pd1, vd1, ad1;

  if ( ego.getPathS()->empty() ) {
    ps0 = ego.getPs();
    pd0 = ego.getPd();
  } else {
    ps0 = ego.getPathS()->back();
    pd0 = ego.getPathD()->back();
  }

  vs0 = ego.getTargetSpeed();
  vd0 = 0;
  as0 = 0;
  ad0 = 0;
  vs1 = ego.getTargetSpeed();
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
