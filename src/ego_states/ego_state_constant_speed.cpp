//
// Created by jun on 7/24/17.
//
#include <iostream>
#include <vector>

#include "../map.h"
#include "../vehicle.h"
#include "../ego.h"
#include "ego_state.h"
#include "ego_state_constant_speed.h"
#include "../utilities.h"
#include "../ego_transition_states/ego_transition_state.h"


EgoStateConstantSpeed::EgoStateConstantSpeed() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(CS_TO_CLL));
  transition_states_.push_back(EgoTransitionStateFactory::createState(CS_TO_CLR));
  transition_states_.push_back(EgoTransitionStateFactory::createState(CS_TO_FT));
}

EgoStateConstantSpeed::~EgoStateConstantSpeed() {}

void EgoStateConstantSpeed::onEnter(Ego& ego) {
  ego.setTargetSpeed(ego.getMaxSpeed()*0.95);
  std::cout << "Enter state: *** CONSTANT SPEED *** " << ego.getTargetSpeed()*2.25
            << " MPH" << std::endl;
}

void EgoStateConstantSpeed::onUpdate(Ego &ego) {
  ego.truncatePath(15);
  planPath(ego);
}

void EgoStateConstantSpeed::onExit(Ego& ego) {
  std::cout << "Exit state: *** CONSTANT SPEED ***" << std::endl;
}

void EgoStateConstantSpeed::planPath(Ego& ego) {

  auto state0 = getState0(ego);
  double vs0 = state0.first[1];

  // extend the current path
  double prediction_time = 3.0 - ego.getPathS()->size()*ego.getTimeStep(); // in s

  double vs1 = ego.getTargetSpeed();
  double ds1 = 0.5*(vs0 + vs1)*prediction_time;
  double pd1 = (ego.getLaneID() - 0.5) * ego.getMap()->getLaneWidth();

  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  planner.setDsBoundary(ds1*0.8, ds1*1.0);
  planner.setVsBoundary(vs1*0.8, vs1*1.0);
  planner.setAsBoundary(0, 0);

  planner.setPdBoundary(pd1, pd1);
  planner.setVdBoundary(0, 0);
  planner.setAdBoundary(0, 0);

  vehicle_trajectory new_path = planner.plan(state0, prediction_time );

  ego.extendPath(new_path);
}
