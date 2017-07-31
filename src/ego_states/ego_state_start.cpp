//
// Created by jun on 7/31/17.
//

#include "../vehicle.h"
#include "../ego.h"
#include "ego_state_start.h"
#include "../ego_transition_states/ego_transition_state.h"
#include "../path_planner.h"
#include "../utilities.h"


EgoStateStart::EgoStateStart() {
  transition_states_.push_back(EgoTransitionStateFactory::createState(ST_TO_FT));
}

EgoStateStart::~EgoStateStart() {}

void EgoStateStart::onEnter(Ego& ego) {
  ego.truncatePath(0);
  std::cout << "Enter state: *** START ***" << std::endl;
}

void EgoStateStart::onUpdate(Ego &ego) {
  ego.truncatePath(15);
  planPath(ego);
}

void EgoStateStart::onExit(Ego& ego) {
  std::cout << "Exit state: *** START ***" << std::endl;
}

void EgoStateStart::planPath(Ego &ego) {
  auto state0 = getState0(ego);

  double vs0 = state0.first[1];
  double as0 = state0.first[2];
  double pd0 = state0.second[0];

  double prediction_time = 1;
  double as1 = as0 + ego.getMaxJerk()*prediction_time;
  double vs1 = vs0 + 0.5*(as0 + as1)*prediction_time;
  double ds1 = 0.5*(vs0 + vs1)*prediction_time;
  double pd1 = pd0;

  PathPlanner planner(ego.getTargetSpeed(), ego.getMaxAcceleration(), ego.getMaxJerk());

  planner.setDsBoundary(0.5*ds1, 1.00*ds1);
  planner.setVsBoundary(0.5*vs1, 1.00*vs1);
  planner.setAsBoundary(0, 0);

  planner.setPdBoundary(pd1, pd1);
  planner.setVdBoundary(0, 0);
  planner.setAdBoundary(0, 0);

  vehicle_trajectory new_path = planner.plan(state0, prediction_time);

  ego.extendPath(new_path);
}
