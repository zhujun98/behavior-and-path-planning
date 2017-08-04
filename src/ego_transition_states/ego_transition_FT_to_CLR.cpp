//
// Created by jun on 7/29/17.
//

#include "ego_transition_FT_to_CLR.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionFTToCLR::EgoTransitionFTToCLR() {}

EgoTransitionFTToCLR::~EgoTransitionFTToCLR() {}

EgoState* EgoTransitionFTToCLR::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLR);
}

bool EgoTransitionFTToCLR::isValid(Ego &ego) const {
  int target_lane_id = ego.getLaneID() + 1;
  if ( !willCollision(ego, 1) && isOptimal(ego, 1) ) {
    ego.setTargetLaneID(target_lane_id);
    ego.truncatePath(15);
    planPath(ego);
    return true;
  } else {
    return false;
  }
}
