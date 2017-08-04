//
// Created by jun on 7/29/17.
//

#include "ego_transition_FT_to_CLL.h"
#include "../ego.h"
#include "../ego_states/ego_state.h"


EgoTransitionFTToCLL::EgoTransitionFTToCLL() {}

EgoTransitionFTToCLL::~EgoTransitionFTToCLL() {}

EgoState* EgoTransitionFTToCLL::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CLL);
}

bool EgoTransitionFTToCLL::isValid(Ego &ego) const {
  int target_lane_id = ego.getLaneID() - 1;
  if ( !willCollision(ego, -1) && isOptimal(ego, -1) ) {
    ego.setTargetLaneID(target_lane_id);
    ego.truncatePath(15);
    planPath(ego);
    return true;
  } else {
    return false;
  }

}
