//
// Created by jun on 7/28/17.
//
#include <algorithm>

#include "ego_transition_FT_to_CS.h"
#include "ego.h"
#include "ego_state.h"


EgoTransitionFTToCS::EgoTransitionFTToCS() {}

EgoTransitionFTToCS::~EgoTransitionFTToCS() {}

EgoState* EgoTransitionFTToCS::getNextState(Ego& ego) const {
  ego.setTargetSpeed(ego.getMaxSpeed());

  return EgoStateFactory::createState(CS);
}

bool EgoTransitionFTToCS::isValid(Ego &ego) const {
  double average_speed = ego.getVs();

  auto path = ego.getPathS();
  if ( !path->empty() ) {
    average_speed = (path->end() - path->begin()) / path->size() ;
  }
  return ( average_speed >= 0.98 * ego.getMaxSpeed() );
}
