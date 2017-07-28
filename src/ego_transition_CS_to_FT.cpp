//
// Created by jun on 7/28/17.
//

#include "ego_transition_CS_to_FT.h"
#include "ego.h"
#include "ego_state.h"


EgoTransitionCSToFT::EgoTransitionCSToFT() {}

EgoTransitionCSToFT::~EgoTransitionCSToFT() {}

EgoState* EgoTransitionCSToFT::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(FT);
}

bool EgoTransitionCSToFT::isValid(Ego &ego) const {
  return ( ego.getSpeed() < 0.95*ego.getTargetSpeed() );
}

bool EgoTransitionCSToFT::checkPreCollision(const Ego& ego) const {
  double prediction_time = 1; // in s
  double min_ds = 1000; // initialize with a large number
  double dv = 0; // speed difference

  // we only check the collision with the front car
  for ( auto &v : ego.getSurroundings()->center ) {
    double ds = v[4] - ego.getPathS()->back();
    if ( ds > 0 && ds < min_ds ) {
      min_ds = ds;
      dv = ego.getSpeed() - v[2];
    }
  }

  double safe_distance = ego.getMinSafeDistance();
  if ( dv > 0 ) { safe_distance += prediction_time*dv; }

  return ( min_ds <= safe_distance );
}