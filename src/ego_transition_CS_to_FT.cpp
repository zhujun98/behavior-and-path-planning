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
  // find the closest front car
  double min_ds = 1000; // initialize with a large number
  double v_front = 1000; // speed of the front car
  double s_front = 1000; // location of the front car
  for ( auto &v : ego.getSurroundings()->center ) {
    double ds;
    if (ego.getPathS()->empty()) {
      ds = v[4] - ego.getPs();
    } else {
      ds = v[4] - ego.getPathS()->front();
    }

    if ( ds > 0 && ds < min_ds ) {
      min_ds = ds;
      s_front = v[4];
      v_front = v[2];
    }
  }

  // check whether the front car can be catched at the path end
  double duration = ego.getPathS()->size()*ego.getTimeStep();
  double distance_at_path_end;
  if ( ego.getPathS()->empty() ) {
    distance_at_path_end = s_front + duration * v_front - ego.getPs();
  } else {
    distance_at_path_end = s_front + duration * v_front - ego.getPathS()->back();
  }

  return ( distance_at_path_end < ego.getMinSafeDistance() );
}
