//
// Created by jun on 7/28/17.
//
#include <cmath>

#include "ego_transition_FT_to_CL.h"
#include "ego.h"
#include "ego_state.h"


EgoTransitionFTToCL::EgoTransitionFTToCL() {}

EgoTransitionFTToCL::~EgoTransitionFTToCL() {}

EgoState* EgoTransitionFTToCL::getNextState(Ego& ego) const {
  return EgoStateFactory::createState(CL);
}

bool EgoTransitionFTToCL::isValid(Ego &ego) const {
  return false;
}


bool EgoTransitionFTToCL::checkPostCollision(Ego &ego) {
  if (ego.getLaneID() > 1 && !checkSideCollision(ego, ego.getSurroundings()->left)) {
    ego.setTargetLaneID(ego.getLaneID() - 1);
    return true;
    // try right
  } else if (ego.getLaneID() < 3 && !checkSideCollision(ego, ego.getSurroundings()->right)) {
    ego.setTargetLaneID(ego.getLaneID() + 1);
    return true;
  }
}

bool EgoTransitionFTToCL::
checkSideCollision(const Ego& ego, std::vector<std::vector<double>> cars) const {
  double prediction_time = 1; // in s

  for ( const auto&v : cars ) {
    double dv  = v[2] - ego.getSpeed();
    double ds_before = v[4] - ego.getPs();
    double ds_after = ds_before + prediction_time*dv;
    double safe_distance = ego.getMinSafeDistance();
    std::cout << ds_before << ", " << ds_after << std::endl;
    if ( ds_before * ds_after <= 0 ) {
      return true;
    } else if ( std::abs(ds_after) < safe_distance ) {
      return true;
    }
  }

  return false;
}
