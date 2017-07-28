//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
#define PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H

#include <vector>

#include "ego_transition_state.h"

class Ego;
class EgoState;


class EgoTransitionFTToCL : public EgoTransitionState {

private:

  bool checkPostCollision(Ego& ego);

  bool checkPreCollision(const Ego& ego) const;

  bool checkSideCollision(const Ego& ego, std::vector<std::vector<double>> cars) const;

public:

  // constructor
  EgoTransitionFTToCL();

  // destructor
  ~EgoTransitionFTToCL();

  bool isValid(Ego& ego) const;

  EgoState* getNextState(Ego& ego) const;
};

#endif //PATH_PLANNING_EGO_TRANSITION_FT_TO_CL_H
