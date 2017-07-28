//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_CHANGE_LANE_H
#define PATH_PLANNING_EGO_STATE_CHANGE_LANE_H

#include "ego_state.h"

class Ego;


class EgoStateChangeLane : public EgoState {
private:

  void planPath(Ego& ego);

public:

  // constructor
  EgoStateChangeLane();

  // destructor
  ~EgoStateChangeLane();

  void onEnter(Ego& ego);

  void onUpdate(Ego& ego);

  void onExit(Ego& ego);
};


#endif //PATH_PLANNING_EGO_STATE_CHANGE_LANE_H
