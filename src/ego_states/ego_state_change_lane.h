//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_CHANGE_LANE_H
#define PATH_PLANNING_EGO_STATE_CHANGE_LANE_H

#include "ego_state.h"

class Ego;


class EgoStateChangeLane : public EgoState {
protected:

  void planPath(Ego& ego);

  // constructor
  EgoStateChangeLane();

public:

  // destructor
  virtual ~EgoStateChangeLane();

};


#endif //PATH_PLANNING_EGO_STATE_CHANGE_LANE_H
