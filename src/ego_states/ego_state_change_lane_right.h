//
// Created by jun on 7/29/17.
//
#ifndef PATH_PLANNING_EGO_STATE_CHANGE_LANE_RIGHT_H
#define PATH_PLANNING_EGO_STATE_CHANGE_LANE_RIGHT_H

#include "ego_state_change_lane.h"


class EgoStateChangeLaneRight : public EgoStateChangeLane {
public:

  // constructor
  EgoStateChangeLaneRight();

  // destructor
  ~EgoStateChangeLaneRight() override;

  void onEnter(Ego& ego) override;

  void onExit(Ego& ego) override;

};


#endif //PATH_PLANNING_EGO_STATE_CHANGE_LANE_RIGHT_H