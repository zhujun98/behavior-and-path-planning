//
// Created by jun on 7/27/17.
//

#ifndef PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H
#define PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H


#include <vector>

#include "ego_state.h"


class Ego;


class EgoStateFollowTraffic : public EgoState {
private:

  void planPath(Ego& ego);

public:

  // constructor
  EgoStateFollowTraffic();

  // destructor
  ~EgoStateFollowTraffic();

  void onEnter(Ego& ego);

  void onUpdate(Ego& ego);

  void onExit(Ego& ego);
};


#endif //PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H
