//
// Created by jun on 7/27/17.
//
#ifndef PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H
#define PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H

#include "ego_state.h"

class Ego;


class EgoStateFollowTraffic : public EgoState {
private:

  void planPath(Ego& ego);

public:

  // constructor
  EgoStateFollowTraffic();

  // destructor
  ~EgoStateFollowTraffic() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;
};


#endif //PATH_PLANNING_EGO_STATE_FOLLOW_TRAFFIC_H