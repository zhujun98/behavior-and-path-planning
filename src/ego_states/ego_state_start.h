//
// Created by jun on 7/31/17.
//

#ifndef PATH_PLANNING_EGO_STATE_START_H
#define PATH_PLANNING_EGO_STATE_START_H


#include "ego_state.h"

class Ego;


class EgoStateStart : public EgoState {
protected:

  void planPath(Ego& ego);

public:
  // constructor
  EgoStateStart();

  // destructor
  ~EgoStateStart() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;

};


#endif //PATH_PLANNING_EGO_STATE_START_H
