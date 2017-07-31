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
  virtual ~EgoStateStart();

  void onEnter(Ego& ego);

  void onUpdate(Ego& ego);

  void onExit(Ego& ego);

};


#endif //PATH_PLANNING_EGO_STATE_START_H
