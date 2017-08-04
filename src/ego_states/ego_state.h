//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_H
#define PATH_PLANNING_EGO_STATE_H

#include <vector>


class Ego;
class Map;
class EgoTransitionState;

//
// ST: start
// CLR: change to the right lane
// CLL: change to the left lane
// CS: constant speed
// FT: follow traffic
//
enum States { ST, CLR, CLL, CS, FT };

class EgoState {
protected:

  // constructor
  EgoState();

  // Storing the transition states which is belong to this state.
  // The order of the states matter!
  std::vector<EgoTransitionState *> transition_states_;

  double timer_;  // track how long the state lasts

public:

  // destructor
  virtual ~EgoState();

  EgoState* checkTransition(Ego& ego);

  virtual void onEnter(Ego& ego) = 0;

  virtual void onUpdate(Ego& ego) = 0;

  virtual void onExit(Ego& ego) = 0;

};


class EgoStateFactory {

public:

  EgoStateFactory();

  ~EgoStateFactory();

  static EgoState* createState(States name);
};


#endif //PATH_PLANNING_EGO_STATE_H
