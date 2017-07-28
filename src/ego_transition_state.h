//
// Created by jun on 7/28/17.
//

#ifndef PATH_PLANNING_EGO_TRANSITION_STATE_H
#define PATH_PLANNING_EGO_TRANSITION_STATE_H


class Ego;
class EgoState;

//
// CS_TO_FT: constant speed to follow traffic
// CL_TO_FT: change lane to follow traffic
// FT_TO_CS: follow traffic to constant speed
// FT_TO_CL: follow traffic to change lane
//
enum TransitionStates { CS_TO_FT, CL_TO_FT, FT_TO_CS, FT_TO_CL };


class EgoTransitionState {
protected:

  // constructor
  EgoTransitionState();

public:

  // destructor
  virtual ~EgoTransitionState();

  virtual bool isValid(Ego& ego) const = 0;

  virtual EgoState* getNextState(Ego& ego) const = 0;
};


class EgoTransitionStateFactory {
public:
  // constructor
  EgoTransitionStateFactory();

  // destructor
  ~EgoTransitionStateFactory();

  static EgoTransitionState* createState(TransitionStates name);
};


#endif //PATH_PLANNING_EGO_TRANSITION_STATE_H
