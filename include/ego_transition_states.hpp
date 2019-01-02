//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_STATES_H
#define PATH_PLANNING_EGO_TRANSITION_STATES_H

class Ego;
class EgoState;


//
// CL_TO_FT: change lane to follow traffic
// FT_TO_CLL: follow traffic to change lane left
// FT_TO_CLR: follow traffic to change lane right
//
enum TransitionStates { CL_TO_FT, FT_TO_CLL, FT_TO_CLR };


class EgoTransitionState {
protected:

  EgoTransitionState();

public:

  virtual ~EgoTransitionState();

  // check whether the transition is valid.
  virtual bool isValid(Ego& ego) const = 0;

  // get the next vehicle state
  virtual EgoState* getNextState(Ego& ego) const = 0;
};


class EgoTransitionStateFactory {
public:
  EgoTransitionStateFactory();

  ~EgoTransitionStateFactory();

  // construct a new transition state
  static EgoTransitionState* createState(TransitionStates name);
};


class EgoTransitionCL2FT : public EgoTransitionState {
public:

  EgoTransitionCL2FT();

  ~EgoTransitionCL2FT() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


class EgoTransitionFT2CLL : public EgoTransitionState {
public:

  EgoTransitionFT2CLL();

  ~EgoTransitionFT2CLL() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


class EgoTransitionFT2CLR : public EgoTransitionState {
public:

  EgoTransitionFT2CLR();

  ~EgoTransitionFT2CLR() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


#endif //PATH_PLANNING_EGO_TRANSITION_STATES_H