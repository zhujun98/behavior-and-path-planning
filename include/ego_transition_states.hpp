//
// Created by jun on 7/28/17.
//
#ifndef PATH_PLANNING_EGO_TRANSITION_STATES_H
#define PATH_PLANNING_EGO_TRANSITION_STATES_H

class Ego;
class EgoState;


//
// CL_TO_FT: change lane to follow traffic
// FT_TO_CLR: follow traffic to change lane right
// FT_TO_CLL: follow traffic to change lane left
//
enum TransitionStates { CL_TO_FT, FT_TO_CLR, FT_TO_CLL };


class EgoTransitionState {
protected:

  EgoTransitionState() = default;

public:

  virtual ~EgoTransitionState() = default;

  // check whether the transition is valid.
  virtual bool isValid(Ego& ego) const = 0;

  // get the next vehicle state
  virtual EgoState* getNextState(Ego& ego) const = 0;
};


class EgoTransitionStateFactory {
public:
  EgoTransitionStateFactory() = default;

  ~EgoTransitionStateFactory() = default;

  // construct a new transition state
  static EgoTransitionState* createState(TransitionStates name);
};


class EgoTransitionCLToFT : public EgoTransitionState {
public:

  EgoTransitionCLToFT();

  ~EgoTransitionCLToFT() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


class EgoTransitionFTToCL : public EgoTransitionState {

protected:
  EgoTransitionFTToCL();

  //
  // check the whether the direction of lane change is optimal
  //
  // @param direction: +1 for to the right and -1 for to the left
  //
  bool isOptimal(const Ego& ego, int direction) const;

  //
  // Plan the path for the lane change. If a feasible path is found,
  // return true and set the path for the ego car. If not, return
  // false.
  //
  // @param direction: +1 for to the right and -1 for to the left
  //
  bool planPath(Ego &ego, int direction) const;

public:

  ~EgoTransitionFTToCL() override;
};


class EgoTransitionFTToCLL : public EgoTransitionFTToCL {
public:

  EgoTransitionFTToCLL();

  ~EgoTransitionFTToCLL() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


class EgoTransitionFTToCLR : public EgoTransitionFTToCL {
public:

  EgoTransitionFTToCLR();

  ~EgoTransitionFTToCLR() override;

  bool isValid(Ego& ego) const override;

  EgoState* getNextState(Ego& ego) const override;
};


#endif //PATH_PLANNING_EGO_TRANSITION_STATES_H