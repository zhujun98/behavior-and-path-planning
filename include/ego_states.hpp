//
// Created by jun on 7/24/17.
//
#ifndef PATH_PLANNING_EGO_STATES_H
#define PATH_PLANNING_EGO_STATES_H

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
  EgoState();

  // Storing the transition states which is belong to this state.
  // The order of the states matter!
  std::vector<EgoTransitionState *> transition_states_;

  double timer_;  // track how long the state lasts

public:

  virtual ~EgoState();

  // Check the validity of the transition states one-by-one. If valid.
  // Switch to the next state.
  EgoState* checkTransition(Ego& ego);

  // action when entering a state
  virtual void onEnter(Ego& ego) = 0;

  // action when updating a state
  virtual void onUpdate(Ego& ego) = 0;

  // action when exiting a state
  virtual void onExit(Ego& ego) = 0;

};


class EgoStateFactory {

public:

  EgoStateFactory();

  ~EgoStateFactory();

  // construct a new state
  static EgoState* createState(States name);
};


class EgoStateFollowTraffic : public EgoState {
private:

  void planPath(Ego& ego);

public:

  EgoStateFollowTraffic();

  ~EgoStateFollowTraffic() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;
};


class EgoStateChangeLaneLeft : public EgoState {
public:

  EgoStateChangeLaneLeft();

  ~EgoStateChangeLaneLeft() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;
};


class EgoStateChangeLaneRight : public EgoState {
public:

  EgoStateChangeLaneRight();

  ~EgoStateChangeLaneRight() override;

  void onEnter(Ego& ego) override;

  void onUpdate(Ego& ego) override;

  void onExit(Ego& ego) override;

};


#endif //PATH_PLANNING_EGO_STATES_H