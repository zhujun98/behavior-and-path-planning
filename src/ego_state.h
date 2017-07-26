//
// Created by jun on 7/24/17.
//

#ifndef PATH_PLANNING_EGO_STATE_H
#define PATH_PLANNING_EGO_STATE_H

#include <vector>

class Ego;
class Map;


class EgoState {
protected:

  int lane_id_;

  // constructor
  EgoState();

  virtual void planPath(Ego& ego, const Map& map) = 0;

public:

  // destructor
  virtual ~EgoState();

  virtual void onEnter(Ego& ego) = 0;

  virtual EgoState* onUpdate(Ego& ego, const Map& map) = 0;

  virtual void onExit(Ego& ego) = 0;

};


#endif //PATH_PLANNING_EGO_STATE_H
