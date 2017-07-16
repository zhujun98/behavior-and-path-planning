//
// Created by jun on 7/16/17.
//

#ifndef BEHAVIOR_PLANNER_TRAFFIC_H
#define BEHAVIOR_PLANNER_TRAFFIC_H


class Traffic {
private:

  struct vehicle_ {
    double px_;
    double py_;
    double vx_;
    double vy_;

    int lane_id_;
  };

public:

  //
  // constructor
  //
  Traffic();

  //
  // destructor
  //
  virtual ~Traffic();


  void update_state();
};


#endif //BEHAVIOR_PLANNER_TRAFFIC_H
