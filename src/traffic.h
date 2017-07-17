//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <vector>
#include <map>


class Traffic {
private:

  struct vehicle_ {
    int id;

    double px_;
    double py_;
    double vx_;
    double vy_;
    double s;
    double d;
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


  void update_state(std::map<int, std::vector<double>> sensor_fusion);
};


#endif //PATH_PLANNING_TRAFFIC_H
