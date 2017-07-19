//
// Created by jun on 7/16/17.
//

#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <vector>
#include <map>


class Car;  // forward declaration
class Ego;  // forward declartion


class Traffic {
private:
  std::map<int, Car> cars_;

public:

  //
  // constructor
  //
  Traffic();

  //
  // destructor
  //
  ~Traffic();

  void update_state(const std::map<int, std::vector<double>>& sensor_fusion);

  //
  // Check whether an ego car will collide with other cars in the traffic
  //
  bool checkCollision(Ego& ego);

  //
  // Print out the traffic
  //
  void printout();
};


#endif //PATH_PLANNING_TRAFFIC_H
