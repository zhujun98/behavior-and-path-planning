//
// Created by jun on 7/16/17.
//
#ifndef PATH_PLANNING_TRAFFIC_H
#define PATH_PLANNING_TRAFFIC_H

#include <vector>


class Ego;
class Vehicle;
class Map;


class Traffic {
private:

  const Map* map_;

  Ego* ego_car_;
  std::vector<Vehicle> other_cars_;

public:

  //
  // constructor
  //
  Traffic(Ego& ego, const Map& map);

  //
  // destructor
  //
  ~Traffic();

  void update(const std::vector<double>& localization,
              const std::vector<std::vector<double>>& sensor_fusion);

  //
  // Check whether an ego car will collide with other cars in the traffic
  //
  bool checkCollision(double t);

  //
  // Print out the traffic
  //
  void printout();
};


#endif //PATH_PLANNING_TRAFFIC_H
