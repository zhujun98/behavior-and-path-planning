#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <vector>
#include <string>

#include "common.hpp"


class Map {

  uint16_t n_lanes_; // number of lanes on one side
  double lane_width_; // width (in meter) of the lane

  // coordinates in the global Cartesian coordinate system
  std::vector<double> x_;
  std::vector<double> y_;
  // longitudinal Frenet coordinate along the reference trajectory
  std::vector<double> s_;
  // components of the unit vector d which is normal to the reference trajectory.
  // (pointing to the right of the traffic direction)
  std::vector<double> dx_;
  std::vector<double> dy_;

  double max_s_;  // maximum Frenet coordinate s, in meter

public:
  explicit Map(const std::string& file_path, double max_s=6945.554);

  ~Map();

  uint16_t nLanes() const;
  double laneWidth() const;

  std::size_t size() const;
  double maxS() const;

  /**
   * get the lane id based on the Frenet coordinate d
   */
  uint16_t getLaneId(double d) const;

  /**
   * Get lane center from ID. If the lane ID does not exist, it returns the center
   * of the nearest valid lane.
   */
  double getLaneCenter(uint16_t lane_id) const;

  /**
   * Calculate the index of the closest waypoint.
   */
  std::size_t closestWaypoint(double x, double y);

  /**
   * Get the index of the next waypoint.
   */
  std::size_t nextWaypoint(double x, double y, double yaw);


  /**
   * Convert the Cartesian coordinate in a trajectory to the Frenet coordinate.
   */
  position cartesianToFrenet(double x, double y, double yaw);

  /**
   * Convert the Frenet coordinate in a trajectory to the Cartesian coordinate.
   */
  position frenetToCartesian(double s, double d);
};


#endif //PATH_PLANNING_MAP_H