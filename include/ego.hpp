//
// Created by jun on 7/25/17.
//
#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H

#include <iostream>
#include <vector>
#include <list>

#include "map.hpp"


class EgoState;


class Ego {

  using surroundings = std::vector<std::vector<std::vector<double>>>;
  using trajectory = std::pair<std::vector<double>, std::vector<double>>;
  using car_state = std::vector<double>;

  bool is_initialized_;

  double time_step_; // time step in s

  // parameters in global coordinate system
  double px_; // in m
  double py_; // in m
  double vx_; // in m/s
  double vy_; // in m/s

  // parameters in Frenet-Serret coordinate system
  double ps_; // in m
  double pd_; // in m
  double vs_; // in m/s
  double vd_; // in m/s
  double as_; // in m/s^2
  double ad_; // in m/s^2

  Map map_;

  EgoState* state_; // vehicle state

  // surrounding vehicles information
  // [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  surroundings surroundings_;

  uint8_t target_lane_id_; // target lane ID

  trajectory path_;

  double max_speed_; // maximum speed (m/s)
  double max_acceleration_; // maximum acceleration (m/s^2)
  double max_jerk_; // maximum jerk (m/s^3)
  double max_steering_; // maximum steering angle (rad)

  // We care about traffic within this distance (m) since the detector
  // has its own range in reality.
  double max_evaluation_distance_; // in m

  /**
   * Update parameters based on the localization data.
   *
   * @param localization: x, y, vx, vy, s, d
   */
  void updateParameters(const std::vector<double>& localization);

  /**
   * Update information of the surrounding vehicles
   */
  void updateSurroundings(const std::vector<std::vector<double>>& sensor_fusion);

  /**
   * Update the unprocessed waypoints.
   */
  void updateUnprocessedPath();

public:

  explicit Ego(const Map& map);

  ~Ego();

  /**
   * Update all parameters and states.
   */
  void update(const std::vector<double>& localization,
              const std::vector<std::vector<double>>& sensor_fusion);

  /**
   * Get the closest vehicles (front and rear) on a given lane.
   *
   * @param lane_id: lane ID
   * @return: (front car_state, rear car_state)
   */
  std::pair<car_state, car_state> getClosestVehicles(uint8_t lane_id) const;

  /**
   * Truncate a path (remove tail) to a given length
   *
   * @param n_keep: number of points to keep
   */
  void truncatePath(unsigned int n_keep);

  /**
   * Append the new path to the end of the old path
   */
  void extendPath(trajectory new_path);

  /**
   * Plan the path in order to follow the traffic.
   */
  void followTraffic();

  /**
   * Plan the path in order to shift to the left lane.
   */
  void shiftLaneLeft();

  /**
   * Plan the path in order to shift to the right lane.
   */
  void shiftLaneRight();

  /**
   * Get the unprocessed waypoints (x, y).
   */
  trajectory getPath();

  car_state getCurrentState() const;

  double getMaxSpeed() const;
  double getMaxAcceleration() const;
  double getMaxJerk() const;
  double getMaxSteering() const;

  // Print out the vehicle's information
  void info() const;

  uint8_t getCurrentLaneId() const;
  uint8_t getTargetLaneId() const;

  void setTargetLaneId(uint8_t value);

  bool isAroundOrigin() const;
};


#endif //PATH_PLANNING_EGO_H
