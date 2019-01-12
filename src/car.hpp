/*
 * Note!!!
 *
 * The simulator does not simulate the real physical movement of a car. Instead,
 * every 20 ms the car moves to the next point on the list perfectly. The car's
 * new rotation becomes the line between the previous waypoint and the car's new
 * location.
 *
 * The simulator runs a cycle every 20 ms, but your C++ path planning program
 * will provide a new path at least one 20 ms cycle behind. The simulator will
 * simply keep progressing down its last given path while it waits for a new
 * generated path. As a result, by the time a new path reaches the simulator,
 * the vehicle has already passed the first few waypoints on that path. The
 * simulator has built-in tools to deal with this timing difference. The
 * simulator actually expects the received path to be a little out of date
 * compared to where the car is, and the simulator will consider which point
 * on the received path is closest to the car and adjust appropriately.
 *
 */
#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <iostream>
#include <vector>
#include <list>

#include "map.hpp"
#include "utilities.hpp"


class CarState;


class Car {

public:
  using surroundings = std::vector<std::vector<std::vector<double>>>;
  using trajectory = std::pair<std::vector<double>, std::vector<double>>;

private:
  bool is_initialized_;

  double time_step_; // timestep in second

  // parameters in Cartisian coordinate system
  double px_; // in m
  double py_; // in m
  double vx_; // in m/s
  double vy_; // in m/s
  double ax_; // in m/s^2
  double ay_; // in m/s^2

  // parameters in Frenet-Serret coordinate system
  double ps_; // in m
  double pd_; // in m
  double vs_; // in m/s
  double vd_; // in m/s
  double as_; // in m/s^2
  double ad_; // in m/s^2

  Map map_;

  CarState* state_; // vehicle state

  // surrounding vehicles information
  // [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]]
  surroundings surroundings_;

  uint8_t target_lane_id_; // target lane ID

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  double max_speed_ = mph2mps(50); // maximum speed (m/s)
  double max_acceleration_ = 10; // maximum acceleration (m/s^2)
  double max_jerk_ = 10; // maximum jerk (m/s^3)
  double max_steering_; // maximum steering angle (rad)

  // We care about traffic within this distance (m) since the detector
  // has its own range in reality.
  double max_evaluation_distance_; // in m

public:
  explicit Car(const Map& map);

  ~Car();

  /**
   * Update parameters based on the localization data.
   *
   * @param localization: x, y, speed, yaw, s, d
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
  std::pair<std::vector<double>, std::vector<double>> getClosestVehicles(uint16_t lane_id) const;

  /**
   * Estimate the state of car at the path end.
   * @return: ((ps, vs, as), (pd, vd, ad))
   */
  trajectory estimateFinalState() const;

  /**
   * Truncate a path (remove tail) to a given length
   *
   * @param n_keep: number of points to keep
   */
  void truncatePath(unsigned int n_keep);

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

  trajectory getPathXY() const;

  double getMaxSpeed() const;
  double getMaxAcceleration() const;
  double getMaxJerk() const;
  double getMaxSteering() const;

  // Print out the vehicle's information
  void info() const;

  uint16_t getCurrentLaneId() const;
  double getCurrentLaneCenter() const;
  uint16_t getTargetLaneId() const;

  void setTargetLaneId(uint8_t value);

  bool isAroundOrigin() const;
};


#endif //PATH_PLANNING_CAR_H
