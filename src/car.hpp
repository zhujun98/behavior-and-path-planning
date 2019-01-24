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
#include <map>
#include <memory>

#include "common.hpp"

class Map;
class PathOptimizer;


class Car {
  //
  // CL: change lane (to left or right)
  // KL: keep lane
  // ST: start up
  //
  enum class States {CL, KL, ST};

  class State;
  class StateStartUp;
  class StateKeepLane;
  class StateChangeLane;

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

  std::unique_ptr<Map> map_;

  std::unique_ptr<State> state_;

  std::unique_ptr<PathOptimizer> path_opt_;

  // key: lane ID, value: vehicle dynamics
  std::map<uint16_t, dynamics> closest_front_cars_;
  std::map<uint16_t, dynamics> closest_rear_cars_;

  uint16_t target_lane_id_; // target lane ID

  std::vector<double> path_s_;
  std::vector<double> path_d_;

  // ignore cars which are too far away (not detectable in real life)
  double max_tracking_dist_ = 100; // in m

  /**
   * Estimate the dynamics of car at the path end.
   * @return: ((ps, vs, as), (pd, vd, ad))
   */
  dynamics estimateFinalDynamics() const;

  /**
   * Update the unprocessed waypoints.
   */
  void updateUnprocessedPath();

  /**
   * Truncate a path (remove tail) to a given length
   *
   * @param n_keep: number of points to keep
   */
  void truncatePath(unsigned int n_keep);

  /**
   * Print out the vehicle's information.
   */
  void info() const;

  /**
   * Update path when starting up.
   */
  bool startUp();

  /**
   * Update path when keeping lane.
   */
  bool keepLane();

  /**
   * Update path when changing lane.
   */
  bool changeLane();

  /**
   * Check whether the given path could collide with a given car.
   */
  bool checkCollision(const trajectory& path, const dynamics& dyn) const;

  /**
   * Check whether the given path could collide the front car in the current lane
   * or the cars in the target lane.
   */
  bool checkAllCollisions(const trajectory& path) const;

  /**
   * construct a new state
   */
  static State* createState(States name);

public:
  static double inf_dist; // a big number represents infinite distance

  explicit Car(const std::string& file_path, double time_step=0.02);

  ~Car();

  /**
   * Update all parameters and states.
   */
  void update(const std::vector<double>& localization,
              const std::vector<std::vector<double>>& sensor_fusion);

  /**
   * Update parameters based on the localization data.
   *
   * @param localization: x, y, speed, yaw, s, d
   */
  void updateParameters(const std::vector<double>& localization);

  /**
   * Update dynamics of the closest vehicles in each lane.
   */
  void updateClosestVehicles(const std::vector<std::vector<double>>& sensor_fusion);

  /**
   * Extend the current path.
   */
  void extendPath(trajectory&& traj);

  /**
   * Return the corresponding Cartisian path of the current Frenet path.
   */
  trajectory getPathXY() const;

  /**
   * Analyze and return the optimized lane ID.
   */
  uint16_t getOptimizedLaneId() const;

  const std::map<uint16_t, dynamics>& getClosestFrontVehicles() const;
  const std::map<uint16_t, dynamics>& getClosestRearVehicles() const;

  uint16_t getCurrentLaneId() const;
  double getCurrentLaneCenter() const;

  uint16_t getTargetLaneId() const;
  void setTargetLaneId(uint16_t id);
  double getTargetLaneCenter() const;

};


#endif //PATH_PLANNING_CAR_H
