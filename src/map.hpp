//
// Created by jun on 7/17/17.
//
#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>
#include <vector>


struct Map {

  uint8_t n_lanes; // number of lanes on one side
  uint8_t lane_width; // width (in meter) of the lane

  // coordinates in the global Cartesian coordinate system
  std::vector<double> x;
  std::vector<double> y;
  // longitudinal Frenet coordinate along the reference trajectory
  std::vector<double> s;
  // components of the unit vector d which is normal to the reference trajectory.
  // (pointing to the right of the traffic direction)
  std::vector<double> dx;
  std::vector<double> dy;

  double max_s;  // maximum Frenet coordinate s, in meter

  Map(const std::string& map_file, double max_s=6945.554) : n_lanes(3), lane_width(4), max_s(max_s) {
    // Read waypoints data
    std::ifstream ifs(map_file.c_str(), std::ifstream::in);
    if (!ifs) throw std::invalid_argument("Cannot open the map file: " + map_file);

    double px;
    double py;
    double ps;
    double dpx;
    double dpy;

    std::string line;
    while (getline(ifs, line)) {
      std::istringstream iss(line);

      iss >> px;
      iss >> py;
      iss >> ps;
      iss >> dpx;
      iss >> dpy;

      x.push_back(px);
      y.push_back(py);
      s.push_back(ps);
      dx.push_back(dpx);
      dy.push_back(dpy);
    }

    ifs.close();
  }

  ~Map() = default;

  /**
   * get the lane id based on the Frenet coordinate d
   */
  uint16_t getLaneId(double d) const {
    if (d < 0) return 0;
    if (d > n_lanes * lane_width) return (uint16_t)(n_lanes + 1);
    return static_cast<uint16_t>(d / lane_width + 1);
  }

  /**
   * Get lane center from ID. If the lane ID does not exist, it returns the center
   * of the nearest valid lane.
   */
  double getLaneCenter(uint16_t lane_id) const {
    if (lane_id == 0) lane_id = 1;
    if (lane_id > n_lanes) lane_id = n_lanes;
    return (lane_id - 0.5) * lane_width;
  }
};


#endif //PATH_PLANNING_MAP_H