#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>
#include <memory>
#include <vector>
#include <limits>

#include "spline.h"

#include "map.hpp"
#include "utilities.hpp"


Map::Map(const std::string& file_path, double max_s)
  : n_lanes_(3), lane_width_(4), max_s_(max_s)
{
  // Read waypoints data
  std::ifstream ifs(file_path.c_str(), std::ifstream::in);
  if (!ifs) throw std::invalid_argument("Cannot open the map file: " + file_path);

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

    x_.push_back(px);
    y_.push_back(py);
    s_.push_back(ps);
    dx_.push_back(dpx);
    dy_.push_back(dpy);
  }

  ifs.close();
}

Map::~Map() = default;

uint16_t Map::nLanes() const { return n_lanes_; }
double Map::laneWidth() const { return lane_width_; }

std::size_t Map::size() const { return x_.size(); }

double Map::maxS() const { return max_s_; }

/**
 * get the lane id based on the Frenet coordinate d
 */
uint16_t Map::getLaneId(double d) const {
  if (d < 0) return 0;
  if (d > n_lanes_ * lane_width_) return (uint16_t)(n_lanes_ + 1);
  return static_cast<uint16_t>(d / lane_width_ + 1);
}

/**
 * Get lane center from ID. If the lane ID does not exist, it returns the center
 * of the nearest valid lane.
 */
double Map::getLaneCenter(uint16_t lane_id) const {
  if (lane_id == 0) lane_id = 1;
  if (lane_id > n_lanes_) lane_id = n_lanes_;
  return (lane_id - 0.5) * lane_width_;
}

std::size_t Map::closestWaypoint(double x, double y) {
  std::size_t closest_wpt = 0;
  double closest = std::numeric_limits<double>::max();
  for (std::size_t i=0; i < x_.size(); ++i) {
    double dist = distance(x, y, x_[i], y_[i]);
    if (dist < closest) {
      closest = dist;
      closest_wpt = i;
    }
  }

  return closest_wpt;
}


std::size_t Map::nextWaypoint(double x, double y, double yaw) {
  std::size_t closest_wpt = closestWaypoint(x, y);

  double closest_yaw = atan2(y_[closest_wpt] - y, x_[closest_wpt] - x);
  double angle = std::abs(yaw - closest_yaw);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/4) {
    ++closest_wpt;
    if (closest_wpt == x_.size()) closest_wpt = 0;
  }

  return closest_wpt;
}


position Map::cartesianToFrenet(double x, double y, double yaw) {
  std::size_t next_wpt = nextWaypoint(x, y, yaw);
  std::size_t prev_wpt;
  if (next_wpt == 0) prev_wpt = x_.size() - 1;
  else prev_wpt = next_wpt - 1;

  // find the projection of x onto n
  double n_x = x_[next_wpt] - x_[prev_wpt];
  double n_y = y_[next_wpt] - y_[prev_wpt];
  double x_x = x - x_[prev_wpt];
  double x_y = y - y_[prev_wpt];

  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  // calculate the Frenet coordinate d
  double d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - x_[prev_wpt];
  double center_y = 2000 - y_[prev_wpt];
  double center_to_pos = distance(center_x, center_y, x_x, x_y);
  double center_to_ref = distance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) d *= -1;

  // calculate the Frenet coordinate s
  double s = 0;
  for (std::size_t i = 0; i < prev_wpt; ++i)
    s += distance(x_[i], y_[i], x_[i+1], y_[i+1]);

  s += distance(0, 0, proj_x, proj_y);
  if (s > max_s_) s -= max_s_;
  else if (s < 0) s += max_s_;

  return {s, d};
}


position Map::frenetToCartesian(double s, double d) {
  while (s > max_s_) s -= max_s_;
  while (s < 0) s += max_s_;
  long next_wpt = std::distance(s_.begin(), std::lower_bound(s_.begin(), s_.end(), s));

  std::vector<double> local_traj_s;
  std::vector<double> local_traj_x;
  std::vector<double> local_traj_y;
  // apply interpolation around the next waypoint
  for (int i = -5; i < 5; ++i ) {
    long wpt = next_wpt + i;

    // we either have wpt < 0 or wpt > 0
    if (wpt < 0) {
      wpt += s_.size();
      local_traj_s.push_back(s_[wpt] - max_s_);
    } else if (wpt >= s_.size()) {
      wpt -= s_.size();
      local_traj_s.push_back(s_[wpt] + max_s_);
    } else {
      local_traj_s.push_back(s_[wpt]);
    }
    local_traj_x.push_back(x_[wpt]);
    local_traj_y.push_back(y_[wpt]);
  }

  tk::spline spline_sx;
  spline_sx.set_points(local_traj_s, local_traj_x);

  tk::spline spline_sy;
  spline_sy.set_points(local_traj_s, local_traj_y);

  double x = spline_sx(s);
  double y = spline_sy(s);

  double slope_x = spline_sx.deriv(1, s);
  double slope_y = spline_sy.deriv(1, s);
  double yaw = std::atan2(slope_y, slope_x) - pi()/2;

  x += d * std::cos(yaw);
  y += d * std::sin(yaw);

  return {x, y};
}