//
// Created by jun on 7/17/17.
//
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <functional>

#include "spline/spline.h"
#include "map.h"
#include "utilities.h"
#include "parameters.h"


Map::Map() : n_lanes_(3), lane_width_(4), max_s_(kMAX_S) {
  // Read waypoints data
  std::string map_file_ = kMAP_PATH;
  std::ifstream ifs(map_file_.c_str(), std::ifstream::in);

  double x;
  double y;
  double s;
  double dx;
  double dy;

  std::string line;
  while (getline(ifs, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    x_.push_back(x);
    y_.push_back(y);
    s_.push_back(s);
    dx_.push_back(dx);
    dy_.push_back(dy);
  }

  ifs.close();;
}

Map::~Map() = default;

int Map::computerLaneID(double d) const {
  int id = (int)(d / lane_width_) + 1;

  if ( id < 1 || id > 3 ) {
    // out of lane
    return -1;
  }

  return id;
}

double Map::distance(double x0, double y0, double x1, double y1) const {
  double dx = x1 - x0;
  double dy = y1 - y0;

  return std::sqrt(dx*dx + dy*dy);
}

size_t Map::closestWaypoint(double x, double y) const {
  std::vector<double> l2;
  for ( int i=0; i < x_.size(); ++ i) {
    l2.push_back(distance(x, y, x_[i], y_[i]));
  }

  auto it = std::min_element(l2.begin(), l2.end());
  auto closest_waypoint = (unsigned long)std::distance(l2.begin(), it);

  return closest_waypoint;
}

size_t Map::nextWaypoint(double x, double y, double theta) const {
  auto closest_waypoint = closestWaypoint(x, y);

  double map_x = x_[closest_waypoint];
  double map_y = y_[closest_waypoint];

  double heading = atan2((map_y - y), (map_x - x));
  double angle = std::abs(theta - heading);

  if (angle > pi()/4) { ++closest_waypoint; }

  return closest_waypoint;
}

std::pair<double, double> Map::cartesianToFrenet(double x, double y) const {
  std::cerr << "cartesianToFrenet is not implemented correctly!" << std::endl;

  auto closest_wpt = closestWaypoint(x, y);
  size_t max_index = s_.size();
  auto prev_wpt = closest_wpt - 1;
  if (prev_wpt > max_index) { prev_wpt -= max_index; }
  auto next_wpt = closest_wpt + 1;
  if (next_wpt > max_index) { next_wpt -= max_index; }

  double n_x = x_[next_wpt] - x_[prev_wpt];
  double n_y = y_[next_wpt] - y_[prev_wpt];
  double x_x = x - x_[prev_wpt];
  double x_y = y - y_[prev_wpt];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - x_[prev_wpt];
  double center_y = 2000 - y_[prev_wpt];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if ( centerToPos <= centerToRef ) { frenet_d *= -1; }

  // calculate s value
  double frenet_s = 0;
  for ( size_t i = 0; i < next_wpt; i++ ) {
    frenet_s += distance(x_[i], y_[i], x_[i+1], y_[i+1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return std::make_pair(frenet_s, frenet_d);
}

std::pair<double, double> Map::frenetToCartesian(double s, double d) const {
  if ( s > max_s_ ) { s -= max_s_; }
  auto next_waypoint = (unsigned long)std::distance(
      s_.begin(), std::lower_bound(s_.begin(), s_.end(), s));

  std::vector<double> local_s;
  std::vector<double> local_x;
  std::vector<double> local_y;
  for ( long i = -5; i < 5; ++i ) {
    long index = next_waypoint + i;
    if ( index < 0 ) {
      index += s_.size();
      local_s.push_back(s_[index] - max_s_);
    } else if ( index >= s_.size() ) {
      index -= s_.size();
      local_s.push_back(s_[index] + max_s_);
    } else {
      local_s.push_back(s_[index]);
    }
    local_x.push_back(x_[index]);
    local_y.push_back(y_[index]);
  }

  tk::spline spline_sx;
  spline_sx.set_points(local_s, local_x);

  tk::spline spline_sy;
  spline_sy.set_points(local_s, local_y);

  double x = spline_sx(s);
  double y = spline_sy(s);

  double dx = spline_sx(s + 0.1) - spline_sx(s - 0.1);
  double dy = spline_sy(s + 0.1) - spline_sy(s - 0.1);
  double yaw = std::atan2(dy, dx) - pi()/2;

  x += d*std::cos(yaw);
  y += d*std::sin(yaw);

  return std::make_pair(x, y);
}

// TODO:: improve the algorithm
vehicle_trajectory Map::trajFrenetToCartesian(const vehicle_trajectory& trajectory) const {
  vehicle_trajectory trajectory_cartesian;
  std::pair<double, double> xy;

  for ( auto is = trajectory.first.begin(), id = trajectory.second.begin();
        is != trajectory.first.end(); ++is, ++id) {
    if ( *is > max_s_ ) {
      xy = frenetToCartesian(*is - max_s_, *id);
    } else {
      xy = frenetToCartesian(*is, *id);
    }

    trajectory_cartesian.first.push_back(xy.first);
    trajectory_cartesian.second.push_back(xy.second);
  }

  return trajectory_cartesian;
}

vehicle_trajectory Map::trajCartesianToFrenet(const vehicle_trajectory& trajectory) const {
  vehicle_trajectory trajectory_frenet;
  std::pair<double, double> sd;

  for ( auto ix = trajectory.first.begin(), iy = trajectory.second.begin();
        ix != trajectory.first.end(); ++ix, ++iy) {
    sd = cartesianToFrenet(*ix, *iy);

    trajectory_frenet.first.push_back(sd.first);
    trajectory_frenet.second.push_back(sd.second);
  }

  return trajectory_frenet;
}

double Map::getLaneWidth() const { return lane_width_; }

double Map::getMaxS() const { return max_s_; }

size_t Map::getNoLanes() const { return n_lanes_; }