//
// Created by jun on 7/17/17.
//
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "map.h"
#include "utilities.h"


Map::Map() {
  n_lanes_ = 3;

  max_s_ = 6945.554;

  load_data();
}


Map::~Map() {}


void Map::load_data() {
  // Read waypoints data
  std::string map_file_ = "../data/highway_map.csv";
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
}

int Map::closestWaypoint(double x, double y) const {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < x_.size(); i++) {
    double map_x = x_[i];
    double map_y = y_[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Map::nextWaypoint(double x, double y, double theta) const {

  int closest_waypoint = closestWaypoint(x, y);

  double map_x = x_[closest_waypoint];
  double map_y = y_[closest_waypoint];

  double heading = std::atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if(angle > pi()/4) { ++closest_waypoint; }

  return closest_waypoint;
}

//
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//
std::pair<double, double> Map::cartesianToFrenet(double x, double y, double theta) const {

  int next_wp = nextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) { prev_wp  = x_.size() - 1; }

  double n_x = x_[next_wp] - x_[prev_wp];
  double n_y = y_[next_wp] - y_[prev_wp];
  double x_x = x - x_[prev_wp];
  double x_y = y - y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - x_[prev_wp];
  double center_y = 2000 - y_[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if ( centerToPos <= centerToRef ) { frenet_d *= -1; }

  // calculate s value
  double frenet_s = 0;
  for ( int i = 0; i < prev_wp; i++ ) {
    frenet_s += distance(x_[i], y_[i], x_[i+1], y_[i+1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return std::make_pair(frenet_s, frenet_d);

}

//
// Transform from Frenet s,d coordinates to Cartesian x,y
//
std::pair<double, double> Map::frenetToCartesian(double s, double d) const {
  int prev_wp = -1;

  while (s > s_[prev_wp + 1] && (prev_wp < (int) (s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % x_.size();

  double heading = atan2((y_[wp2] - y_[prev_wp]), (x_[wp2] - x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - s_[prev_wp]);

  double seg_x = x_[prev_wp] + seg_s * cos(heading);
  double seg_y = y_[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return std::make_pair(x, y);

}

vehicle_traj Map::trajFrenetToCartesian(const vehicle_traj& traj) {
  vehicle_traj traj_cartesian;
  std::pair<double, double> xy;

  for ( auto is = traj.first.begin(), id = traj.second.begin();
        is != traj.first.end(); ++is, ++id) {
    if ( *is > max_s_ ) {
      xy = frenetToCartesian(*is - max_s_, *id);
    } else {
      xy = frenetToCartesian(*is, *id);
    }

    traj_cartesian.first.push_back(xy.first);
    traj_cartesian.second.push_back(xy.second);
  }

  return traj_cartesian;
}

vehicle_traj Map::trajCartesianToFrenet(const vehicle_traj& traj) {

  vehicle_traj traj_frenet;
  std::pair<double, double> sd;

  for ( auto ix = traj.first.begin(), iy = traj.second.begin();
        ix != traj.first.end(); ++ix, ++iy) {
    sd = cartesianToFrenet(*ix, *iy, 0.0);

    traj_frenet.first.push_back(sd.first);
    traj_frenet.second.push_back(sd.second);
  }

  return traj_frenet;
}
