//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#ifndef PATH_PLANNING_UTILITIES_H
#define PATH_PLANNING_UTILITIES_H

//
// For converting back and forth between radians and degrees.
//
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

//
//
//
template <class T>
inline void print1DContainer(T v) {
  for ( auto i : v ) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;
}

//
//
//
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

//
// Checks if the SocketIO event has JSON data.
//
inline std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
//
//
inline int closestWaypoint(double x, double y,
                           std::vector<double> maps_x,
                           std::vector<double> maps_y) {

  double closestLen = 100000; //large number
  int closest_waypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if ( dist < closestLen ) {
      closestLen = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

//
//
//
inline int nextWaypoint(double x, double y, double theta,
                        std::vector<double> maps_x,
                        std::vector<double> maps_y) {

  int closest_waypoint = closestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closest_waypoint];
  double map_y = maps_y[closest_waypoint];

  double heading = std::atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if(angle > pi()/4) { ++closest_waypoint; }

  return closest_waypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline std::vector<double> getFrenet(double x, double y, double theta,
                                     std::vector<double> maps_x,
                                     std::vector<double> maps_y) {

  int next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) { prev_wp  = maps_x.size() - 1; }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y)/(n_x*n_x + n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if ( centerToPos <= centerToRef ) { frenet_d *= -1; }

  // calculate s value
  double frenet_s = 0;
  for ( int i = 0; i < prev_wp; i++ ) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline std::vector<double> getXY(double s, double d, std::vector<double> maps_s,
                          std::vector<double> maps_x, std::vector<double> maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

#endif //PATH_PLANNING_UTILITIES_H
