//
// Created by jun on 12/10/18.
//

#include <exception>

#include "trajectory.hpp"
#include "utilities.hpp"
#include "spline/spline.h"


std::size_t closestWaypoint(double x, double y, const trajectory& traj_x, const trajectory& traj_y) {
  std::size_t closest_wpt = 0;
  double closest = std::numeric_limits<double>::max();
  for (std::size_t i=0; i < traj_x.size(); ++i) {
    double dist = distance(x, y, traj_x[i], traj_y[i]);
    if (dist < closest) {
      closest = dist;
      closest_wpt = i;
    }
  }

  return closest_wpt;
}


std::size_t nextWaypoint(double x, double y, double yaw, const trajectory& traj_x, const trajectory& traj_y) {
  std::size_t closest_wpt = closestWaypoint(x, y, traj_x, traj_y);

  double closest_yaw = atan2(traj_y[closest_wpt] - y, traj_x[closest_wpt] - x);
  double angle = std::abs(yaw - closest_yaw);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/4) {
    ++closest_wpt;
    if (closest_wpt == traj_x.size()) closest_wpt = 0;
  }

  return closest_wpt;
}


position cartesianToFrenet(double x, double y, double yaw, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y) {

  std::size_t next_wpt = nextWaypoint(x, y, yaw, traj_x, traj_y);
  std::size_t prev_wpt;
  if (next_wpt == 0) prev_wpt = traj_x.size() - 1;
  else prev_wpt = next_wpt - 1;

  // find the projection of x onto n
  double n_x = traj_x[next_wpt] - traj_x[prev_wpt];
  double n_y = traj_y[next_wpt] - traj_y[prev_wpt];
  double x_x = x - traj_x[prev_wpt];
  double x_y = y - traj_y[prev_wpt];

  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  // calculate the Frenet coordinate d
  double d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - traj_x[prev_wpt];
  double center_y = 2000 - traj_y[prev_wpt];
  double center_to_pos = distance(center_x, center_y, x_x, x_y);
  double center_to_ref = distance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) d *= -1;

  // calculate the Frenet coordinate s
  double s = 0;
  for (std::size_t i = 0; i < prev_wpt; ++i)
    s += distance(traj_x[i], traj_y[i], traj_x[i+1], traj_y[i+1]);

  s += distance(0, 0, proj_x, proj_y);
  if (s > max_s) s -= max_s;
  else if (s < 0) s += max_s;

  return {s, d};
}


position frenetToCartesian(double s, double d, const trajectory& traj_s, double max_s,
                           const trajectory& traj_x, const trajectory& traj_y) {
  while (s > max_s) s -= max_s;
  while (s < 0) s += max_s;
  long next_wpt = std::distance(traj_s.begin(), std::lower_bound(traj_s.begin(), traj_s.end(), s));

  trajectory local_traj_s;
  trajectory local_traj_x;
  trajectory local_traj_y;
  // apply interpolation around the next waypoint
  for (int i = -5; i < 5; ++i ) {
    long wpt = next_wpt + i;

    // we either have wpt < 0 or wpt > 0
    if (wpt < 0) {
      wpt += traj_s.size();
      local_traj_s.push_back(traj_s[wpt] - max_s);
    } else if (wpt >= traj_s.size()) {
      wpt -= traj_s.size();
      local_traj_s.push_back(traj_s[wpt] + max_s);
    } else {
      local_traj_s.push_back(traj_s[wpt]);
    }
    local_traj_x.push_back(traj_x[wpt]);
    local_traj_y.push_back(traj_y[wpt]);
  }

  tk::spline spline_sx;
  spline_sx.set_points(local_traj_s, local_traj_x);

  tk::spline spline_sy;
  spline_sy.set_points(local_traj_s, local_traj_y);

  double x = spline_sx(s);
  double y = spline_sy(s);

  double dx = spline_sx(s + 0.1) - spline_sx(s - 0.1);
  double dy = spline_sy(s + 0.1) - spline_sy(s - 0.1);
  double yaw = std::atan2(dy, dx) - pi()/2;

  x += d * std::cos(yaw);
  y += d * std::sin(yaw);

  return {x, y};
}
