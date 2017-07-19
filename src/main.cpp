//
// Created by jun on 7/16/17.
//
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "utilities.h"
#include "vehicle.h"
#include "map.h"
#include "traffic.h"


int main() {

  uWS::Hub h;

  Ego my_car;
  Traffic traffic;
  Map highway_map;

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  h.onMessage([&my_car, &traffic, &highway_map]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      std::string s = hasData(data);

      if (s != "") {
        auto j = nlohmann::json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization data (without noise).
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_speed = j[1]["speed"];  // in MPH
          double car_yaw = j[1]["yaw"];  // in degree
          car_speed *= 4.0/9;  // MPH to m/s
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];

          std::vector<double> localization = {car_x, car_y, car_s, car_d, car_yaw, car_speed};

          // Previous path data passed to the planner.
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // End s and d values of the previous path.
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // All other car's data on the same side of the road
          // in the format [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d]].
          auto sensor_fusion_readout = j[1]["sensor_fusion"];
          std::map<int, std::vector<double>> sensor_fusion;
          for ( auto it : sensor_fusion_readout ) {
            int key = it[0];
            double vx = it[3];
            double vy = it[4];
            double speed = std::sqrt(vx*vx + vy*vy);
            double yaw = std::atan2(vy, vx);
            it[3] = speed;
            it[4] = yaw;
            std::vector<double> value (it.begin() + 1, it.end());
            sensor_fusion.insert(std::make_pair(key, value));
          }

          nlohmann::json msgJson;

          assert( previous_path_x.size() == previous_path_y.size() );

          //
          // Keep first maximum 10 points from the unfinished path
          //
          int keep_points = 10;
          std::vector<double> continued_path_x;
          std::vector<double> continued_path_y;
          if ( previous_path_x.size() == 0 ) {
            continued_path_x.push_back(car_x);
            continued_path_y.push_back(car_y);
          } else if ( previous_path_x.size() < keep_points ) {
            for ( auto i : previous_path_x ) { continued_path_x.push_back(i); }
            for ( auto i : previous_path_y ) { continued_path_y.push_back(i); }
          } else {
            for ( auto it = previous_path_x.begin();
                  it != previous_path_x.begin() + keep_points; ++it ) {
              continued_path_x.push_back(*it);
            }
            for ( auto it = previous_path_y.begin();
                  it != previous_path_y.begin() + keep_points; ++it ) {
              continued_path_y.push_back(*it);
            }
          }

          // Transfer the trajectory in Cartesian coordinate system to
          // the corresponding one in Frenet coordinate system and pass
          // the transformed trajectory to "my_car"
          auto trajectory_frenet = highway_map.trajCartesianToFrenet(
              std::make_pair(continued_path_x, continued_path_y));

          // Update the state of the ego car
          my_car.update_state(localization, trajectory_frenet);

          // Update the states of other vehicles on the road
          traffic.update_state(sensor_fusion);

          // Path planning
          trajectory_frenet = my_car.plan_path();

          // Transfer the trajectory in Frenet coordinate system output
          // by "my_car" and pass it to the simulator.
          auto trajectory_cartesian = highway_map.trajFrenetToCartesian(trajectory_frenet);

          msgJson["next_x"] = trajectory_cartesian.first;
          msgJson["next_y"] = trajectory_cartesian.second;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
