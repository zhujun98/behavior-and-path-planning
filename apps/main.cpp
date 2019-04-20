#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>

#include "json.hpp"

#include "car.hpp"


/**
 * Get the JSON data from the SocketIO event.
 */
inline std::string parseSocketData(const std::string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/**
 * Convert degree to Radian.
 */
inline double deg2rad(double x) { return x * pi() / 180; }


/**
 * Convert speed from MPH to m/s.
 */
inline double mph2mps(double v) { return v * 4.0 / 9; }


int main(int argc, char* argv[]) {

  std::string map_name;
  if (argc >=2 ) map_name = argv[1];
  else map_name = "data/highway_map.csv";

  uWS::Hub h;

  Car car(kTimeStep, mph2mps(kMaxSpeedInMPH), kMaxAcc, kMaxJerk);
  car.loadMap(map_name);

  // Note: the first argument is a pointer, while in the earlier version it is passed by value!
  h.onMessage([&car](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2') {

      std::string socket_data = parseSocketData(data);

      if (!socket_data.empty()) {
        const nlohmann::json json_data = nlohmann::json::parse(socket_data);

        std::string event = json_data[0].get<std::string>();

        if (event == "telemetry") {
          // Car's localization data (without noise).
          double x = json_data[1]["x"];  // in m
          double y = json_data[1]["y"];  // in m
          double speed = mph2mps(json_data[1]["speed"]);  // in m/s
          double yaw = deg2rad(json_data[1]["yaw"]);  // in rad
          double s = json_data[1]["s"];  // in m
          double d = json_data[1]["d"];  // in m

          std::vector<double> localization = {x, y, speed * std::cos(yaw), speed * std::sin(yaw), s, d};

          // Unprocessed path data previously passed to the planner.
          auto previous_path_x = json_data[1]["previous_path_x"];
          auto previous_path_y = json_data[1]["previous_path_y"];

          // End point of the previous path.
//          double end_path_s = json_data[1]["end_path_s"];
//          double end_path_d = json_data[1]["end_path_d"];

          // All other car's data on the same side of the road
          // in the format [[ID, x (m), y (m), vx (m/s), vy (m/s), s (m), d (m)]].
          std::vector<std::vector<double>> sensor_fusion = json_data[1]["sensor_fusion"];

          nlohmann::json msgJson;

          car.update(localization, sensor_fusion);

          trajectory path_xy = car.getPathXY();
          // define the path that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = path_xy.first;
          msgJson["next_y"] = path_xy.second;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
      // I guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
                         char *message, size_t length) {
    ws->close();
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
