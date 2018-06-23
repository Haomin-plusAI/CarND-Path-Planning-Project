#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "helpers.h"
#include "behaviour.h"
#include "state_machine.h"
#include "trajectory.h"
#include "collision_detector.h"
#include "path_generator.h"
#include "cost_functions.h"
#include "path_validator.h"
#include "map.h"
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

// 20 millisecond
double MIN_TIMESTEP = 0.02;

int PATH_LENGTH = 50;
int PREVIOUS_PATH_POINTS_TO_KEEP = 40;

// The maximum duration of a maneuvre in seconds
double MAX_MANEUVER_DURATION = 1;

double current_acceleration = 1.0;

double MAX_ACCELERATION = 1.3;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void printCarPosition(const Vehicle &ego)
{
  cout << "(x=" << ego.x << ", y=" << ego.y << ") "
       << "(s=" << ego.s << ", d=" << ego.d << ") "
       << "yaw=" << ego.theta << ", speed=" << ego.getSpeed()
       << endl;
}

int main()
{
  uWS::Hub h;
  Map &map = Map::getInstance();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);

    map.addWaypoint(x, y, s, d_x, d_y);
  }

  map.buildSplines();

  // This is the trajectory adopted by the car
  Trajectory main_trajectory;
  Behaviour behaviour = Behaviour();
  const PathValidator path_validator = PathValidator();

  h.onMessage([&behaviour, &path_validator, &map, &main_trajectory, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                                                                                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = fmod(j[1]["s"], MAX_TRACK_S);
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          // cout << sensor_fusion << endl;
          vector<Vehicle> vehicles;
          for (auto v_data : sensor_fusion)
          {
            Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6], 0.0);
            vehicles.push_back(v);
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Car yaw is given in degrees, so we convert to radians
          double car_yaw_rad = deg2rad(car_yaw);

          vector<double> ppx;
          vector<double> ppy;
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            ppx.push_back(previous_path_x[i]);
            ppy.push_back(previous_path_y[i]);
          }

          int prev_path_size = previous_path_x.size();

          double car_speed_mps = KmPerHourToMetersPerSecond(milesPerHourToKmPerHour(car_speed));
          Vehicle ego = Vehicle(-1, car_x, car_y, cos(car_yaw_rad), sin(car_yaw_rad), car_s, car_d, 0.0);

          Trajectory chosen_trajectory;
          State chosen_state;
          double lowest_cost = 10000;
          bool state_chosen = false;

          chosen_trajectory = behaviour.nextTrajectory(ego, vehicles, ppx, ppy);

          for (int i = 0; i < chosen_trajectory.xs.size(); ++i)
          {
            next_x_vals.push_back(chosen_trajectory.xs[i]);
            next_y_vals.push_back(chosen_trajectory.ys[i]);
          }
          main_trajectory = chosen_trajectory;

          cout << "*** Updated trajectory -  size of path for controller = " << next_x_vals.size() << endl;
          printCarPosition(ego);

          // Setting up the our next waypoints
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
