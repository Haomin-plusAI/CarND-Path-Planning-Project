#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// TODO - complete this function
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    // create T matrix
    MatrixXd t_matrix(3, 3);
    t_matrix << pow(T, 3), pow(T, 4), pow(T, 5),
                3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
                6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
        
    // create sf diff vector
    VectorXd sf_diff(3);
    sf_diff << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
               end[1] - (start[1] + start[2] * T),
               end[2] - start[2];
               
    VectorXd coeffs = t_matrix.inverse() * sf_diff;
    
    vector<double> final_coeffs = {start[0], start[1], 0.5 * start[2], coeffs[0], coeffs[1], coeffs[2]};
    return final_coeffs;
    
    // return {1,2,3,4,5,6};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
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

void printCarPosition(json pos){
	auto carPos = pos[1];
	cout << "(x=" << carPos["x"] << ", y=" << carPos["y"] << ") "
			 << "(s=" << carPos["s"] << ", d=" << carPos["d"] << ") "
			 << "yaw=" << carPos["yaw"] << ", speed=" << carPos["speed"]
			 << endl;

}


int main() {
  uWS::Hub h;
  Map& map = Map::getInstance();

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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

  // This is the trajectory adopted by the car
  Trajectory main_trajectory;
  Behaviour behaviour = Behaviour();


  h.onMessage([&behaviour, &map, &main_trajectory, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

						printCarPosition(j);

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
						for(auto v_data: sensor_fusion){
							Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6], 0.0);
							vehicles.push_back(v);
						}

						// TODO predict future position of vehicles
						// create a vector<vector <Vehicle>> for the next
						// T timesteps (20ms timestep)
            double acc_sign = 1.0;
            int myLane = calculateLane(car_d, 4.0, 1.5);
            bool withinLane = isWithinLane(car_d, 4.0, 1.5);
            

            CollisionDetector cdetector = CollisionDetector(main_trajectory);                                    
            for(const Vehicle& v : vehicles)
            {                
              if(!v.isInLane)
              {
                continue;
              }

              if(v.lane == myLane)
              {
                
                double dist = distance(car_x, car_y, v.x, v.y);
                double s_diff = v.s - car_s;
                // cout << "Vehicle " << v.id << " is " << dist << "m (s-diff=" << s_diff
                //       << ") away in the same lane " << endl;
                // if(car_s < v.s && s_diff < 25.0){
                //   acc_sign = -1;
                //   cout << "**** Will decelerate" << endl;
                // }
              }
            }

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
						double car_yaw_rad = deg2rad(car_yaw);
						
            int prev_path_size = previous_path_x.size();
            if(prev_path_size > 0 && prev_path_size < PATH_LENGTH)
            {
              // Make sure our main trajectory mirrors points in the previous_path
              main_trajectory.removeFirstN(main_trajectory.size() - prev_path_size);
              vector<double> a_a = main_trajectory.averageAcceleration(0.02);
              cout << "Trajectories updated: size=" << main_trajectory.size() 
                   << " avg speed=" << main_trajectory.averageSpeed(0.02) << "mps" 
                   << " avg acc=(" << a_a[0] << ", " << a_a[1] << ")" << endl;
            }

          
            double car_speed_mps = KmPerHourToMetersPerSecond(milesPerHourToKmPerHour(car_speed));
            Vehicle ego = Vehicle(-1, car_x, car_y, cos(car_yaw_rad), sin(car_yaw_rad), car_s, car_d, 0.0);            
            
            // double acc_s = 0.0;
            // if(previous_path_x.size() > 0)
            // {
            //   cout << "Previous path length = " << previous_path_x.size() << endl;
            //   double last_point_x = previous_path_x[0];
            //   double last_point_y = previous_path_y[0];
            //   double previous_theta = atan2(car_y - last_point_y, car_x - last_point_x);
            //   // double previous_theta = atan2(last_point_y - car_y, last_point_x - car_x );

            //   vector<double> previous_s_d = getFrenet(last_point_x, last_point_y, previous_theta, map_waypoints_x, map_waypoints_y);
            //   acc_s = car_s - previous_s_d[0];
            //   cout << "** S acceleration = " << acc_s << endl;

            //   acc_s = acc_s <= 0 ? 0 : acc_s * 50;
            // }

            
            
            Trajectory chosen_trajectory;
            State chosen_state;
            double lowest_cost = 10000;

            if(main_trajectory.size() == 0)
            {              
              
              // Little trick to seed our path_generator with the first position of the car
              main_trajectory.add(car_x, car_y, car_s, 0.0, 0.0, car_d, 0.0, 0.0, car_yaw_rad);              
              PathGenerator path_gen = PathGenerator(ego, main_trajectory);

              vector<State> next_states = behaviour.update(ego, vehicles, main_trajectory);
              for(const State& state : next_states)
              {
                  auto paths = path_gen.generatePaths(state, ego, main_trajectory, 0, 1, 1.0);
                  
                  for(auto& path: paths){
                    CostFunction cost_speed_fn = speedCostFunction;
                    double cost_speed = cost_speed_fn(ego, vehicles, path, state, 1.5);

                    CostFunction dist_cars_cost_fn = distanceToClosestCarAheadCostFunction;
                    double cost_dist_cars = dist_cars_cost_fn(ego, vehicles, path, state, 2.0);
                    
                    CostFunction future_dist_to_goal_cost_fn = futureDistanceToGoalCostFunction;
                    double future_dist_to_goal_cost = future_dist_to_goal_cost_fn(ego, vehicles, path, state, 1.0);

                    double final_cost = cost_speed + cost_dist_cars + future_dist_to_goal_cost;
                    cout << "* Generating paths for state: (" << state.s_state
                      << "," << state.d_state << ")\n" 
                      << "\t---> cost speed = " <<  cost_speed << "\n"
                      << "\t---> cost dist cars = " <<  cost_dist_cars << "\n"
                      << "\t---> cost future dist goal = " <<  future_dist_to_goal_cost << "\n"
                      << "\t---> FINAL COST = " <<  final_cost << "\n"
                      << endl;                     

                    if(final_cost < lowest_cost)
                    {
                      lowest_cost = final_cost;
                      chosen_trajectory = path;
                      chosen_state = state;
                    }
                }                
              }

              cout << "* FIRST TIME - Chosen state: (" << chosen_state.s_state
                   << "," << chosen_state.d_state << ")" << endl; 
              // Make sure to remove the first position, since the car is already "there"
              chosen_trajectory.removeFirstN(1);

              for(int i = 0; i < chosen_trajectory.xs.size(); ++i)
              {
                next_x_vals.push_back(chosen_trajectory.xs[i]);
                next_y_vals.push_back(chosen_trajectory.ys[i]);
              }
              main_trajectory = chosen_trajectory;
              
              behaviour.updateState(chosen_state);
              
              vector<double> a_a = main_trajectory.averageAcceleration(CONTROLLER_UPDATE_RATE_SECONDS);
              cout << "All trajectories added: size=" << main_trajectory.size() 
                   << " avg speed=" << main_trajectory.averageSpeed(CONTROLLER_UPDATE_RATE_SECONDS) << "mps" 
                   << " avg acc=(" << a_a[0] << ", " << a_a[1] << ")" << endl;


              cout << "*** Initial position=" << main_trajectory.ss[0] 
                   << " [26] Predicted position = " << main_trajectory.ss[26]
                   << " Predicted speed = " << main_trajectory.s_vels[26] << "mps" << endl;
            }
            else
            {

              cout << "\n\n******* UPDATING " << endl;
              // Get last point from trajectory
              // From it you get position, speed, acceleration
              int traj_size = main_trajectory.size();


              PathGenerator path_gen = PathGenerator(ego, main_trajectory);
              vector<State> next_states = behaviour.update(ego, vehicles, main_trajectory);
              int from_point = 25;              

              for(const State& state : next_states)
              {
                  
                  auto paths = path_gen.generatePaths(state, ego, main_trajectory, from_point, 1, 1.5);
                  
                  for(auto& path: paths){
                    CostFunction cost_speed_fn = speedCostFunction;
                    double cost_speed = cost_speed_fn(ego, vehicles, path, state, 1.0);

                    CostFunction dist_cars_cost_fn = distanceToClosestCarAheadCostFunction;
                    double cost_dist_cars = dist_cars_cost_fn(ego, vehicles, path, state, 2.0);
                    
                    CostFunction future_dist_to_goal_cost_fn = futureDistanceToGoalCostFunction;
                    double future_dist_to_goal_cost = future_dist_to_goal_cost_fn(ego, vehicles, path, state, 1.0);
                    
                    CostFunction speed_diff_to_car_ahead_fn = speedDifferenceWithClosestCarAheadCostFunction;
                    double speed_diff_to_car_ahead_cost = speed_diff_to_car_ahead_fn(ego, vehicles, path, state, 100.0);
                    
                    CostFunction collision_time_cost_fn = collisionTimeCostFunction;
                    double collision_time_cost = collision_time_cost_fn(ego, vehicles, path, state, 100.0);
                    
                    CostFunction dist_car_future_lane_cost_fn = distanceToClosestCarAheadFutureLaneCostFunction;
                    double dist_car_future_lane_cost = dist_car_future_lane_cost_fn(ego, vehicles, path, state, 100.0);

                    double final_cost = cost_speed + cost_dist_cars 
                                        + future_dist_to_goal_cost + speed_diff_to_car_ahead_cost
                                        + collision_time_cost + dist_car_future_lane_cost;
                    
                    // if(ego.lane != state.current_lane)
                    // {
                    //   cout << "*********************************************************************************" << endl;
                    //   cout << "*********************************************************************************" << endl;
                    //   cout << "*********************************************************************************" << endl;
                    //   cout << "*********************************************************************************" << endl;
                    //   cout << "*********************************************************************************" << endl;
                    //   exit(1);
                    // }

                    cout << "** Generating paths for state: (" << state.s_state
                      << "," << state.d_state << ")" 
                      << ", (current-lane=" << state.current_lane << ",future-lane=" << state.future_lane << ")\n" 
                      << "EGO LANE = " << ego.lane << "\n"
                      << "\t---> cost speed = " <<  cost_speed << "\n"
                      << "\t---> cost dist car same lane = " <<  cost_dist_cars << "\n"
                      << "\t---> cost dist car future lane = " <<  dist_car_future_lane_cost << "\n"
                      << "\t---> cost future dist goal = " <<  future_dist_to_goal_cost << "\n"
                      << "\t---> cost speed diff car ahead = " <<  speed_diff_to_car_ahead_cost << "\n"
                      << "\t---> cost collision ahead = " <<  collision_time_cost << "\n"
                      << "\t---> FINAL COST = " <<  final_cost << "\n"
                      << endl;                     

                    if(final_cost < lowest_cost)
                    {
                      lowest_cost = final_cost;
                      chosen_trajectory = path;
                      chosen_state = state;
                    }
                }                
              }
              cout << "* UPDATE - Chosen state: (" << chosen_state.s_state
                   << "," << chosen_state.d_state << ")" << endl; 


              main_trajectory = chosen_trajectory;              
                          
              for(int i = 0; i < chosen_trajectory.xs.size(); ++i)
              {
                next_x_vals.push_back(chosen_trajectory.xs[i]);
                next_y_vals.push_back(chosen_trajectory.ys[i]);
              }

              main_trajectory = chosen_trajectory;
              behaviour.updateState(chosen_state);

              vector<double> a_a = main_trajectory.averageAcceleration(0.02);
              cout << "(Update) All trajectories added: size=" << main_trajectory.size() 
                   << " avg speed=" << main_trajectory.averageSpeed(0.02) << "mps" 
                   << " avg acc=(" << a_a[0] << ", " << a_a[1] << ")" << endl;

              // for(int j = 0; j < main_trajectory.size(); ++j)
              // {
              //   cout << "[" << j << "]" 
              //        << " s-acc=" << main_trajectory.s_accs[j] 
              //        << " d-acc=" << main_trajectory.d_accs[j]
              //        << endl;
              // }
            }
            
            cout << "Size of path for controller = " << next_x_vals.size() << endl;
            

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
  // program
  // doesn't compile :-(
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
