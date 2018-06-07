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
#include "trajectory.h"
#include "collision_detector.h"

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
  }

  // This is the trajectory adopted by the car
  Trajectory main_trajectory;



  h.onMessage([&main_trajectory, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            if(!withinLane)
            {
              // cout << "Car lane is outside lane - expected lane " << myLane << endl;
            }
            else
            {
              // cout << "Car lane is " << myLane << endl;
              for(const Vehicle& v : vehicles)
              {
                vector<double> collision = cdetector.predictCollision(v, 0.02);
                // if(collision.size() > 0)
                // {
                //   cout 
                // }

                if(!v.isInLane)
                {
                  continue;
                }

                if(v.lane == myLane)
                {
                  
                  double dist = distance(car_x, car_y, v.x, v.y);
                  double s_diff = v.s - car_s;
                  cout << "Vehicle " << v.id << " is " << dist << "m (s-diff=" << s_diff
                       << ") away in the same lane " << endl;
                  if(car_s < v.s && s_diff < 25.0){
                    acc_sign = -1;
                    cout << "**** Will decelerate" << endl;
                  }
                }
              }

            }
            

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
						double car_yaw_rad = deg2rad(car_yaw);
						
            // double dist_inc = 0.5;
						// for(int i = 0; i < 50; ++i){
						// 	next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
						// 	next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
						// }            

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

            for(int i = 0; i < PATH_LENGTH && i < previous_path_x.size(); ++i)
            {              
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // int added_from_previous = 0;
            // if(previous_path_x.size() >= 2)
            // {
            //   for(int i = 0; i < 47 && i < previous_path_x.size(); ++i)
            //   {
            //     next_x_vals.push_back(previous_path_x[i]);
            //     next_y_vals.push_back(previous_path_y[i]);
            //   }
            //   added_from_previous = next_x_vals.size();
            // }

            double car_speed_mps = KmPerHourToMetersPerSecond(milesPerHourToKmPerHour(car_speed));
            // if(added_from_previous >= 2)
            // { 
            //   double prev_x = next_x_vals[added_from_previous - 1];
            //   double prev_y = next_y_vals[added_from_previous - 1];
            //   vector<double> previous_s_d = getFrenet(prev_x, prev_y, car_yaw_rad, map_waypoints_x, map_waypoints_y);

            //   double prev_prev_x = next_x_vals[added_from_previous - 2];
            //   double prev_prev_y = next_y_vals[added_from_previous - 2];
            //   double prev_prev_yaw = atan2(prev_y - prev_prev_y, prev_x - prev_prev_x);
            //   vector<double> prev_previous_s_d = getFrenet(prev_prev_x, prev_prev_y, prev_prev_yaw, map_waypoints_x, map_waypoints_y);

            //   double last_velocity = (previous_s_d[0] - prev_previous_s_d[0]) * 50;
            //   cout << "Last projected speed = " << last_velocity << endl;

            //   double final_speed = last_velocity < 2.78 ? 10 : last_velocity * 1.1;  
            //   if(final_speed > 22.0)
            //   {
            //     final_speed = 22.0;
            //   }              
 

            // }
            
            double final_speed = car_speed_mps < 2.78 ? 10 : car_speed_mps * 1.1;


            if(final_speed > 22.0)
            {
              final_speed = 22.0;
            }

            double acc_s = 0.0;
            if(previous_path_x.size() > 0)
            {
              cout << "Previous path length = " << previous_path_x.size() << endl;
              double last_point_x = previous_path_x[0];
              double last_point_y = previous_path_y[0];
              double previous_theta = atan2(car_y - last_point_y, car_x - last_point_x);
              // double previous_theta = atan2(last_point_y - car_y, last_point_x - car_x );

              vector<double> previous_s_d = getFrenet(last_point_x, last_point_y, previous_theta, map_waypoints_x, map_waypoints_y);
              acc_s = car_s - previous_s_d[0];
              cout << "** S acceleration = " << acc_s << endl;

              acc_s = acc_s <= 0 ? 0 : acc_s * 50;
            }

            double final_acc = 4;
            final_acc = acc_s;
            if(final_acc > 10){
              final_acc = 9.0;
            }
            cout << "Final speed = " << final_speed << endl;
            
            

            if(main_trajectory.size() == 0)
            {

              vector<double> start_s = {car_s, 0.0, 0.0};
              vector<double> end_s = {car_s + 8, 8, 8};

              vector<double> start_d = {car_d, 0.0, 0.0};
              vector<double> end_d = {car_d, 0, 0};

              vector<double> coeffs_s = JMT(start_s, end_s, MAX_MANEUVER_DURATION);
              vector<double> coeffs_d = JMT(start_d, end_d, MAX_MANEUVER_DURATION);

              cout << "\n\n******* COMPUTING " << endl;
              cout << "Start s = " << start_s[0] << ", end s = " << end_s[0] << " acc s=" << acc_s << endl;
              cout << "Car speed MPH=" << car_speed << " mps=" << car_speed_mps << " final speed=" << final_speed << endl;
              
              // main_trajectory.add(car_x, car_y, car_s, car_d, car_yaw_rad);

              for(int i = 0; i < PATH_LENGTH; ++i)
              {
                double t = 0.02 * (i + 1);
                double t_2 = pow(t, 2);
                double t_3 = pow(t, 3);
                double t_4 = pow(t, 4);
                double t_5 = pow(t, 5);

                // cout << "t2=" << t_2 << " t3=" << t_3 << " t4=" << t_4 << " t5=" << t_5 << endl;
                // cout << "a0=" << coeffs_s[0] << " a1=" << coeffs_s[1] << " a2=" << coeffs_s[2] << " a3=" << coeffs_s[3] 
                //      << " a4=" << coeffs_s[4] << " a5=" << coeffs_s[5] << endl;


                double s_t = start_s[0] + start_s[1] * t + 0.5 * start_s[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
                double s_t_dot = start_s[1] + start_s[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
                double s_t_dot_dot = start_s[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;

                double d_t = start_d[0] + start_d[1] * t + start_d[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
                double d_t_dot = start_d[1] + start_d[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
                double d_t_dot_dot = start_d[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
                


                cout << "s[" << i << "]: pos= " << s_t << " vel="<< s_t_dot << " acc=" << s_t_dot_dot << endl;                
                vector<double> x_y = getXY(s_t, d_t, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                if(abs(s_t_dot_dot) * 50.0 > 10.0 )        
                {
                  cout << "[WARNING] Acceleration greater than 10ms: " << s_t_dot_dot << endl;
                }


                // TODO FIX THIS
                double yaw = 0.0;
                main_trajectory.add(x_y[0], x_y[1], 
                                    s_t, s_t_dot, s_t_dot_dot, 
                                    d_t, d_t_dot, d_t_dot_dot, 
                                    yaw);
                next_x_vals.push_back(x_y[0]);
                next_y_vals.push_back(x_y[1]);
              }

              vector<double> a_a = main_trajectory.averageAcceleration(0.02);
              cout << "All trajectories added: size=" << main_trajectory.size() 
                   << " avg speed=" << main_trajectory.averageSpeed(0.02) << "mps" 
                   << " avg acc=(" << a_a[0] << ", " << a_a[1] << ")" << endl;
            }
            else
            {
              // Get last point from trajectory
              // From it you get position, speed, acceleration
              int traj_size = main_trajectory.size();


              vector<double> start_s = {main_trajectory.ss[traj_size - 1], 
                                        main_trajectory.s_vels[traj_size - 1],
                                        main_trajectory.s_accs[traj_size - 1]};

              double predicted_speed = start_s[1];
              cout << "* Predicted speed = " << predicted_speed << "mps" << endl;


              double desired_speed = acc_sign > 0 ? predicted_speed * (current_acceleration + 0.01) : predicted_speed * (0.97);
              cout << "* Desired speed = " << desired_speed << "mps" << endl;

              if(desired_speed > 22.0){
                desired_speed = 22.0;                        
              }else{
                current_acceleration = current_acceleration + acc_sign * 0.01;
                if(current_acceleration < 1.0){
                  current_acceleration = 1.0;
                }
              }

              // Determine final point, speed and acceleration
              vector<double> end_s = {start_s[0] + desired_speed, desired_speed, current_acceleration};

              // Keep the car on the lane for now
              vector<double> start_d = {main_trajectory.ds[traj_size - 1],
                                        main_trajectory.d_vels[traj_size - 1],
                                        main_trajectory.d_accs[traj_size - 1]};
              vector<double> end_d = {main_trajectory.ds[traj_size - 1], 0, 0};

              vector<double> coeffs_s = JMT(start_s, end_s, MAX_MANEUVER_DURATION);
              vector<double> coeffs_d = JMT(start_d, end_d, MAX_MANEUVER_DURATION);

              cout << "\n\n******* COMPUTING UPDATE POINTS **********" << endl;
              cout << "Start s = " << start_s[0] << ", end s = " << end_s[0] << " acc s=" << acc_s << endl;
              cout << "Car speed MPH=" << car_speed << " mps=" << car_speed_mps << endl;

              int points_missing = PATH_LENGTH - next_x_vals.size();
              for(int i = 0; i < points_missing; ++i)
              {
                double t = 0.02 * (i + 1);
                double t_2 = pow(t, 2);
                double t_3 = pow(t, 3);
                double t_4 = pow(t, 4);
                double t_5 = pow(t, 5);

                // cout << "t2=" << t_2 << " t3=" << t_3 << " t4=" << t_4 << " t5=" << t_5 << endl;
                // cout << "a0=" << coeffs_s[0] << " a1=" << coeffs_s[1] << " a2=" << coeffs_s[2] << " a3=" << coeffs_s[3] 
                //      << " a4=" << coeffs_s[4] << " a5=" << coeffs_s[5] << endl;


                double s_t = start_s[0] + start_s[1] * t + 0.5 * start_s[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
                double s_t_dot = start_s[1] + start_s[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
                double s_t_dot_dot = start_s[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;

                double d_t = start_d[0] + start_d[1] * t + start_d[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
                double d_t_dot = start_d[1] + start_d[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
                double d_t_dot_dot = start_d[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
                


                // cout << "(Updated) s[" << i << "]: pos= " << s_t << " vel="<< s_t_dot << " acc=" << s_t_dot_dot << endl;                
                // cout << "(Updated) d[" << i << "]: pos= " << d_t << " vel="<< d_t_dot << " acc=" << d_t_dot_dot << endl;                
                vector<double> x_y = getXY(s_t, d_t, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                               


                // TODO FIX THIS
                double yaw = 0.0;
                main_trajectory.add(x_y[0], x_y[1], 
                                    s_t, s_t_dot, s_t_dot_dot, 
                                    d_t, d_t_dot, d_t_dot_dot, 
                                    yaw);
                next_x_vals.push_back(x_y[0]);
                next_y_vals.push_back(x_y[1]);
              }

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
            


            // cout << "Length (1) = " << next_x_vals.size() << endl;
            // int previous_path_length = previous_path_x.size();
            // for(int i = 0; i < PREVIOUS_PATH_POINTS_TO_KEEP && i < previous_path_length; ++i)
            // {
            //   next_x_vals.push_back(previous_path_x[i]);
            //   next_y_vals.push_back(previous_path_y[i]);
            // }

            
            // double pos_x = car_x;
            // double pos_y = car_y;
            // double angle = deg2rad(car_yaw);
            // int filled_points_length = next_x_vals.size();
            // cout << "Length (2) = " << next_x_vals.size() << endl;
            // if(filled_points_length > 1){
            //   pos_x = next_x_vals[filled_points_length - 1];
            //   pos_y = next_y_vals[filled_points_length - 1];

            //   double pos_x2 = previous_path_x[filled_points_length-2];
            //   double pos_y2 = previous_path_y[filled_points_length-2];
            //   angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            // }

            // vector<double> pos_s_d = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
            // double pos_s = pos_s_d[0];
            // double pos_d = pos_s_d[1];
            // double s_inc = 0.2;
            // for(int i = 0; i < PATH_LENGTH - filled_points_length; ++i)
            // {
            //   auto xy = getXY(pos_s + s_inc * (i+1), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   next_x_vals.push_back(xy[0]);
            //   next_y_vals.push_back(xy[1]);
            // }

            cout << "Length (3) = " << next_x_vals.size() << endl;
            
            // for(int i = 0; i < 50; ++i){
            //   auto xy = getXY(car_s + s_inc * (i+1), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   next_x_vals.push_back(xy[0]);
            //   next_y_vals.push_back(xy[1]);
            // }


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
