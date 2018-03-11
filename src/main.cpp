#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// set global variables
double last_v_error = 0;
double target_speed = 0;
int lane = 1; // 0=left, 1=middle, 2=right

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO:
            double update_time = 0.02; // one step every 20ms
            double total_time = 1.0; // total time for path is 1s
            int no_steps = int(total_time/update_time); // number of total steps
            double speed_limit = 49.2; // set speed limit to slightly below 50 mph
            int max_map_points = 181; // maximum amount of map points before it wraps around (back to waypoint 1)
            double car_detection_distance = 50; // distance in front of car where vehicles are recognized
            double safety_distance = 30; // car tries to hold that distance to cars in front of it
            int no_of_lanes = 3; // 0=left, 1=middle, 2=right
            int rightmost_lane = no_of_lanes-1; // 2=right
            int leftmost_lane = 0; // 0=left

            int prev_path_size = previous_path_y.size();

            // look for other cars
            bool obstacle_front = false;
            bool obstacle_right = false;
            bool obstacle_left = false;
            double obstacle_speed;
            double obstacle_distance = car_detection_distance;
            double lane_change_limit_front = 35;
            double lane_change_limit_back = 10;
            for (int i=0; i<sensor_fusion.size(); i++)
            {
              // read in car data from sensor fusion data
              double other_car_vx = sensor_fusion[i][3];
              double other_car_vy = sensor_fusion[i][4];
              double other_car_s = sensor_fusion[i][5];
              double other_car_d = sensor_fusion[i][6];
              double other_car_speed = sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
              double distance_to_car = other_car_s - car_s;
              double other_car_lane = floor(other_car_d/4);

              // currently observed car is in front of our car, is in the same lane, and is the closest one
              if (distance_to_car > 0 and distance_to_car < car_detection_distance and distance_to_car < obstacle_distance and other_car_lane == lane)
              {
                // set flag, save speed and distance
                obstacle_front = true;
                obstacle_speed = other_car_speed * 2.23694;
                obstacle_distance = distance_to_car;
              }

              // currently observed car is in the lane to the right and in a certain distance (unsafe)
              if (lane != rightmost_lane and other_car_lane == lane + 1 and car_s-lane_change_limit_back < other_car_s and other_car_s < car_s+lane_change_limit_front)
              {
                // set flag
                obstacle_right = true;
              }

              // currently observed car is in the lane to the left and in a certain distance (unsafe)
              if (lane != leftmost_lane and other_car_lane == lane -1 and car_s-lane_change_limit_back < other_car_s and other_car_s < car_s+lane_change_limit_front)
              {
                // set flag
                obstacle_left = true;
              }


            }

            // calculate error value
            double distance_weight = 0.4;
            double speed_weight = 1.2;
            double v_error, v_error_diff, delta_t;

            // obstacle in current lane detected
            if (obstacle_front == true)
            {
              if (lane == leftmost_lane) // car is in leftmost lane
              {
                if (obstacle_right == false) // right lane has space
                {
                  lane ++; // change one lane to the right
                }
              }
              else if (lane == rightmost_lane) // car is in rightmost lane
              {
                if (obstacle_left == false) // left lane has space
                {
                  lane --; // change one lane to the left
                }
              }
              else // car is in a middle lane
              {
                if (obstacle_left == false) // left lane has space
                {
                  lane --; // change one lane to the left
                }
                else if (obstacle_right == false) // right lane has space
                {  
                  lane ++; // change one lane to the right
                }
              }


              v_error = speed_weight * (obstacle_speed - car_speed) + distance_weight * (obstacle_distance - safety_distance);
              //v_error = speed_limit-car_speed;
            }
            else
            {
              v_error = speed_limit-car_speed;
            }

            // calculate time difference between last step and current step
            delta_t = (prev_path_size == 0) ? 0 : ((no_steps-prev_path_size)*update_time);
            // calculate differential speed error
            v_error_diff = (delta_t > 0) ? ((v_error-last_v_error)/delta_t) : 0;
            last_v_error = v_error;

            // apply PD control
            double k_p = 0.008;
            double k_d = 0.006;
            target_speed += (k_p * v_error + k_d * v_error_diff) * 0.44704; // target speed in m/s

            // set the target distance between points to achieve the target speed
            double distance_inc = target_speed/no_steps; 

            // fit a spline to the next x points with a defined distance between each point
            vector<double> spline_x, spline_y; // vector for point values
            int spline_length = 3; // fit spline to that many points
            double spline_s = 35;  //set standard distance between those points
            int car_lane = floor(car_d/4);
            if (car_lane != lane)
            {
              cout << "lane change" << endl;
              spline_s = 55; // less force to the side
            }

            // variables for the current values of the car or path
            double current_x, current_y, current_yaw;
            // previous states of the car
            double previous_car_x, previous_car_y;

            // if the previous path is almost empty, us the car's position as starting point and the position before to make sure the path is tangent
            if (prev_path_size < 2)
            {
              current_x = car_x;
              current_y = car_y;
              current_yaw = deg2rad(car_yaw);

              previous_car_x = car_x - distance_inc * cos(car_yaw);
              previous_car_y = car_y - distance_inc * sin(car_yaw);
            }
            // otherwise use the last elements of the previously generated path
            else
            {
              current_x = previous_path_x[prev_path_size-1];
              current_y = previous_path_y[prev_path_size-1];

              previous_car_x = previous_path_x[prev_path_size-2];
              previous_car_y = previous_path_y[prev_path_size-2];

              current_yaw = atan2(current_y - previous_car_y, current_x - previous_car_x); // needed for transformation
            }

            // save the last two points as the start of the spline
            spline_x.push_back(previous_car_x);
            spline_x.push_back(current_x);
               
            spline_y.push_back(previous_car_y);
            spline_y.push_back(current_y);

            // create the other reference points for the spline
            for (int i=1; i<spline_length+1; i++)
            {
              vector<double> spline_x_y = getXY(car_s+spline_s*i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              spline_x.push_back(spline_x_y[0]);
              spline_y.push_back(spline_x_y[1]);
            }

            // change (shift) the x-y coordinate system to the car's perspective (helps to avoid multiple splines as space curves)
            for (int i=0; i < spline_x.size(); i++)
            {
              double shift_x = spline_x[i] - current_x;
              double shift_y = spline_y[i] - current_y;

              double hypotenuse = sqrt(shift_x*shift_x+shift_y*shift_y);
              double alpha = atan2(shift_y,shift_x);
              double beta = alpha - current_yaw;

              spline_x[i] = hypotenuse * cos(beta);
              spline_y[i] = hypotenuse * sin(beta);
            }

            // create the spline and set the generated points
            tk::spline s;
            s.set_points(spline_x, spline_y);

            // load the previous path from last update cycle
            for (int i=0; i<previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // linearize the spline to devide it into increments
            double linearize_x = 30;
            double x_pos, y_pos, x_map, y_map;
            double linearize_dist = linearize_x / (distance(0, 0, linearize_x, s(linearize_x)) / distance_inc);

            // calculate points on spline from linearization
            for (int i=1; i<=no_steps-previous_path_x.size(); i++)
            {
              x_pos = linearize_dist * i;
              y_pos = s(x_pos);

              double beta = atan2(y_pos, x_pos);
              double alpha = beta + current_yaw;
              double hypotenuse = sqrt(x_pos*x_pos+y_pos*y_pos);

              x_map = current_x + hypotenuse * cos(alpha);
              y_map = current_y + hypotenuse * sin(alpha);

              // transform back to global coordinate system
              x_pos = x_pos*cos(current_yaw)-y_pos*sin(current_yaw) + current_x;
              y_pos = x_pos*sin(current_yaw)+y_pos*cos(current_yaw) + current_y;


              // add values to vector
              next_x_vals.push_back(x_map);
              next_y_vals.push_back(y_map);
            }

            // send x and y values to simulator 
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
