#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::sqrt;
using std::round;
using std::exp;

using std::cout;
using std::endl;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          /**
           * Step 1: Find and evaluate obstacles in our path to choose the
           *   fastest lane to travel in.
           */
          car_speed /= 2.24; 		// stupid unit conversion

          // Initialize car position and velocity, and some road info
          double SPEED_LIMIT = 22.0;
          vector<int> lanes{0, 1, 2}; // 0 = left lane, 1 = middle, 2 = right
          int num_lanes = lanes.size();
          double ref_speed = SPEED_LIMIT;	// target speed (meters/second)
          double delta_t = 0.02;			// time step (seconds)

          // Sensing parameters
          double sense_limit = 150;			// ignore objects beyond this distance
          double care_limit = 50; 			// do not take action beyond this dist
					double look_forward = 11;			// dist ahead to split "weigh" vs "don't hit"
          double safe_gap = 20;					// space req'd to fit the car safely
          double clearance = 8.0;
          double rear_clearance = 7.0;
          double high_velocity = 1000;	// "clear road" allows infinite speed

          // Weighting parameters
          double app_offset = 9.0;			// "tailgating" closeness
          double follow_comp = 9.0;			// following the "reported" speed of an
          															// object in front never works
          															// compensate extra to prevent crashes
          double pocket_comp = 8.0;			// when triggered, drops speed further
          															// to "unstuck" the car from a pocket
          															// in traffic
          double dist_weight = 1;				// relative importance, dist vs speed
          double speed_weight = 1.5;		// relative importance, speed vs dist
          double stay_weight = 0.15;		// amount of bias for staying in lane
          double block_thres = 2.0;			// overlap criteria for "lane blocked"
          double block_weight = 50;			// amount of bias against blocked lanes

        	// Map out objects around us - by lane
          // Distances of objects in front of us and close/rear to us
          vector<double> near_obj_front(num_lanes, sense_limit);
          vector<double> near_obj_rear(num_lanes, -sense_limit);
          // Speeds of objects in front of us and close/rear to us
          vector<double> v_obj_front(num_lanes, high_velocity);
          vector<double> v_obj_rear(num_lanes, high_velocity);

          // Check each object caught by sensor fusion
          bool take_action = false;
          for (int i = 0; i < sensor_fusion.size(); i++) {
          	double obj_s = sensor_fusion[i][5];
        		double obj_dist = obj_s - car_s;

        		// Calculate which lane  the object is in
        		double obj_d = sensor_fusion[i][6];
        		for (int l : lanes) {
        			if (obj_d > 4*l && obj_d <= 4+4*l) {

            		double vx = sensor_fusion[l][3];
      					double vy = sensor_fusion[l][4];
              	// Log speed and position of closest object ahead of us
              	if (obj_s > car_s + clearance &&
              			obj_dist < near_obj_front[l]) {
        					v_obj_front[l] = sqrt(vx*vx + vy*vy);
        					near_obj_front[l] = obj_dist;

        					// Flag for action if object is close to us
            			if (obj_dist < care_limit)
            				take_action = true;
              	}
              	// Log speed and position of closest object behind/beside us
              	// Include overlap (extra 3m) to double-count objects near
              	// our front corner
              	if (obj_s < car_s + look_forward &&
              			obj_dist > near_obj_rear[l]) {
        					v_obj_rear[l] = sqrt(vx*vx + vy*vy);
              		near_obj_rear[l] = obj_dist;
              	}
        			}
        		}
          }

          // Diagnostic outputs
          cout << "---------------------------------------------------" << endl;
          cout << "Old car_speed = " << car_speed << endl;
          cout << "    Nearest object (front) distances: ";
          for (double dist : near_obj_front) { cout << dist << ", "; }
          cout << endl << "    Nearest object (front) speeds: ";
          for (double v : v_obj_front) { cout << v << ", "; }
          cout << endl << "    Nearest object (rear) distances: ";
          for (double dist : near_obj_rear) { cout << dist << ", "; }
          cout << endl << "    Nearest object (rear) speeds: ";
          for (double v : v_obj_rear) { cout << v << ", "; }
          cout << endl;

          // Set default lane target before evaluating lane changes
          int old_lane = round((car_d - 2) / 4);
          int lane = old_lane;
          int next_lane;

					// Calculate proximity and adjust speed behind obstructions
					double approach_dist, follow_speed;
					if (near_obj_rear[old_lane] > 0) {
						approach_dist = near_obj_rear[old_lane];
						follow_speed = v_obj_rear[old_lane];
					}
					else {
						approach_dist = near_obj_front[old_lane];
						follow_speed = v_obj_front[old_lane];
					}
					ref_speed -= (ref_speed - follow_speed + follow_comp)
											 / (1+exp(approach_dist - app_offset));

					cout << "ref_speed modified: " << ref_speed << endl;

  				// Take action when obstacles are close enough to be of concern
  				if (take_action) {

          	// Calculate relative desirability of lane options
            vector<double> weighted_pref;
          	double weight;
            for (int l : lanes) {
            	// Linear combo of speed and distance of the object in front.
            	// Increased if the lane is the current lane (lazy bias);
            	// reduced if the lane is blocked by an object.
            	weight = (near_obj_front[l] * dist_weight +
												v_obj_front[l] * speed_weight) /
												(1 - (l == old_lane) * stay_weight +
												(near_obj_rear[l] > -block_thres) * block_weight);

            	weighted_pref.push_back(weight);
            }

          	// Calculate the best lane option
            double best = 0.0;
          	for (int l : lanes) {
          		cout << "    weighted_pref[" << l << "] = " << weighted_pref[l] << "; "
          				 << "    weighted_pref[old] = " << weighted_pref[old_lane] << endl;

        			if (weighted_pref[l] > best) {
        				best = weighted_pref[l];

        				// Only allow 1 lane change at a time
        				next_lane = (l > old_lane) - (l < old_lane) + old_lane;
        			}
          	}

            cout << "old_lane: " << old_lane << ", lane wanted: " << next_lane << endl;

    				// Assess gaps between objects to pass
    				double gap = approach_dist - near_obj_front[next_lane];
    				if (gap > safe_gap) {

    					cout << "  object next lane (front): " << near_obj_front[next_lane] << endl;

    					if (near_obj_front[next_lane] > safe_gap)
    						lane = next_lane; // take the earlier lane change for safety
    					else
    						lane = old_lane;	// no immediate space, wait for gap
    				}
  					else {
  						// Calculate how much distance we are obstructed by (need to move
  						// around)
  						double obstruction = approach_dist - near_obj_rear[next_lane];

  						cout << "  obstruction amount: " << obstruction << endl;

  						if (near_obj_rear[next_lane] < clearance &&
									obstruction < clearance + rear_clearance &&
  								obstruction > 0)
  							ref_speed -= pocket_comp;	// stuck in a pocket, slow down
  						else
  							lane = next_lane;		// space behind object but no gap in front
  					}

    				// Final check - abort lane change if we do not have clearance
    				if (near_obj_rear[next_lane] < clearance && 
								near_obj_rear[next_lane] > -rear_clearance) {

    					cout << "Cannot change lanes due to clearance" << endl;

    					lane = old_lane;
    				}

          	cout << "New lane = " << lane << endl;
          }


          /**
           * Step 2: with the lane preference determined, define a path of
           *   (x,y) points for the car to visit per .02 seconds
           */
          vector<double> next_x_vals;		// Containers to pass in msg
          vector<double> next_y_vals;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Adjust speed based on the state of the car vs. target reference
          if (car_speed > SPEED_LIMIT) {
            car_speed = SPEED_LIMIT;
            cout << "\t\t\t SPEED LIMIT BROKEN" << endl;
          }
          else
            car_speed += 0.3 * ((car_speed < ref_speed - .3)
          										 - (car_speed > ref_speed));

					cout << "New car_speed = " << car_speed << endl;

          // Generate a spline for the new path
          vector<double> splinepts_x;
          vector<double> splinepts_y;

        	// Spline must be tangent to current travel path
          if (previous_path_x.size() < 5) {
          	// If there are not enough previous path points, we generate a
          	// "virtual" one based on current heading
          	splinepts_x.push_back(ref_x - cos(ref_yaw));
          	splinepts_y.push_back(ref_y - sin(ref_yaw));

          	// The car's current position becomes the second spline point
          	splinepts_x.push_back(ref_x);
          	splinepts_y.push_back(ref_y);
          }
          else {
          	// If there are points left over from the previous path made,
          	// keep the immediately upcoming 3 points in the new path
          	// and reset our reference x and y
          	ref_x = previous_path_x[1];
          	ref_y = previous_path_y[1];
          	double ref_x_prev = previous_path_x[0];
          	double ref_y_prev = previous_path_y[0];

            //next_x_vals.push_back(previous_path_x[0]);
            //next_y_vals.push_back(previous_path_y[0]);
          	next_x_vals.push_back(ref_x_prev);
          	next_y_vals.push_back(ref_y_prev);
          	next_x_vals.push_back(ref_x);
          	next_y_vals.push_back(ref_y);

          	// We will use these two points as the start for the new spline
          	splinepts_x.push_back(ref_x_prev);
          	splinepts_y.push_back(ref_y_prev);
          	splinepts_x.push_back(ref_x);
          	splinepts_y.push_back(ref_y);
          	splinepts_x.push_back(previous_path_x[4]);
          	splinepts_y.push_back(previous_path_y[4]);

          	// Recompute car's heading
          	ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          // Generate 4th, 5th, 6th points to draw the spline through
          vector<double> splinept3 = getXY(car_s+40, 2+4*lane,
          									map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> splinept4 = getXY(car_s+75, 2+4*lane,
          									map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> splinept5 = getXY(car_s+90, 2+4*lane,
          									map_waypoints_s, map_waypoints_x, map_waypoints_y);

          splinepts_x.push_back(splinept3[0]);
          splinepts_y.push_back(splinept3[1]);

          splinepts_x.push_back(splinept4[0]);
          splinepts_y.push_back(splinept4[1]);

          splinepts_x.push_back(splinept5[0]);
          splinepts_y.push_back(splinept5[1]);

          // Transform spline points from map coords to vehicle coords, aka make
          // x0 = 0, y0 = 0, yaw0 = 0
          for (int i = 0; i < splinepts_x.size(); i++) {
          	double shift_x = splinepts_x[i] - ref_x;
          	double shift_y = splinepts_y[i] - ref_y;

          	splinepts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
          	splinepts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // Create spline
          tk::spline s;
          s.set_points(splinepts_x, splinepts_y);

          // Calculate x-spacing along the spline to generate path points
          double horizon_x = 10;
          double horizon_y = s(horizon_x);
          double horizon_dist = sqrt(horizon_x*horizon_x + horizon_y*horizon_y);
        	double spacing = horizon_x / (horizon_dist / (delta_t * car_speed));

        	double spline_x, spline_y;
          for (spline_x = spacing; spline_x < horizon_x; spline_x += spacing) {
          	// Complete each x,y point along the spline
          	spline_y = s(spline_x);

          	// Transform back from vehicle coordinates to map coordinates
          	double x_next = (spline_x*cos(ref_yaw) - spline_y*sin(ref_yaw))
          								  + ref_x;
          	double y_next = (spline_x*sin(ref_yaw) + spline_y*cos(ref_yaw))
          									+ ref_y;

          	// Push new path points to path
          	next_x_vals.push_back(x_next);
          	next_y_vals.push_back(y_next);

          }		  
          // END


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
