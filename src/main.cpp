#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

            //starting lane
          int lane = 1;
          
          // reference velocity mph at 0 so that it speeds up at line 161 to 49,5 mph
          double ref_v = 0.0 ;
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_v]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // previous path size 
          int prev_path_size = previous_path_x.size();
                   

          
          

          
          //A list of waypoints that ara further apart to then smoothly integrate multiple waypoints in between later
		  vector <double> ptsx;
          vector <double> ptsy;
          
          // are we too close to another car?
          bool too_close = false;
          
          // referencing the starting point or the end of the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //to only consider safe lanes for lane changes
          bool is_lane_1_safe = true;
          bool is_lane_2_safe = true;
          bool is_lane_3_safe = true;
          
          double min_dist_1 = 10000000.0;
          double min_dist_2 = 10000000.0;
          double min_dist_3 = 10000000.0;
          
          int closest_id_1 = 0;
          int closest_id_2 = 0;
          int closest_id_3 = 0;
          
          double speed_lane_1 = 0.0;
          double speed_lane_2 = 0.0;
          double speed_lane_3 = 0.0;
          
          int best_lane = 1;
          
          // take the end of s for our car position if we have previous points in the pipeline
            if (prev_path_size > 0) {
              car_s = end_path_s;
            }
          
          
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            //d value of cars
            float d = sensor_fusion[i][6];
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            

              //if there are previous points in the pipeline we need to project into the future
              check_car_s += ((double)prev_path_size * 0.02 * check_speed);
            
            
            double car_diff = car_s - check_car_s;
            
            
            //begin is lane safe check
            //lane1
            if (d >= 0 && d <= 4)
            {
              if (-15.0 <= car_diff && car_diff <= 20.0 && lane!=0)
              {
                is_lane_1_safe = false;
              }
              
              if (car_diff <= 0.0)
              {
                if ((0.0 - car_diff) < min_dist_1)
                {
                 min_dist_1 = 0.0 - car_diff;
                 closest_id_1 = i;
                 speed_lane_1 = check_speed;
                }
              }
              
            }
            
            //lane2
            if (d > 4 && d <= 8)
            {
              if (-15.0 <= car_diff && car_diff <= 20.0 && lane!=1)
              {
                is_lane_2_safe = false;
              }
              
               if (car_diff <= 0.0)
              {
                if ((0.0 - car_diff) < min_dist_2)
                {
                 min_dist_2 = 0.0 - car_diff;
                 closest_id_2 = i;
                 speed_lane_2 = check_speed;
                }
              }
            }
            
            //lane3
            if (d > 8 && d <= 12)
            {
              if (-15.0 <= car_diff && car_diff <= 20.0 && lane!=2)
              {
                is_lane_3_safe = false;
              }
               if (car_diff <= 0.0)
              {
                if ((0.0 - car_diff) < min_dist_3)
                {
                 min_dist_3 = 0.0 - car_diff;
                 closest_id_3 = i;
                 speed_lane_3 = check_speed;
                }
              }
            }
            //end is lane safe check
            
        
            
            // check if car is in my lane
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
            {
              //double vx = sensor_fusion[i][3];
              //double vy = sensor_fusion[i][4];
              //double check_speed = sqrt(vx * vx + vy * vy);
              //double check_car_s = sensor_fusion[i][5];
              
              //if there are previous points in the pipeline we need to project into the future
              //check_car_s += ((double)prev_path_size * 0.02 * check_speed);
              

              
              
              // if the car is in front of me and within a certain distance (30)
              if ((check_car_s > car_s) && (check_car_s - car_s < 30))
              {
     
                too_close = true;
                
               
                
                
              }
            }
          }
          //accelerate and decelerate within the restrictions
          
          if ((speed_lane_1 > speed_lane_2 && speed_lane_1 > speed_lane_3) || min_dist_1 > 500)
          {
            best_lane = 0;
          }
          
          if ((speed_lane_2 > speed_lane_1 && speed_lane_2 > speed_lane_3) || min_dist_2 > 500)
          {
            best_lane = 1;
          }
          
          if ((speed_lane_3 > speed_lane_1 && speed_lane_3 > speed_lane_2) || min_dist_3 > 500)
          {
            best_lane = 2;
          }
          
          if (is_lane_3_safe && lane!=2 && best_lane ==3 && too_close)
          {
            lane=2; 
          }

          if (is_lane_2_safe && lane!=1 && best_lane ==2 && too_close)
          {
            lane=1;                
          }

          if (is_lane_1_safe && lane!=0 && best_lane ==1 && too_close)
          {
            lane=0; 
          }          
          
          if (too_close)
          {
            ref_v -= 0.224;
          }
          else if (ref_v < 49.5)
          {
           ref_v += 0.224; 
          }
             
         
          
          
          if (prev_path_size < 2)
          {
            //create a path tangent to the current car pose with 2 adding 2 points
            // cars yaw stays the same as the current --> no need to change
          	double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
                                            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
       		
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
          	// use the previously stacked up path and continue using the last two points of that path in order to be tangential
            // yaw needs to be calculated in the future, because we can't take the current yaw
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];
            
            double prev_ref_x = previous_path_x[prev_path_size-2];
            double prev_ref_y = previous_path_y[prev_path_size-2];
            
            ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
       		
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);            
          }
          
          //Using the above created starting reference we push back broadly evenly spread points to create a smooth splie
          //Using frenet coordinates we need to transform these back to XY
          
          vector <double> next_wp_1= getXY(car_s+30,(lane*4+2),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector <double> next_wp_2= getXY(car_s+60,(lane*4+2),map_waypoints_s,map_waypoints_x,map_waypoints_y);          
          vector <double> next_wp_3= getXY(car_s+90,(lane*4+2),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp_1[0]);
          ptsx.push_back(next_wp_2[0]);          
          ptsx.push_back(next_wp_3[0]);  

          ptsy.push_back(next_wp_1[1]);          
          ptsy.push_back(next_wp_2[1]);
          ptsy.push_back(next_wp_3[1]);
          
          //shift to cars local coordinates for the angle to be 0
          
          for (int i = 0; i < ptsx.size(); i++)
          {
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw) - (shift_y * sin(0-ref_yaw)));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + (shift_y * cos(0-ref_yaw)));
          }
   
          // create spline object using the spline.h library
          tk::spline spl;

          spl.set_points(ptsx,ptsy);
          

    
          // add the previous points that have not yet been used to the next planned points of the car
          for (int i = 0; i < previous_path_x.size(); i++)
          {
           next_x_vals.push_back(previous_path_x[i]);
           next_y_vals.push_back(previous_path_y[i]);
          }
          
          //break up spline into small increments so that we travel at the desired velocity and feed them in to the next planned points
          double target_x = 30.0;
          double target_y = spl(target_x);
          double target_distance = sqrt((target_x * target_x) + (target_y * target_y));
          double x_addon = 0.0;
          
          for (int i = 0; i <= (50 - previous_path_x.size()); i++)
          {
            // finding N distance using the 2.24 to turn to m/s
            double N = (target_distance / (.02 * ref_v / 2.24));
            double x_point = x_addon + (target_x / N);
            double y_point = spl(x_point);
            
            x_addon = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //shift back to global coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
          
          
          
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