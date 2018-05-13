#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "Executor.h"
#include "helper.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;
using namespace helper;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.

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

double target_t = 0;
double target_s = 0;
double target_sdot = 0;
double target_sddot = 0;
double set_target = false;
double reached_target = false;
auto last_t = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
).count();
double last_car_speed = 0;
vector<double> s_coef;



int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Executor exector;
  exector.Init();
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
            auto ms = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
            ).count();
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            auto dt = (duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
            ).count() - last_t) / 1000.0;

            double car_a = (car_speed - last_car_speed) / (dt + 0.000001);

            last_t = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
            ).count();
            last_car_speed = car_speed;

            if (set_target == false){
                set_target = true;
                target_s = car_s + 100;
                target_sdot = 20.0;
                target_sddot = 0.0;
                reached_target = false;
                target_t = duration_cast<milliseconds>(
                    system_clock::now().time_since_epoch()
                ).count()/1000.0 + 10;
                dt = 0;
                s_coef = solve_jerm_minimizing_coefs(
                    {car_s, car_speed, car_a},
                    {target_s, target_sdot, target_sddot},
                    target_t - ms/1000.0
                );

            } else if (car_s > target_s){

            }

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


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            cout << ms << ", " <<  target_t - ms/1000.0 << ", " << dt << ", " << car_s << ", " << car_speed << ", " << car_a <<
                ", " << target_s << ", " << target_sdot << ", " << target_sddot << endl;
            cout << end_path_s << ", " << end_path_d << endl;


            for( int i = 0; i < s_coef.size(); i++){
                cout << s_coef[i] << ", ";
            }
            cout << endl;

            auto t_past = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
            ).count()/1000.0 - (target_t - 10);
            for(int i = 0; i < 50; i++)
            {
                double s = evaluate(s_coef, (i+1) * 0.02 + t_past);
                cout << s << ", ";
                double d = 6;
                vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                next_x_vals.push_back(xy[0]);
                next_y_vals.push_back(xy[1]);
            }
            cout << endl;



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
