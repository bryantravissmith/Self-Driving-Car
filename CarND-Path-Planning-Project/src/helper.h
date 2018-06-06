#ifndef HELPER
#define HELPER

#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "config.h"
#include "spline.h"

using namespace std;
using namespace Eigen;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinate
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
// Modified to use use splines/smoothing of map instead of linearized geometry
vector<double> getXY(double s, double d, const vector<double> &maps_s, tk::spline map_x_spline, tk::spline map_y_spline) //const vector<double> &maps_x, const vector<double> &maps_y)
{
    double sp = fmod(s, TRACK_LENGTH);
	double seg_x = map_x_spline(sp);
	double seg_y = map_y_spline(sp);

    double seg_dx = seg_x - map_x_spline(sp-1.0);
    double seg_dy = seg_y - map_y_spline(sp-1.0);
    double heading = atan2(seg_dy, seg_dx);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double evaluate(vector<double> coefs, double t){
    double value = 0;
    for(int i = 0; i < coefs.size(); i++){
        value += coefs[i] * pow(t, i);
    }
    return value;
}

vector<double> solve_jerm_minimizing_coefs(vector<double> start, vector<double> end, double t){
    MatrixXd A(3, 3);
    A << pow(t, 3), pow(t, 4), pow(t, 5),
        3*pow(t, 2), 4*pow(t, 3), 5 * pow(t, 4),
        6*t, 12*pow(t, 2), 20*pow(t, 3);

    MatrixXd b = MatrixXd(3,1);
    b << end[0] - (start[0] + start[1]*t + start[2] * pow(t, 2) / 2.0),
        end[1] - (start[1] + 2*start[2]*t),
        end[2] - 2*start[2];

    auto AInv = A.inverse();
    auto solution = AInv * b;
    return {
        start[0], start[1], start[2]/2,
        solution(0, 0), solution(1, 0), solution(2, 0)
    };
}


int get_index_closest_vehicle_same_lane(double car_s, double car_d, vector<vector<double>> sensor_fusion){
    int car_lane = (int) round( (car_d - 2.0) / 4.0 );
    double closest_distance = TRACK_LENGTH;
    int closest_index = -1;

    for(int i = 0; i < sensor_fusion.size(); i++){
        int other_lane = (int) round( (sensor_fusion[i][6] - 2.0) / 4.0 );
        if(other_lane == car_lane && sensor_fusion[i][5] > car_s){
            double distance = sensor_fusion[i][5] - car_s;
            if(distance < closest_distance){
                closest_distance = distance;
                closest_index = i;
            }
        }
    }
    return closest_index;
}

vector<bool> get_vehicle_state(double car_s, double car_d, vector<vector<double>> sensor_fusion){
    bool car_left = false;
    bool car_right = false;
    bool car_ahead = false;
    bool car_ahead_close = false;

    int car_lane = (int) round( (car_d - 2.0) / 4.0 );
    if(car_lane < 1){
        car_left = true;
    }
    if(car_lane > 1){
        car_right = true;
    }
    for(int i = 0; i < sensor_fusion.size(); i++){
        int other_lane = (int) round( (sensor_fusion[i][6] - 2.0) / 4.0 );
        if(other_lane == car_lane){
            if(sensor_fusion[i][5] < car_s + 1.5*FOLLOW_DISTANCE*2 && sensor_fusion[i][5] > car_s){
                car_ahead = true;
            }
            if(sensor_fusion[i][5] < car_s + 1.5*FOLLOW_DISTANCE && sensor_fusion[i][5] > car_s){
                car_ahead_close = true;
            }
        }
        if(other_lane - car_lane == 1){
            if(sensor_fusion[i][5] < car_s + 1.55*FOLLOW_DISTANCE && sensor_fusion[i][5] > car_s - 0.5*FOLLOW_DISTANCE){
                car_right = true;
            }
        }
        if(other_lane - car_lane == -1){
            if(sensor_fusion[i][5] < car_s + 1.55*FOLLOW_DISTANCE && sensor_fusion[i][5] > car_s - 0.5*FOLLOW_DISTANCE){
                car_left = true;
            }
        }
    }
    return {car_left, car_right, car_ahead, car_ahead_close};
}

#endif
