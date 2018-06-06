#include "Executor.h"
#include "spline.h"
#include "helper.h"
#include "cost.h"
#include "config.h"


using namespace std;
using namespace std::chrono;
using json = nlohmann::json;

Executor::Executor() {}

Executor::~Executor() {}

void Executor::Init() {
    this->target_d = 6.0;
    this->setting_d = 6.0;
}

void Executor::UpdatePlan(vector<double> s_coefs, vector<double> d_coefs){
    this->plan_trajectory_s_coefs = s_coefs;
    this->plan_trajectory_d_coefs = d_coefs;
}

void Executor::SetTrajectory(
    vector<double>& next_x_vals, vector<double>& next_y_vals, json car_data,
    const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y
){
    // Main car's localization Data
    double car_x = car_data["x"];
    double car_y = car_data["y"];
    double car_s = car_data["s"];
    double car_d = car_data["d"];
    double car_yaw = car_data["yaw"];
    double car_speed = car_data["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = car_data["previous_path_x"];
    auto previous_path_y = car_data["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = car_data["end_path_s"];
    double end_path_d = car_data["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = car_data["sensor_fusion"];

    vector<bool> states = get_vehicle_state(car_s, car_d, sensor_fusion);
    bool car_left = states[0];
    bool car_right = states[1];
    bool car_ahead = states[2];
    bool car_head_close = states[3];

    //cout << "Car Left: " << car_left << "\tCar Right: " << car_right << "\tCar Head: " << car_ahead << "\tClose Ahead: " << car_head_close << endl;
    int previous_size = previous_path_x.size();

    for(int i = 0; i < previous_size; i++){
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if(previous_size < 10){

        double target_speed = MAX_SPEED;
        double ref_x = car_x;
        double ref_y = car_y;
        double ref_speed = car_speed;
        double ref_yaw = deg2rad(car_yaw);
        double prev_ref_x = car_x - 0.02*cos(ref_yaw);
        double prev_ref_y = car_y - 0.02*sin(ref_yaw);
        double dx, dy;

        if (previous_size > 2){
            ref_x = previous_path_x[previous_size-1];
            ref_y = previous_path_y[previous_size-1];
            prev_ref_x = previous_path_x[previous_size-2];
            prev_ref_y = previous_path_y[previous_size-2];

            dx = ref_x - prev_ref_x;
            dy = ref_y - prev_ref_y;
            ref_yaw = atan2(dy, dx);
            ref_speed = sqrt(pow(dx, 2) + pow(dy, 2)) / 0.02;
        }

        int car_ahead_sensor_index;


        if(car_ahead){
            car_ahead_sensor_index = get_index_closest_vehicle_same_lane(car_s, car_d, sensor_fusion);
            double sensor_speed = sqrt(
                pow(sensor_fusion[car_ahead_sensor_index][3],2.0) +
                pow(sensor_fusion[car_ahead_sensor_index][4],2.0)
            );
            target_speed = sensor_speed;
        }

        vector<double> s_coef,s_coef_plan;
        double min_cost = 9999999999.0;
        double final_target_speed_cost = 0;
        for(double dv = -4.0; dv <= 4.0; dv += 0.5){
             if ( ref_speed + dv <= MAX_SPEED && ref_speed + dv >= 0){
                 s_coef= solve_jerm_minimizing_coefs(
                      {car_s, ref_speed, 0},
                      {car_s + (ref_speed + dv / 2.0) * PATH_DT * NUM_PATH_POINTS, ref_speed + dv, 0},
                      PATH_DT * NUM_PATH_POINTS
                );

                double buffer_cost = 0;
                double target_speed_cost = 0;
                if(car_ahead){
                    car_ahead_sensor_index = get_index_closest_vehicle_same_lane(car_s, car_d, sensor_fusion);
                    double sensor_speed = sqrt(
                        pow(sensor_fusion[car_ahead_sensor_index][3],2.0) +
                        pow(sensor_fusion[car_ahead_sensor_index][4],2.0)
                    );
                    double sensor_s = sensor_fusion[car_ahead_sensor_index][5];
                    buffer_cost = ahead_buffer_cost(s_coef, PATH_DT * NUM_PATH_POINTS, sensor_s, sensor_speed);
                }

                target_speed_cost = cost_ref_velocity(ref_speed+dv);
                double total_cost = target_speed_cost + buffer_cost;
                //cout << car_s << ", " << car_d << ", " <<  ref_speed << ", " << dv << ", " << buffer_cost  << ", " << target_speed_cost  << ", " << total_cost << endl;

                if(total_cost < min_cost){
                    s_coef_plan = {};
                    for(int i = 0; i < s_coef.size(); i++){
                        s_coef_plan.push_back(s_coef[i]);
                    }
                    min_cost = total_cost;
                    final_target_speed_cost = target_speed_cost;
                }
             }
        }

        if(!car_left && final_target_speed_cost > 0.15 && car_ahead){
            this->target_d -= 4.0;
            this->setting_d -= 4.0;
            s_coef= solve_jerm_minimizing_coefs(
                 {car_s, ref_speed, 0},
                 {car_s + ref_speed * PATH_DT * NUM_PATH_POINTS, ref_speed, 0},
                 PATH_DT * NUM_PATH_POINTS
           );
        } else if(!car_right && final_target_speed_cost > 0.15 && car_ahead) {
            this->target_d += 4.0;
            this->setting_d += 4.0;
            s_coef= solve_jerm_minimizing_coefs(
                 {car_s, ref_speed, 0},
                 {car_s + ref_speed * PATH_DT * NUM_PATH_POINTS, ref_speed, 0},
                 PATH_DT * NUM_PATH_POINTS
           );
        } else {
            this->setting_d -= 0.05 * (car_d - this->target_d);

        }
        vector<double> d_coef_plan = solve_jerm_minimizing_coefs(
             {car_d, 0, 0},
             {this->setting_d, 0, 0},
             PATH_DT * NUM_PATH_POINTS
       );

        double correction_x, correction_y = 0;
        vector<double> xy0 = getXY(s_coef_plan[0], d_coef_plan[0], maps_s, this->map_x_spline, this->map_y_spline);
        next_x_vals = {};
        next_y_vals = {};

        if(previous_size > 2){
            next_x_vals = {previous_path_x[0]};
            next_y_vals = {previous_path_y[0]};
            correction_x = xy0[0] - next_x_vals[0];
            correction_y = xy0[1] - next_y_vals[0];
        }

        for (int i = 1; i < NUM_PATH_POINTS; i++){

            double s = evaluate(s_coef_plan, PATH_DT * i);
            double d = evaluate(d_coef_plan, PATH_DT * i);

            vector<double> xy = getXY(s, d, maps_s, this->map_x_spline, this->map_y_spline);
            next_x_vals.push_back(xy[0] - correction_x);
            next_y_vals.push_back(xy[1] - correction_y);
        }

        double x_correct, y_correct = 0;
        for( int i = 1; i < next_x_vals.size(); i++){
            double dx = next_x_vals[i] - next_x_vals[i-1] - x_correct;
            double dy = next_y_vals[i] - next_y_vals[i-1] - y_correct;
            double speed = sqrt(pow(dx, 2) + pow(dy, 2)) / PATH_DT;
            if (speed >= MAX_SPEED){
                next_x_vals[i] = next_x_vals[i-1] + dx * MAX_SPEED / speed;
                next_y_vals[i] = next_y_vals[i-1] + dy * MAX_SPEED / speed;
            }
        }
    }

    return;


}
