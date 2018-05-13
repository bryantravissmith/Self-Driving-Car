#include "Executor.h"
#include "spline.h"
#include <math.h>
#include "helper.h"


using namespace std;
using namespace helper;

Executor::Executor() {}

Executor::~Executor() {}

void Executor::Init() {
    this->plan_trajectory_s_coefs = {0, 0, 0, 0, 0, 0};
    this->plan_trajectory_d_coefs = {0, 0, 0, 0, 0, 0};
}

void Executor::UpdatePlan(vector<double> s_coefs, vector<double> d_coefs){
    this->plan_trajectory_s_coefs = s_coefs;
    this->plan_trajectory_d_coefs = d_coefs;
}

void Executor::SetTrajectory(
    vector<double>& next_x_vals, vector<double>& next_y_vals,
    const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y
){
    tk::spline s;
    s.set_points(next_x_vals, next_y_vals);    // X needs to be sorted, strictly increasing
    vector<double> s_trajectory;
    vector<double> d_trajectory;
    vector<double> xy;

    for(int i = 0; i < 40; i++){
        s_trajectory = this->Evaluate(
            this->plan_trajectory_s_coefs,
            0.250 * i
        );
        d_trajectory = this->Evaluate(
            this->plan_trajectory_d_coefs,
            0.250 * i
        );
        xy = getXY(
            s_trajectory[0],
            d_trajectory[0],
            maps_s,
            maps_x,
            maps_y
        );

    }
    for(int i = 0; i < 50; i++){
        next_x_vals.push_back(0);
        next_y_vals.push_back(0);
    }
}

vector<double> Executor::Evaluate(vector<double> coef, double t_sec){
    double p = 0;
    double v = 0;
    double a = 0;
    double j = 0;

    for (int i = 0; i < coef.size(); i++){
        double value = coef[i] * pow(t_sec, i);
        p += value;
        if (i >= 1) v += i * coef[i] * pow(t_sec, i-1);
        if (i >= 2) a += i * (i - 1) * coef[i] * pow(t_sec, i-2);
        if (i >= 3) j += i * (i - 1) * (i-2) * coef[i] * pow(t_sec, i-3);
    }
    return {p, v, a, j};
}
