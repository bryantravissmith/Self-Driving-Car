#ifndef COSTS
#define COSTS

#include <math.h>
#include <vector>
#include "config.h"
#include "helper.h"

using namespace std;


double logistic(double v){
    return 2.0 / (1.0 + exp(-v)) - 1.0;
}

double cost_ref_velocity(double v){
    double cost = abs(MAX_SPEED - v) / MAX_SPEED;
    return cost;
}

double ahead_buffer_cost(vector<double> s_coef, double t, double s_other, double v_other){
    double distance = s_other + v_other * t - evaluate(s_coef, t);
    if (distance > 2*FOLLOW_DISTANCE){
        return 0.0;
    }
    return pow(2.0*FOLLOW_DISTANCE - distance, 2) / pow(FOLLOW_DISTANCE, 2);
}

#endif
