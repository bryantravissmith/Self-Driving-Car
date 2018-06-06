#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <vector>
#include <math.h>
#include "json.hpp"
#include "spline.h"

using namespace std;
using json = nlohmann::json;

class Executor {

public:

  vector<double> plan_trajectory_s_coefs;
  vector<double> plan_trajectory_d_coefs;
  tk::spline map_x_spline;
  tk::spline map_y_spline;
  double target_d;
  double setting_d;

  /* Constructo  */
  Executor();

  /* Destructor */
  virtual ~Executor();

  /* Initialize */
  void Init();

  /* Update the PID error variables given cross track error */
  void UpdatePlan(vector<double> s_coefs, vector<double> d_coefs);

  void SetTrajectory(
      vector<double>& next_x_vals, vector<double>& next_y_vals, json car_data,
      const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y
  );
};

#endif /* EXECUTER_H */
