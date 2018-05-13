#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <vector>
#include <math.h>

using namespace std;

class Executor {

public:

  vector<double> plan_trajectory_s_coefs;
  vector<double> plan_trajectory_d_coefs;

  /* Constructo  */
  Executor();

  /* Destructor */
  virtual ~Executor();

  /* Initialize */
  void Init();

  /* Update the PID error variables given cross track error */
  void UpdatePlan(vector<double> s_coefs, vector<double> d_coefs);

  void SetTrajectory(
      vector<double>& next_x_vals, vector<double>& next_y_vals,
      const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y
  );

  vector<double> Evaluate(vector<double> coef,double t_sec);
};



#endif /* EXECUTER_H */
