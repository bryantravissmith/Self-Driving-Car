#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if(estimations.size() == 0 | (estimations.size()!=ground_truth.size())){
      std::cout<<"Tools::CalculateRMSE - 0 or mismatch size";
      return rmse;
  }

  for(int i=0; i < estimations.size(); ++i){
        // ... your code here
  	VectorXd resid = ground_truth[i]-estimations[i];
  	VectorXd resid2 = resid.array() * resid.array();
  	rmse += resid2;
  }

  rmse /= estimations.size();

  return rmse.array().sqrt();
}
