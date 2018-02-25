#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float p_sqrt = sqrt(pow(px, 2) + pow(py,2));

  if(p_sqrt==0){
      std::cout<<"Tools::CalculateJacobia - Devide by Zero Error"<<endl;
  } else{
      Hj << px / p_sqrt, py / p_sqrt, 0, 0,
          -py / pow(p_sqrt,2), px / pow(p_sqrt,2), 0, 0,
          py*(vx*py-vy*px)/pow(p_sqrt,3), px*(vy*px-vx*py)/pow(p_sqrt,3),
          px/p_sqrt, py/p_sqrt;
  }

	return Hj;
}
