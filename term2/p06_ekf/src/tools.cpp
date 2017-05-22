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
    * Calculate the RMSE here.
    */
	VectorXd rmse(4);
	rmse << 0,0,0,0;
  
	// estimation vector size should not be zero
  if (estimations.size() <= 0) {
      cout << " estimations is NULL or empty" << endl;
      return rmse;
  }

  if ( ground_truth.size() <= 0) {
      cout << " ground_truth is NULL or empty" << endl;
      return rmse;
  }
        
	// estimation vector size should equal ground truth vector size
  if ( (estimations.size() != ground_truth.size())) {
      cout << " estimations.size() != ground_truth.size() " << endl;
      return rmse;
  }
        
  VectorXd sqRes(4);
  sqRes << 0.0, 0.0, 0.0, 0.0 ;
  VectorXd sqRest(4);
        
	// accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
    //cout << estimations.at(i) << endl;
    //cout << ground_truth.at(i) << endl;
    sqRest = ( estimations.at(i) - ground_truth.at(i)) ;
    //cout << sqRest << endl << endl;
    sqRest = sqRest.array() * sqRest.array();
    //cout << sqRest << endl << endl;
    sqRes = sqRes + sqRest;
    //cout << sqRes << endl << endl;
	}

	// calculate the mean
  sqRes /= estimations.size();
        
	// calculate the squared root
  rmse = sqRes.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
    */
	MatrixXd Hj(3,4);
  
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float px2_py2 = px*px + py*py ;
  float sqrt_px2_py2 = sqrt(px2_py2) ;
  float sqrtOfCube_px2_py2 = sqrt(px2_py2 * px2_py2 * px2_py2) ;
        
	// check division by zero
	if ( (px2_py2 == 0.0) || (sqrt_px2_py2 == 0.0) || (sqrtOfCube_px2_py2 == 0.0) ) {
    cout << "Divide by zero!" << endl;
    return Hj;
    //return NULL; // does not work
  }
        
	// compute the Jacobian matrix
  Hj << (px/sqrt_px2_py2), (py/sqrt_px2_py2), 0.0, 0.0,
        (-py/px2_py2), (px/px2_py2), 0.0, 0.0,
        ((py* (vx * py - vy * px) )/sqrtOfCube_px2_py2), ((px* (vy * px - vx * py) )/sqrtOfCube_px2_py2), (px/sqrt_px2_py2), (py/sqrt_px2_py2) ;
  
    return Hj;  
}
