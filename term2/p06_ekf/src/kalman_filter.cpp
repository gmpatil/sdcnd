#include <math.h>  
#include <iostream>

#include "kalman_filter.h"

        
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  float px2_py2 = px*px + py*py ;
  float sqrt_px2_py2 = sqrt(px2_py2) ;
  float sqrtOfCube_px2_py2 = sqrt(px2_py2 * px2_py2 * px2_py2) ;
        
	//check division by zero
	if ( (px2_py2 == 0.0) || (sqrt_px2_py2 == 0.0) || (sqrtOfCube_px2_py2 == 0.0) ) {
      cout << "Divide by zero!" << endl;
      return;
  }
        
	//compute the Jacobian matrix
  Hj << (px/sqrt_px2_py2), (py/sqrt_px2_py2), 0.0, 0.0,
        (-py/px2_py2), (px/px2_py2), 0.0, 0.0,
        ((py* (vx * py - vy * px) )/sqrtOfCube_px2_py2), ((px* (vy * px - vx * py) )/sqrtOfCube_px2_py2), (px/sqrt_px2_py2), (py/sqrt_px2_py2) ;

  VectorXd z_pred(3);
  const float  PI_F=3.14159265358979f;
  z_pred(0) = sqrt_px2_py2 ;
  float phi = atan2(py, px);
  z_pred(1) = atan2 (sin(phi), cos(phi)) ;
  z_pred(2) = ((px * vx) + (py * vy)) / sqrt_px2_py2 ;
  
	VectorXd y = z - z_pred;
	MatrixXd Ht = Hj.transpose();
	MatrixXd S = Hj * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;  
}
