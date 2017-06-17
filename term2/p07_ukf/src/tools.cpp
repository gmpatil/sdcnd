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

void Tools::writeNIS(double nis, bool lidar) {
  if (!initialized){
    lidar_nis_fl_.open("lidar_nis.txt");
    radar_nis_fl_.open("radar_nis.txt");
    initialized = true;
  }
  
  if (lidar){
    lidar_nis_fl_ << nis << std::endl;
  } else {
    radar_nis_fl_ << nis << std::endl;
  }
}