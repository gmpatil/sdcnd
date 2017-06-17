#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
  bool initialized = false;
  ofstream lidar_nis_fl_; 
  ofstream radar_nis_fl_ ;
  
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * Helper method to write to lidar or radar NIS file
   */
  void writeNIS(double nis, bool lidar);  
};

#endif /* TOOLS_H_ */