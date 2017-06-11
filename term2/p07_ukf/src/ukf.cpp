#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  // the fastest legal sports car acceleration is currently 0 to 60 mph in 2.2
  // seconds. 0 to 60 mph in 2.2 seconds is about 12m/s^2.
  // For cycle it should be even less.
  std_a_ = 0.15; // 3
  double std_a2 = std_a_ * std_a_;

  std_yawdd_ = (M_PI/8.0) * 0.5; // 16 sec +/- 8 sec to circle

  P_ = MatrixXd(5, 5);
  P_ << std_a2, 0, 0, 0, 0,
        0, std_a2, 0, 0, 0,
        0, 0, std_a2, 0, 0,
        0, 0, 0, std_yawdd_ * std_yawdd_, 0;
        0, 0, 0, 0, 0.001;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  if (!is_initialized_) {
    // first measurement
    cout << "UKF: " << endl;
    x_ << 1, 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      x_(2) = 0.0;
      x_(3) = 0.0;
      x_(4) = 0.0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0.0;
      x_(3) = 0.0;
      x_(3) = 0.0;
    }

    time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  long long previous_timestamp_ = time_us_;
  //dt - expressed in seconds
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	
	time_us_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    
  }
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

// Private methods.
void generateSigmaPoints(MatrixXd* Xsig_out, VectorXd x, int n_x,
        MatrixXd P) {
  
  //define spreading parameter
  double lambda = 3 - n_x;

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  //calculate square root of P
  MatrixXd A = P.llt().matrixL();
  
  //calculate sigma points ...
  //set sigma points as columns of matrix Xsig
  // 1st column mean or X
  Xsig.col(0)  = x;
  
  MatrixXd nxSigmasDisc = (sqrt((lambda + n_x)) * A);
  //MatrixXd nxSigmas = x + nxSigmasDisc;
  //         nxSigmas = x - nxSigmasDisc;
  for (int j=1; j<= n_x; j++) {
    for (int i=0; i< x.size(); i++) {
      Xsig(i,j) = x(i) + nxSigmasDisc(i, j-1);
      Xsig(i,j + n_x) = x(i) - nxSigmasDisc(i, j-1);      
    }
  }

  //print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  //write result
  *Xsig_out = Xsig;
}

void generateAugmentedSigmaPoints(MatrixXd* Xsig_out, VectorXd x, int n_x, MatrixXd P ) {
  //set augmented dimension
  int n_aug = 7;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
 
  //create augmented mean state
  x_aug.head(n_x) = x;
  x_aug(n_x) = 0; //mean a not std_a;
  x_aug(n_x + 1) = 0; //mean yaw not std_yawdd;
  
  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x, n_x) = P;
  MatrixXd Q = MatrixXd(2, 2);
  Q(0,0) = std_a * std_a;
  Q(0,1) = 0 ;
  Q(1,0) = 0 ;
  Q(1,1) = std_yawdd * std_yawdd ;
  P_aug.bottomRightCorner(2,2) = Q;
  
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  //create augmented sigma points
  //set first column of sigma point matrix
  Xsig_aug.col(0)  = x_aug;  

  //set remaining sigma points
  for (int i = 0; i < n_aug ; i++)
  {
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda+n_aug) * A.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * A.col(i);
  }  

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;
  
}