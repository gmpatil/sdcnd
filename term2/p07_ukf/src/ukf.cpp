#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// static constants
static const int n_sigma = 2 * 7 + 1; // number of sigma points
static const int n_z_radar = 3;  // radar measurement dimensions. r, phi, and r_dot
static const int n_z_laser = 2; // laser measurement dimensions, x, y


/**
 * Generate augmented sigma points.
 * 
 * @param x - state vector
 * @param P - covariance matrix
 * @param n_x - number of state dimensions
 * @param n_aug - number of augumented state dimensions
 * @param lamda - spreading parameter
 * @param std_a - process noise standard deviation longitudinal acceleration in m/s^2
 * @param std_yawdd - process noise standard deviation yaw acceleration in rad/s^2
 * @param Xsig_out - augmented sigma points matrix
 */
void generateAugmentedSigmaPoints(VectorXd x, MatrixXd P, int n_x, int n_aug, 
        double lambda, double std_a, double std_yawdd, MatrixXd* Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug, 2 * n_aug + 1);
 
  //create augmented mean state
  x_aug.head(n_x) = x;
  x_aug(n_x) = 0; //mean a not std_a;
  x_aug(n_x + 1) = 0; //mean yaw not std_yawdd;
  
  //create augmented covariance matrix
  P_aug.topLeftCorner(n_x, n_x) = P;
//  MatrixXd Q = MatrixXd::Zero(2, 2);
//  Q(0,0) = std_a * std_a;
//  Q(0,1) = 0 ;
//  Q(1,0) = 0 ;
//  Q(1,1) = std_yawdd * std_yawdd ;
//  P_aug.bottomRightCorner(2,2) = Q;
  
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;  

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  
  //create augmented sigma points
  //set first column of sigma point matrix
  Xsig_aug.col(0)  = x_aug;  

  //set remaining sigma points
  for (int i = 0; i < n_aug ; i++)
  {
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda + n_aug) * A.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda + n_aug) * A.col(i);
  }  

  //print result
//  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;
  
}

double normalizeAngle(double angle){
    if ((angle > M_PI) || ((angle < -M_PI))){
      angle = atan2 (sin(angle), cos(angle)) ; 
    } 
    
    return angle;
}

/**
 * Predict sigma points.
 * 
 * @param Xsig_aug - augmented sigma points.
 * @param delta_t - time in seconds between measurement updates
 * @param n_x - number of state dimensions
 * @param n_aug - number of augumented state dimensions
 * @param Xsig_out - predicted sigma points.
 */
void predictSigmaPoint(MatrixXd Xsig_aug, double delta_t, int n_x, int n_aug, 
        MatrixXd* Xsig_out) {
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd::Zero(n_x, 2 * n_aug + 1);

  double deltaTsq = delta_t * delta_t;
      
  //predict sigma points
  // x => (px, py, vel, yaw angle, yaw angle change rate)
  for (int i=0; i < (2 * n_aug + 1); i++ ) {    
    double px     = Xsig_aug(0, i);
    double py     = Xsig_aug(1, i);
    double v      = Xsig_aug(2, i);
    double yaw    = Xsig_aug(3, i);
    double yawDot = Xsig_aug(4, i);
    double va     = Xsig_aug(5, i);
    double yawDotDot = Xsig_aug(6, i);

    double yawDotDeltaT = yawDot * delta_t;
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);
    double deltaTsq = delta_t * delta_t;
    
    // prediction
    double p_px;
    double p_py;
    double p_v;
    double p_yaw;
    double p_yawDot;
    
    // avoid division by zero
    if (fabs(yawDot) > 0.001) {
      p_px = px + (v/yawDot) * (sin(yaw + yawDotDeltaT) - sinYaw);
      p_py = py + (v/yawDot) * (-1.0 * cos(yaw + yawDotDeltaT) + cosYaw);
    } else {
      // yawDot ~= 0.0 ;
      p_px = px + v * cosYaw * delta_t ;
      p_py = py + v * sinYaw * delta_t ;
    }
	
    p_v = v;
    p_yaw = (yaw + yawDotDeltaT);
    p_yawDot = yawDot ;

    // add noise
    p_px = p_px + (0.5) * deltaTsq * cosYaw * va ;
    p_py = p_py + (0.5) * deltaTsq * sinYaw * va ;
    p_v = p_v + delta_t * va ;
    p_yaw = p_yaw + (0.5) * deltaTsq * yawDotDot;
    p_yawDot = p_yawDot + delta_t * yawDotDot ;   
    
    Xsig_pred(0, i) = p_px;
    Xsig_pred(1, i) = p_py;
    Xsig_pred(2, i) = p_v;
    Xsig_pred(3, i) = p_yaw;
    Xsig_pred(4, i) = p_yawDot;   
  }
  
  //print result
//  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;   
}

/**
 * Predict mean and covariance using predicted sigma points.
 * 
 * @param Xsig_pred  - predicted sigma points
 * @param n_x - number of state dimensions
 * @param n_aug - number of augumented state dimensions
 * @param lamda - spreading parameter
 * @param x_out - predicted state mean 
 * @param P_out - predicted state covariance
 */
void predictStateMeanAndCovariance(MatrixXd Xsig_pred, int n_x, int n_aug, 
        VectorXd weights, VectorXd* x_out, MatrixXd* P_out) {
//  cout << "In predictMeanAndCovariance" << endl;
  //create vector for predicted state
  VectorXd x = VectorXd::Zero(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd::Zero(n_x, n_x);
  
  //predict state mean    
  x.fill(0.0);  
  for (int i=0; i < 2 * n_aug + 1; i++){
    x = x +  weights(i) * Xsig_pred.col(i) ; 
  } 
  
//  cout << "after loop1" << endl;
  
  //x = Xsig_pred*weights;
  
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    x_diff(3) = normalizeAngle(x_diff(3));
    
    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

//  cout << "after loop2" << endl;
  
  //print result
//  std::cout << "Predicted state X:" << std::endl;
//  std::cout << x << std::endl;
//  std::cout << "Predicted state covariance matrix P:" << std::endl;
//  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
  
//  cout << "ret predictMeanAndCovariance" << std::endl;  
}

/**
 * Transform predicted sigma points into radar measurement space.
 * 
 * @param Xsig_pred - matrix with predicted sigma points
 * @param n_aug - number of augumented state dimensions
 * @param weights - 
 * @param std_radar - 
 * @param std_radphi - 
 * @param std_radrd -  
 * @param z_out - vector for mean predicted measurement
 * @param S_out - matrix for predicted measurement covariance
 * @param Zsig_out - matrix for predicted sigma points in radar measurement space
 * 
 */
void predictRadarMeasurement(MatrixXd Xsig_pred, int n_aug, VectorXd weights, 
        MatrixXd R, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {
//  cout << "In predictRadarMeasurement" << endl;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z_radar, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z_radar);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z_radar,n_z_radar);

  //transform sigma points into measurement space
  //calculate mean predicted measurement  
  z_pred.fill(0.0);

  for (int i=0; i < 2 * n_aug + 1; i++){
    double px = Xsig_pred(0,i);
    double py = Xsig_pred(1,i);
    double v = Xsig_pred(2,i);
    double yaw = normalizeAngle(Xsig_pred(3,i));
    double vx = cos(yaw) * v ;
    double vy = sin(yaw) * v ;
    
    // avoid division by zero
    if (fabs(px) < 1e-3){
        px = 1e-3;
    }
    
    if (fabs(py) < 1e-3) {
        py = 1e-3;
    }
                        
    double rho = max(0.0001, sqrt(px * px + py * py));
    //ro 
    Zsig(0, i) = rho;
    // phi
    Zsig(1,i) = normalizeAngle(atan2(py, px));
    //roDot
    Zsig(2, i) = (px * vx + py * vy) /rho; // max(rho, 1.0e-5)

    double wi = weights(i);
    
    //calculate mean predicted measurement    
    z_pred = z_pred + (wi * Zsig.col(i));
    z_pred(1) = normalizeAngle(z_pred(1));
  }   

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = normalizeAngle(z_diff(1));
    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  S = S + R;  

  //print result
//  std::cout << "z_pred radar (ro, phi, roDot):\n" << z_pred << std::endl;
//  std::cout << "S radar: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}


/**
 * Transform predicted sigma points into lidar measurement space.
 * 
 * @param Xsig_pred - matrix with predicted sigma points
 * @param n_aug - 
 * @param weights - 
 * @param std_laspx - 
 * @param std_laspy - 
 * @param z_out - vector for mean predicted measurement
 * @param S_out - matrix for predicted measurement covariance
 * @param Zsig_out - matrix for predicted sigma points in radar measurement space
 * 
 */
void predictLidarMeasurement(MatrixXd Xsig_pred, int n_aug, VectorXd weights, 
        MatrixXd R, VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {
//  cout << "In predictLidarMeasurement" << endl;  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z_laser, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z_laser);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z_laser,n_z_laser);

  //transform sigma points into measurement space
  //calculate mean predicted measurement  
  z_pred.fill(0.0);
//  cout << "In predictLidarMeasurement before loop" << endl; 
  for (int i=0; i < 2 * n_aug + 1; i++){
    //px 
    Zsig(0, i) = Xsig_pred(0,i);
    
    // py
    Zsig(1,i) = Xsig_pred(1, i);
    
    double wi = weights(i);
    
    //calculate mean predicted measurement    
    z_pred = z_pred + (wi * Zsig.col(i));
  }   

  //calculate measurement covariance matrix S
  S.fill(0.0);
//  cout << "In predictLidarMeasurement before loop2" << endl; 
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  S = S + R;  

  //print result
//  std::cout << "z_pred Lidar (px, py):\n" << z_pred << std::endl;
//  std::cout << "S lidar: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}


/**
 * Update state for radar measurements
 * 
 * @param Xsig_pred - matrix with predicted sigma points in state space
 * @param x - vector for predicted state mean
 * @param P - matrix for predicted state covariance
 * @param Zsig - matrix with sigma points in measurement space
 * @param z_pred - vector for mean predicted measurement
 * @param S - matrix for predicted measurement covariance
 * @param z - vector for incoming radar measurement
 * @param n_x - number state dimensions
 * @param n_aug - number of augmented state dimensions
 * @param weights - weights for sigma points
 * @param x_out - vector for updated state mean
 * @param P_out - matrix for updated state covariance
 */
void updateStateForRadar(MatrixXd Xsig_pred, VectorXd x, MatrixXd P, 
        MatrixXd Zsig, VectorXd z_pred, MatrixXd S, VectorXd z, 
        int n_x, int n_aug, VectorXd weights, VectorXd* x_out, MatrixXd* P_out,
        double* nis) {
  cout << "In updateStateForRadar" << endl;
  cout << "Predicted state:\n" << x << endl;
  cout << "Predicted state(z_pred radar):\n" << z_pred << endl;
  cout << "Measurement state(z radrar):\n" << z << endl;
          
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x, n_z_radar);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd x_diff = Xsig_pred.col(i) - x;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    x_diff(3) = normalizeAngle(x_diff(3)); // yaw angle
    z_diff(1) = normalizeAngle(z_diff(1));  // phi angle
    
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }  
  
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd::Zero(n_x, n_z_radar);
  K = Tc * S.inverse();
  
  //update state mean and covariance matrix  
  VectorXd z_diff = (z - z_pred);
  //angle normalization
  z_diff(1) = normalizeAngle(z_diff(1)); // phi angle
  
  x = x + K * z_diff ;  
  
  //angle normalization
  x(3) = normalizeAngle(x(3)); // yaw angle
    
  //covariance matrix
  P = P - K * S * K.transpose();

  double nis_radar = z_diff.transpose() * S.inverse() * z_diff ; 
  
  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
//  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
  *nis = nis_radar;
}

/**
 * Update state for lidar measurements
 * 
 * @param Xsig_pred - matrix with predicted sigma points in state space
 * @param x - vector for predicted state mean
 * @param P - matrix for predicted state covariance
 * @param Zsig - matrix with sigma points in measurement space
 * @param z_pred - vector for mean predicted measurement
 * @param S - matrix for predicted measurement covariance
 * @param z - vector for incoming radar measurement
 * @param n_x - number state dimensions
 * @param n_aug - number of augmented state dimensions
 * @param weights - weight for sigma points
 * @param x_out - vector for updated state mean
 * @param P_out - matrix for updated state covariance
 */
void updateStateForLidar(MatrixXd Xsig_pred, VectorXd x, MatrixXd P, 
        MatrixXd Zsig, VectorXd z_pred, MatrixXd S, VectorXd z, int n_x, 
        int n_aug, VectorXd weights, VectorXd* x_out, MatrixXd* P_out,
        double* nis) {
  cout << "In updateStateForLidar" << endl;
  cout << "Predicted state:\n" << x << endl;
  cout << "Predicted state(z_pred lidar):\n" << z_pred << endl;
  cout << "Measurement state(z lidar):\n" << z << endl;
            
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x, n_z_laser);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 sigma points
    //residual
    VectorXd x_diff = Xsig_pred.col(i) - x;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    x_diff(3) = normalizeAngle(x_diff(3));  // yaw angle
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }  
  
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd::Zero(n_x, n_z_laser);
  K = Tc * S.inverse();
  
  //update state mean and covariance matrix  
  VectorXd z_diff = (z - z_pred);
  x = x + K * z_diff ;  

  //angle normalization
  x(3) = normalizeAngle(x(3)); // yaw angle

  //covariance matrix
  P = P - K * S * K.transpose();

  double nis_laser = z_diff.transpose() * S.inverse() * z_diff ; 
  
  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
//  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
  *nis = nis_laser;  
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  // [px py vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5); 

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // the fastest legal sports car acceleration is currently 0 to 60 mph in 2.2
  // seconds. 0 to 60 mph in 2.2 seconds is about 12m/s^2.
  // For cycle it should be even less.
  std_a_ = 1.15 ; //0.15; // 2.
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.55 ; //(M_PI/6.0);// .7 // 16 sec +/- 8 sec to circle  

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
  double std_a2 = std_a_ * std_a_;

  P_ = MatrixXd(5, 5);
  P_.fill(0);
  P_ << 1.0, 0, 0, 0, 0,
        0, 1.0, 0, 0, 0,
        0, 0, 1.0, 0, 0, 
        0, 0, 0, 1.0, 0,
        0, 0, 0, 0, 1.0;    
  
  n_x_ = 5; // state dimension
  n_aug_ = 7; // augmented dimension
  lambda_ = 3.0 - n_x_; // spreading parameter
  
  R_radar_ = MatrixXd::Zero(n_z_radar, n_z_radar);  
  R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_ * std_radr_ ;
  R_radar_(1,1) = std_radphi_ * std_radphi_ ;
  R_radar_(2,2) = std_radrd_ * std_radrd_ ;
  
  R_lidar_ = MatrixXd::Zero(n_z_laser, n_z_laser);  
  R_lidar_.fill(0.0);
  R_lidar_(0,0) = std_laspx_ * std_laspx_ ;
  R_lidar_(1,1) = std_laspy_ * std_laspy_ ;  
  
  H_  = MatrixXd(2, n_x_); // Px, Py
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0; 
  
  is_initialized_ = false;
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
  //cout << "In ProcessMeasurement" << endl;
  if (!is_initialized_) {
    // first measurement
    cout << "UKF: " << endl;
    x_ << 1, 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rho_dot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);  // px
      x_(1) = rho * sin(phi);  // py 
      x_(2) = 0.0;             // vel. rho_dot != vel, was not useful to set on data set 1. //fabs(rho_dot)
      x_(3) = 0.0;             // yaw. phi != yaw, was not useful to set on data set 1 //phi
      x_(4) = 0.0;             // yaw rate
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      double x = meas_package.raw_measurements_[0];
      double y = meas_package.raw_measurements_[1];
      // safety feedback
      if ( fabs(x) < 0.001 && fabs(y) < 0.001 ) {
         x = 0.001;
         y = 0.001;
      }      
      x_(0) = x;   //px
      x_(1) = y;   //py
      x_(2) = 0.0; //vel
      x_(3) = 0.0; //yaw   // yaw != phi = atan2(x_(1), x_(0));
      x_(4) = 0.0;
    }

    time_us_ = meas_package.timestamp_;
    
    VectorXd weights = VectorXd(2 * n_aug_+ 1);
    weights.fill(0.5/(n_aug_ + lambda_));
    weights(0) = lambda_/(lambda_ + n_aug_);
    
    weights_ = weights;
    
    // wait for vehicle move away for the origin to finish initializing.
//    if (fabs(x_(0)) + fabs(x_(1)) < 1e-3)  {
//      is_initialized_ = false;
//    } else {    
//      // done initializing, no need to predict or update
//      is_initialized_ = true;
//    }
    is_initialized_ = true;
    
    return;
  }

  long long previous_timestamp_ = time_us_;
  //dt - expressed in seconds
	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	
	time_us_ = meas_package.timestamp_;

  double step = 0.1;
  while (dt > 0.2){
    Prediction(step);
    dt -= step;
  }

  Prediction(dt);
    
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) 
          && (use_radar_)) {    
    UpdateRadar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) 
          && (use_laser_)) {    
    UpdateLidar(meas_package);
  }  
  
  //cout << "Returning from ProcessMeasurement" << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd Xsig_aug;
  
  //cout << "In prediction" << endl;
  generateAugmentedSigmaPoints(x_, P_, n_x_, n_aug_, lambda_, std_a_, 
          std_yawdd_, &Xsig_aug);
  predictSigmaPoint(Xsig_aug, delta_t, n_x_, n_aug_, &Xsig_pred_);
  predictStateMeanAndCovariance(Xsig_pred_, n_x_, n_aug_, weights_, &x_, &P_);
  
  //cout << "ret from prediction" << endl;
}
 
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * Non-linear function.
 * @param {MeasurementPackage} meas_package
 */
//void UKF::UpdateLidar(MeasurementPackage meas_package) {
//  /**
//  Complete this function! Use lidar data to update the belief about the object's
//  position. Modify the state vector, x_, and covariance, P_.
//
//  You'll also need to calculate the lidar NIS.
//  */
//  //cout << "In updateLidar" << endl;
//  
//  MatrixXd z_sigma_pred_lidar; // predicted sigma points in measurement space  
//  VectorXd z_predict_lidar; // predicted sigma mean in measurement space
//  MatrixXd S; // predicted measurement covariance
//  VectorXd z_lidar = meas_package.raw_measurements_; // current lidar measurement
//  VectorXd x_out;
//  MatrixXd P_out;
//  
//  predictLidarMeasurement(Xsig_pred_, n_aug_, weights_, R_lidar_, 
//          &z_predict_lidar, &S, &z_sigma_pred_lidar);
//  
//  updateStateForLidar(Xsig_pred_, x_, P_, z_sigma_pred_lidar, z_predict_lidar, 
//          S, z_lidar, n_x_, n_aug_, weights_, &x_out, &P_out, &NIS_laser_);
//  
//  x_ = x_out;
//  P_ = P_out;  
//}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * Linear function.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd z_lidar = meas_package.raw_measurements_; // current lidar measurement
	VectorXd z_pred = H_ * x_;
	VectorXd y = z_lidar - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;  
	MatrixXd S = H_ * PHt + R_lidar_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
	P_ = (I - K * H_) * P_;    
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //cout << "In updateRadar" << endl;
  
  MatrixXd z_sigma_pred_radar; // predicted sigma points in measurement space  
  VectorXd z_predict_radar; // predicted sigma mean in measurement space
  MatrixXd S; // predicted measurement covariance
  VectorXd z_radar = meas_package.raw_measurements_;
  VectorXd x_out;
  MatrixXd P_out;
  
  predictRadarMeasurement(Xsig_pred_, n_aug_, weights_, R_radar_, 
          &z_predict_radar, &S, &z_sigma_pred_radar);
  
  updateStateForRadar(Xsig_pred_, x_, P_, z_sigma_pred_radar, z_predict_radar, 
          S, z_radar, n_x_, n_aug_, weights_, &x_out, &P_out, &NIS_radar_);
  
  x_ = x_out;
  P_ = P_out;
}


/*
 std_a_ = 5.0 ; //0.15; 
 std_yawdd_ = 3 * M_PI;
 RMSE: 0.20, 0.18. 0.81, 0.66
 
 
 
 */