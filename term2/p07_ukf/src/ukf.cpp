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

// local member methods
void generateAugmentedSigmaPoints(VectorXd x, MatrixXd P, int n_x, int n_aug, 
        double lambda, double std_a, double std_yawdd, MatrixXd* Xsig_out);
void predictSigmaPoint(MatrixXd Xsig_aug, double delta_t, int n_x, int n_aug, 
        MatrixXd* Xsig_out) ;
void predictMeanAndCovariance(MatrixXd Xsig_pred, int n_x, int n_aug, 
        double lambda, VectorXd* x_out, MatrixXd* P_out);
void predictRadarMeasurement(MatrixXd Xsig_pred, int n_aug, VectorXd weights, 
        double std_radr, double std_radphi, double std_radrd, VectorXd* z_out, 
        MatrixXd* S_out, MatrixXd* Zsig_out);
void predictLidarMeasurement(MatrixXd Xsig_pred, int n_aug, 
        VectorXd weights, double std_laspx, double std_laspy, VectorXd* z_out, 
        MatrixXd* S_out, MatrixXd* Zsig_out);
void updateStateForRadar(MatrixXd Xsig_pred, VectorXd x, MatrixXd P, 
        MatrixXd Zsig, VectorXd z_pred, MatrixXd S, VectorXd z, 
        int n_x, int n_aug, VectorXd weights, VectorXd* x_out, MatrixXd* P_out,
        double* nis);
void updateStateForLidar(MatrixXd Xsig_pred, VectorXd x, MatrixXd P, 
        MatrixXd Zsig, VectorXd z_pred, MatrixXd S, VectorXd z, int n_x, 
        int n_aug, VectorXd weights, VectorXd* x_out, MatrixXd* P_out, 
        double* nis);

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
  // the fastest legal sports car acceleration is currently 0 to 60 mph in 2.2
  // seconds. 0 to 60 mph in 2.2 seconds is about 12m/s^2.
  // For cycle it should be even less.
  std_a_ = 0.15; 
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = (M_PI/8.0) * 0.5; // 16 sec +/- 8 sec to circle  

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
  P_ << std_a2, 0, 0, 0, 0,
        0, std_a2, 0, 0, 0,
        0, 0, std_a2, 0, 0,
        0, 0, 0, std_yawdd_ * std_yawdd_, 0;
        0, 0, 0, 0, 0.001;
        
  n_x_ = 5; // state dimension
  n_aug_ = 7; // augmented dimension
  lambda_ = 3 - n_x_; // spreading parameter
    
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
      x_(2) = rho_dot;
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
    
    VectorXd weights = VectorXd(2 * n_aug_ + 1);
    double weight_0 = lambda_/(lambda_ + n_aug_);
    weights(0) = weight_0;
    for (int i=1; i< 2 * n_aug_ +1; i++) {  
      double weight = 0.5/(n_aug_ + lambda_);
      weights(i) = weight;
    }
//  VectorXd weights = VectorXd(2 * n_aug_+ 1);
//  double weight = 0.5/(n_aug_ + lambda_);
//  weights.fill(weight);
//  weights(0) = lambda_/(lambda_ + n_aug_);
    
    weights_ = weights;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  long long previous_timestamp_ = time_us_;
  //dt - expressed in seconds
	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	
	time_us_ = meas_package.timestamp_;
  
  Prediction(dt);

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) 
          && (use_radar_)) {
    UpdateRadar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) 
          && (use_laser_)) {
    UpdateLidar(meas_package);
  }  
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
  
  generateAugmentedSigmaPoints(x_, P_, n_x_, n_aug_, lambda_, std_a_, 
          std_yawdd_, &Xsig_aug);
  predictSigmaPoint(Xsig_aug, delta_t, n_x_, n_aug_, &Xsig_pred_);
  predictMeanAndCovariance(Xsig_pred_, n_x_, n_aug_, lambda_, &x_, &P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  MatrixXd z_sigma_pred_lidar; // predicted sigma points in measurement space  
  VectorXd z_predict_lidar; // predicted sigma mean in measurement space
  MatrixXd S; // predicted measurement covariance
  VectorXd z_lidar = meas_package.raw_measurements_; // current lidar measurement
  VectorXd x_out;
  MatrixXd P_out;
  
  predictLidarMeasurement(Xsig_pred_, n_aug_, weights_, std_laspx_, std_laspy_, 
          &z_predict_lidar, &S, &z_sigma_pred_lidar);
  updateStateForLidar(Xsig_pred_, x_, P_, z_sigma_pred_lidar, z_predict_lidar, 
          S, z_lidar, n_x_, n_aug_, weights_, &x_out, &P_out, &NIS_laser_);
  
  x_ = x_out;
  P_ = P_out;  
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

  MatrixXd z_sigma_pred_radar; // predicted sigma points in measurement space  
  VectorXd z_predict_radar; // predicted sigma mean in measurement space
  MatrixXd S; // predicted measurement covariance
  VectorXd z_radar = meas_package.raw_measurements_;
  VectorXd x_out;
  MatrixXd P_out;
  
  predictRadarMeasurement(Xsig_pred_, n_aug_, weights_, std_radr_, std_radphi_, 
          std_radrd_, &z_predict_radar, &S, &z_sigma_pred_radar);
  updateStateForRadar(Xsig_pred_, x_, P_, z_sigma_pred_radar, z_predict_radar, 
          S, z_radar, n_x_, n_aug_, weights_, &x_out, &P_out, &NIS_radar_);
  
  x_ = x_out;
  P_ = P_out;
}

// Private methods.

/**
 * Generate sigma points.
 * 
 * @param x - state vector
 * @param P - covariance matrix
 * @param n_x - number of state dimensions
 * @param lamda - spreading parameter
 * @param Xsig_out - sigma points matrix
 */
void generateSigmaPoints(VectorXd x, MatrixXd P, int n_x, double lambda, 
        MatrixXd* Xsig_out) {

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
    Xsig_aug.col(i+1)     = x_aug + sqrt(lambda + n_aug) * A.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda + n_aug) * A.col(i);
  }  

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;
  
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
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  //predict sigma points
  //avoid division by zero
  double phiDotDeltaT = 0.0 ;
  double cosPhi = 0.0;
  double sinPhi = 0.0;
  double deltaTsq = 0.0;
  
  for (int i=0; i < (2 * n_aug + 1); i++ ) {    
    phiDotDeltaT = Xsig_aug(4, i) * delta_t;
    cosPhi = cos(Xsig_aug(3, i));
    sinPhi = sin(Xsig_aug(3, i));
    deltaTsq = delta_t * delta_t;
    
    if (Xsig_aug(4, i) > 0.001) {
      Xsig_pred(0, i) = Xsig_aug(0, i) 
              + (Xsig_aug(2, i)/Xsig_aug(4, i)) * (sin(Xsig_aug(3, i) + phiDotDeltaT)
              - sinPhi);
      Xsig_pred(1, i) = Xsig_aug(1, i) 
              + (Xsig_aug(2, i)/Xsig_aug(4, i)) * (-1.0 * cos(Xsig_aug(3, i) + phiDotDeltaT)
              + cosPhi);
      Xsig_pred(2, i) = Xsig_aug(2, i) ;
      Xsig_pred(3, i) = Xsig_aug(3, i) + phiDotDeltaT;
      Xsig_pred(4, i) = Xsig_aug(4, i) ;
    } else {
      // phiDotDeltaT = 0.0 ;
      Xsig_pred(0, i) = Xsig_aug(0, i) 
              + (Xsig_aug(2, i)) * cosPhi * delta_t ;
      Xsig_pred(1, i) = Xsig_aug(1, i) 
              + (Xsig_aug(2, i)) * sinPhi * delta_t ;
      Xsig_pred(2, i) = Xsig_aug(2, i) ;
      Xsig_pred(3, i) = Xsig_aug(3, i) ;
      Xsig_pred(4, i) = Xsig_aug(4, i) ;
    }
    
    Xsig_pred(0, i) = Xsig_pred(0, i) + (0.5) * deltaTsq * cosPhi * Xsig_aug(5, i) ;
    Xsig_pred(1, i) = Xsig_pred(1, i) + (0.5) * deltaTsq * sinPhi * Xsig_aug(5, i) ;
    Xsig_pred(2, i) = Xsig_pred(2, i) + delta_t * Xsig_aug(5, i) ;
    Xsig_pred(3, i) = Xsig_pred(3, i) + (0.5) * deltaTsq * Xsig_aug(6, i) ;
    Xsig_pred(4, i) = Xsig_pred(4, i) + delta_t * Xsig_aug(6, i) ;   
  }
  
  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

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
void predictMeanAndCovariance(MatrixXd Xsig_pred, int n_x, int n_aug, 
        double lambda, VectorXd* x_out, MatrixXd* P_out) {
  //create vector for weights
  VectorXd weights = VectorXd(2 * n_aug + 1);
  
  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);
  
  //set weights
  double oneByLambdaNa = (1.0 / (lambda + n_aug)) ;
 
  weights.fill(0.5 * oneByLambdaNa);
  weights(0) = lambda * oneByLambdaNa; 
  
  //predict state mean    
  x.fill(0.0);  
  for (int i=0; i < 2 * n_aug + 1; i++){
    x = x +  weights(i) * Xsig_pred.col(i) ; 
  } 
  
  //x = Xsig_pred*weights;
  
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }

  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
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
        double std_radr, double std_radphi, double std_radrd, VectorXd* z_out, 
        MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar,n_z_radar);

  //transform sigma points into measurement space
  //calculate mean predicted measurement  
  z_pred.fill(0.0);

  for (int i=0; i < 2 * n_aug + 1; i++){
    //ro 
    Zsig(0, i) = sqrt(Xsig_pred(0,i) * Xsig_pred(0,i) + Xsig_pred(1, i) * Xsig_pred(1, i));
    
    // phi
    Zsig(1,i) = atan2(Xsig_pred(1, i), Xsig_pred(0, i));
    
    //roDot
    Zsig(2, i) = (Xsig_pred(0,i) * cos(Xsig_pred(3,i)) * Xsig_pred(2,i) 
            + Xsig_pred(1,i) * sin(Xsig_pred(3,i)) * Xsig_pred(2,i)) / Zsig(0, i) ;
    
    double wi = weights(i);
    
    //calculate mean predicted measurement    
    z_pred = z_pred + (wi * Zsig.col(i));
  }   

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = atan2 (sin(z_diff(1)), cos(z_diff(1))) ;    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  
  MatrixXd R = MatrixXd(n_z_radar, n_z_radar);  
  R.fill(0.0);
  // sigma px sq
  R(0,0) = std_radr * std_radr ;
  R(1,1) = std_radphi * std_radphi ;
  R(2,2) = std_radrd * std_radrd ;

  S = S + R;  

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

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
void predictLidarMeasurement(MatrixXd Xsig_pred, int n_aug, 
        VectorXd weights, double std_laspx, double std_laspy, VectorXd* z_out, 
        MatrixXd* S_out, MatrixXd* Zsig_out) {
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_laser, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_laser);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_laser,n_z_laser);

  //transform sigma points into measurement space
  //calculate mean predicted measurement  
  z_pred.fill(0.0);

  for (int i=0; i < 2 * n_aug + 1; i++){
    //ro 
    Zsig(0, i) = sqrt(Xsig_pred(0,i) * Xsig_pred(0,i) + Xsig_pred(1, i) * Xsig_pred(1, i));
    
    // phi
    Zsig(1,i) = atan2(Xsig_pred(1, i), Xsig_pred(0, i));
    
    //roDot
    Zsig(2, i) = (Xsig_pred(0,i) * cos(Xsig_pred(3,i)) * Xsig_pred(2,i) 
            + Xsig_pred(1,i) * sin(Xsig_pred(3,i)) * Xsig_pred(2,i)) / Zsig(0, i) ;
    
    double wi = weights(i);
    
    //calculate mean predicted measurement    
    z_pred = z_pred + (wi * Zsig.col(i));
  }   

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = atan2 (sin(z_diff(1)), cos(z_diff(1))) ;    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  
  MatrixXd R = MatrixXd(n_z_laser, n_z_laser);  
  R.fill(0.0);
  // sigma px sq
  R(0,0) = std_laspx * std_laspx ;
  R(1,1) = std_laspy * std_laspy ;


  S = S + R;  

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

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

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z_radar);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd x_diff = Xsig_pred.col(i) - x;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    x_diff(3) = atan2 (sin(x_diff(3)), cos(x_diff(3))) ;    
    x_diff(4) = atan2 (sin(x_diff(4)), cos(x_diff(4))) ;    
    z_diff(1) = atan2 (sin(z_diff(1)), cos(z_diff(1))) ;    
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }  
  
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x, n_z_radar);
  K = Tc * S.inverse();
  
  //update state mean and covariance matrix  
  VectorXd z_diff = (z - z_pred);
  x = x + K * z_diff ;  

    
  //covariance matrix
  P = P - K * S * K.transpose();

  double nis_radar = z_diff.transpose() * S.inverse() * z_diff ; 
  
  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

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

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z_laser);

  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd x_diff = Xsig_pred.col(i) - x;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    x_diff(3) = atan2 (sin(x_diff(3)), cos(x_diff(3))) ;    
    x_diff(4) = atan2 (sin(x_diff(4)), cos(x_diff(4))) ;    
    z_diff(1) = atan2 (sin(z_diff(1)), cos(z_diff(1))) ;    
    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }  
  
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x, n_z_laser);
  K = Tc * S.inverse();
  
  //update state mean and covariance matrix  
  VectorXd z_diff = (z - z_pred);
  x = x + K * z_diff ;  

    
  //covariance matrix
  P = P - K * S * K.transpose();

  double nis_laser = z_diff.transpose() * S.inverse() * z_diff ; 
  
  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  //write result
  *x_out = x;
  *P_out = P;
  *nis = nis_laser;
  
}