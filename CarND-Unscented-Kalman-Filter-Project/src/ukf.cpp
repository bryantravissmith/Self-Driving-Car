#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 3.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.2;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  radar_counter_ = 0;
  laser_counter_ = 0;
  nis_laser_above_thresh_counter_ = 0;
  nis_radar_above_thresh_counter_ = 0;
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  Xsig_pred_  = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2 * n_aug_ + 1);

  for(int i = 0; i < 2*n_aug_+1; i++){
      if(i == 0){
        weights_(i) = lambda_ / (lambda_ + n_aug_);
      } else {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
      }
  }

  is_initialized_ = false;

  time_us_ = 0;

  H_ = MatrixXd(2, n_x_);
  H_.fill(0);
  H_(0,0) = 1;
  H_(1,1) = 1;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << pow(std_laspx_, 2), 0, 0, pow(std_laspy_, 2);

  R_radar_ = MatrixXd(3, 3);
  R_radar_.fill(0);
  R_radar_(0,0) = pow(std_radr_, 2);
  R_radar_(1,1) = pow(std_radphi_, 2);
  R_radar_(2,2) = pow(std_radrd_, 2);
}

UKF::~UKF() {}

void UKF::AdjustAngle(double *angle) {
    while (*angle > M_PI) *angle -=2.*M_PI;
    while (*angle < -M_PI) *angle +=2.*M_PI;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if( !is_initialized_ ){
    x_ = VectorXd(n_x_);
    x_.fill(0);

    P_ = MatrixXd::Identity(n_x_, n_x_);
    P_(0,0) = 0.1;
    P_(1,1) = 0.1;


    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
      float py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
      float theta = meas_package.raw_measurements_[1];
      x_ << px, py, 0, theta, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }


  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in second
    time_us_ = meas_package.timestamp_;
    this->Prediction(dt);
    this->UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ ){
    // Laser updates
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in second
    time_us_ = meas_package.timestamp_;
    this->Prediction(dt);
    this->UpdateLidar(meas_package);
  }

  return;
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
  VectorXd x_aug = VectorXd(7);
  x_aug.fill(0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = pow(std_a_, 2);
  P_aug(6, 6) = pow(std_yawdd_, 2);
  //create sigma point matrix

  MatrixXd A = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0);
  Xsig_aug.col(0) = x_aug;

  for(int i = 0; i < n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i);
  }

  for(int i = 0; i < 2*n_aug_+1; i++){
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    // Precalculate sin and cos for optimization
    double sin_yaw = sin(yaw);
    double cos_yaw = cos(yaw);
    double arg = yaw + yawd*delta_t;

    // Predicted state values
    double px_p, py_p;
    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
        double v_yawd = v/yawd;
          px_p = p_x + v_yawd * (sin(arg) - sin_yaw);
          py_p = p_y + v_yawd * (cos_yaw - cos(arg) );
    } else {
        double v_delta_t = v*delta_t;
          px_p = p_x + v_delta_t*cos_yaw;
          py_p = p_y + v_delta_t*sin_yaw;
    }
    double v_p = v;
    double yaw_p = arg;
    double yawd_p = yawd;

    // Add noise
    px_p += 0.5*nu_a*pow(delta_t,2)*cos_yaw;
    py_p += 0.5*nu_a*pow(delta_t,2)*sin_yaw;
    v_p += nu_a*delta_t;
    yaw_p += 0.5*nu_yawdd*pow(delta_t,2);
    yawd_p += nu_yawdd*delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  x_ = Xsig_pred_ * weights_;
  P_.fill(0);

  for(int i=0; i < 2*n_aug_+1 ; i++){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    AdjustAngle(&x_diff(3));
    P_ += weights_(i) * ( x_diff * x_diff.transpose() );
  }
  return;

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
  int n_z = 2;
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0);

  for(int i = 0; i < 2 * n_aug_ + 1; i++){
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0);
  for(int i = 0; i < 2 * n_aug_ + 1; i++){
     VectorXd z_diff = Zsig.col(i) - z;
     S += weights_(i) * z_diff * z_diff.transpose();
  }
  S += R_laser_;

  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);

  for(int i = 0; i < 2 * n_aug_ + 1 ; i ++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //AdjustAngle(&x_diff(3));
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  VectorXd z_diff = z - z_pred;

  float NIS;
  NIS = z_diff.transpose() * S.inverse() * z_diff;

  laser_counter_++;
  if(NIS > 5.99) nis_laser_above_thresh_counter_++;
  std::cout << "NIS: " << NIS << ", " << nis_laser_above_thresh_counter_ / laser_counter_ << std::endl;
  std::cout << x_(3);
  MatrixXd K;
  K = Tc * S.inverse();

  x_ += K * z_diff;
  //AdjustAngle(&x_(3));
  P_ -= K * S * K.transpose();

  return;
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
  int n_z = 3;
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  Zsig.fill(0);

  for(int i = 0; i < 2 * n_aug_ + 1; i++){
      Zsig(0,i) = sqrt(pow(Xsig_pred_(0,i), 2) + pow(Xsig_pred_(1,i), 2));
      Zsig(1,i) = atan2(Xsig_pred_(1,i), Xsig_pred_(0,i));
      Zsig(2,i) = Xsig_pred_(0,i) * cos(Xsig_pred_(3,i)) * Xsig_pred_(2,i) / Zsig(0,i);
      Zsig(2,i) += Xsig_pred_(1,i) * sin(Xsig_pred_(3,i)) * Xsig_pred_(2,i) / Zsig(0,i);
  }

 //mean predicted measurement
 VectorXd z_pred = VectorXd(n_z);
 z_pred = Zsig * weights_;

 //measurement covariance matrix S
 MatrixXd S = MatrixXd(n_z,n_z);
 S.fill(0);
 for(int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    AdjustAngle(&z_diff(1));
     S += weights_(i) * ( z_diff * z_diff.transpose() );
 }
 S += R_radar_;

 MatrixXd Tc = MatrixXd(n_x_, n_z);
 Tc.fill(0);

 for(int i = 0; i < 2 * n_aug_ + 1 ; i ++){
   VectorXd z_diff = Zsig.col(i) - z_pred;
   AdjustAngle(&z_diff(1));

   VectorXd x_diff = Xsig_pred_.col(i) - x_;
   AdjustAngle(&x_diff(3));

   Tc += weights_(i) * ( x_diff * z_diff.transpose() );
 }

 VectorXd z_diff = z - z_pred;
 AdjustAngle(&z_diff(1));

 float NIS;
 NIS = z_diff.transpose() * S.inverse() * z_diff;

 radar_counter_++;
 if(NIS > 7.8) nis_radar_above_thresh_counter_++;
 std::cout << "NIS: " << NIS << ", " << nis_radar_above_thresh_counter_ / radar_counter_ << std::endl;

 MatrixXd K;
 K = Tc * S.inverse();

 x_ += K * z_diff;
 P_ -= K * S * K.transpose();

}
