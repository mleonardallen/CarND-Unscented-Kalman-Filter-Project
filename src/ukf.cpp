#include <iostream>
#include <math.h>
#include "ukf.h"
#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  n_sig_ = n_aug_ * 2 + 1;

  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;
  var_a_ = std_a_ * std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  var_yawdd_ = std_yawdd_ * std_yawdd_;

  //create vector for weights
  weights_ = VectorXd(n_sig_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < n_sig_; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

VectorXd UKF::ProcessMeasurement(MeasurementPackage *measurement_pack) {

  /*****************************************************************************
   * Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    // first measurement
    x_.fill(0.0);
    x_.head(2) = measurement_pack->getCartesianMeasurement();
    previous_timestamp_ = measurement_pack->timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return x_;
  }

  /*****************************************************************************
   * Prediction
   * Predicts sigma points, the state, and the state covariance matrix.
   ****************************************************************************/

  // dt - expressed in seconds
  double dt = (measurement_pack->timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack->timestamp_;

  MatrixXd x_sig_aug = GenerateSigmaPoints();
  MatrixXd x_sig_pred = SigmaPointPrediction(x_sig_aug, dt);

  x_ = GetSigmaMean(x_sig_pred);

  // Note:
  // P_ is calculated below in the same loop as the other covariances for computational effeciency.
  // This works because Tc and S do not depend on P_

  /*****************************************************************************
   * Update
   * Updates the state and the state covariance matrix using a laser measurement.
   ****************************************************************************/

  // Transform predicted sigma points into measurement space
  MatrixXd z_sig = measurement_pack->transformSigmaPoints(x_sig_pred);
  VectorXd z_pred = GetSigmaMean(z_sig);

  MatrixXd S = MatrixXd(z_sig.rows(), z_sig.rows());
  MatrixXd Tc = MatrixXd(n_x_, z_sig.rows());

  P_.fill(0.0); // Process Covariance
  Tc.fill(0.0); // Cross Correlation
  S.fill(0.0); // Measurement Covariance

  for (int i = 0; i < n_sig_; i++) {

    // x_diff + angle normalization
    VectorXd x_diff = x_sig_pred.col(i) - x_;
    x_diff(3/*phi*/) = atan2(sin(x_diff(3/*phi*/)), cos(x_diff(3/*phi*/)));

    // z_diff, plus angle normalization for radar (internal to radar measurement package class)
    VectorXd z_diff = measurement_pack->getError(z_sig.col(i), z_pred);

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    S = S + weights_(i) * z_diff * z_diff.transpose();
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix  
  S = S + measurement_pack->getMeasurementNoiseCovariance();

  // Kalman
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  // residual + angule normalization
  VectorXd y = measurement_pack->getError(measurement_pack->getRawMeasurement(), z_pred);

  // Update state mean and covariance matrix
  x_ = x_ + K * y;
  P_ = P_ - K * S * K.transpose();

  NIS_ = y.transpose() * Si * y;

  return x_;
}

/**
 * Generate sigma points
 * @return generated sigma points (augmented)
 */
MatrixXd UKF::GenerateSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = var_a_;
  P_aug(n_x_+1, n_x_+1) = var_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

/**
 * Predict sigma points
 * @param augmented sigma points
 * @param delta_t
 * @return predicted sigma points
 */
MatrixXd UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, n_sig_);

  //predict sigma points
  for (int i = 0; i< n_sig_; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;
    double delta_t_2 = delta_t * delta_t;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t_2 * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t_2 * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t_2;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  return Xsig_pred;
}

/**
 * Get mean of given sigma points
 * @param sig -sigma points
 * @return mean
 */
VectorXd UKF::GetSigmaMean(MatrixXd sig) {
  VectorXd mean = VectorXd(sig.rows());
  mean.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    mean += weights_(i) * sig.col(i);
  }
  return mean;  
}

/**
 * Get estimated state in cartesian form - used for RMSE calculation
 * @return estimated state (cartesian)
 */
VectorXd UKF::getCartesianEstimate() {
  VectorXd cartesian = VectorXd(4);

  double x = x_(0);
  double y = x_(1);
  double vx = x_(2) * cos(x_(3));
  double vy = x_(2) * sin(x_(3));

  cartesian << x, y, vx, vy;
  return cartesian;
}

double UKF::getNIS() {
  return NIS_;
}