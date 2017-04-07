#include "Eigen/Dense"
#include <sstream>
#include <iostream>
#include <math.h>

#include "radar_measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Measurement space
int RadarMeasurementPackage::n_z_ = 3;

// Radar measurement noise standard deviation radius in m
double RadarMeasurementPackage::std_r_ = 0.3;

// Radar measurement noise standard deviation angle in rad
double RadarMeasurementPackage::std_phi_ = 0.03;

// Radar measurement noise standard deviation radius change in m/s
double RadarMeasurementPackage::std_rd_ = 0.3;

/**
 * Measurement covariance matrix (R)
 */
MatrixXd RadarMeasurementPackage::R_ = (
  MatrixXd(n_z_, n_z_) << std_r_*std_r_, 0, 0,
    0, std_phi_*std_phi_, 0,
    0, 0, std_rd_*std_rd_
).finished();

/**
 * Constructor
 *
 * @param line Sensor data
 */
RadarMeasurementPackage::RadarMeasurementPackage(string line) {

  istringstream iss(line);

  sensor_type_ = MeasurementPackage::RADAR;
  raw_measurements_ = VectorXd(n_z_);

  double ro;
  double phi;
  double ro_dot;

  string sensor_type;

  iss >> sensor_type >> ro >> phi >> ro_dot >> timestamp_;
  raw_measurements_ << ro, phi, ro_dot;

  // Ground truth
  gt_values_ = VectorXd(4);
  double x_gt;
  double y_gt;
  double vx_gt;
  double vy_gt;

  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
}

/*
 * Note: Converts measurement from polar to cartesian
 * @return measurement (cartesian)
 */
VectorXd RadarMeasurementPackage::getCartesianMeasurement() {
  return toCartesian(raw_measurements_);
}

/*
 * Note: Converts measurement from polar to cartesian
 * @return measurement (polar)
 */
VectorXd RadarMeasurementPackage::getRawMeasurement() {
  return raw_measurements_;
}

/**
 * Get error/diff between two vectors
 * Includes angle smoothing (phi)
 *
 * @param pred
 * @param mean
 * @return error/diff
 */
MatrixXd RadarMeasurementPackage::getError(const VectorXd& pred, const VectorXd& mean) {
  VectorXd y = pred - mean;
  y(1) = atan2(sin(y(1)), cos(y(1)));

  return y;
}

/**
 * Get measurement noise covariance matrix (R)
 *
 * @return Measurement noise covariance matrix (R)
 */
MatrixXd RadarMeasurementPackage::getMeasurementNoiseCovariance() {
  return R_;
}

/**
 * Transform state sigma points into measurement space
 * @param sig - state sigma points
 * @return z_sig - measurement sigma points
 */
MatrixXd RadarMeasurementPackage::transformSigmaPoints(const MatrixXd& sig) {
  int n_sig = sig.cols();
  MatrixXd Zsig = MatrixXd(n_z_, n_sig);
  for (int i = 0; i < n_sig; i++) {
    Zsig.col(i) = toPolar(sig.col(i));
  }
  return Zsig;
}

/**
 * Convert polar to cartesian
 *
 * @param measurement (polar)
 * @return measurement (cartesian)
 */
VectorXd RadarMeasurementPackage::toCartesian(const VectorXd& measurement) {
  VectorXd cartesian = VectorXd(2);

  double ro = measurement(0);
  double phi = measurement(1);

  cartesian << ro * cos(phi), ro * sin(phi);
  return cartesian;
}

/**
 * Convert cartesian to polar
 *
 * @param x The state or measurement (cartesian)
 * @return The state or measurement (polar)
 */
VectorXd RadarMeasurementPackage::toPolar(const VectorXd& x) {

  VectorXd x_polar = VectorXd(3);

  // extract values for better readibility
  double px = x(0);
  double py = x(1);
  double v  = x(2);
  double yaw = x(3);

  // get x and y components of velocity
  double vx = cos(yaw)*v;
  double vy = sin(yaw)*v;
  double ro = sqrt(px * px + py * py);

  // avoid division by zero
  if(fabs(ro) < 0.0001) {
    x_polar << 0, 0, 0;
    return x_polar;
    ro = 0.0001;
  }

  double phi = atan2(py, px);
  double ro_dot = (px*vx + py*vy) / ro;
  
  x_polar << ro, phi, ro_dot;
  return x_polar;
}
