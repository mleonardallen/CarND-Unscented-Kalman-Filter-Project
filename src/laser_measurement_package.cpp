#include "Eigen/Dense"
#include <sstream>
#include <iostream>

#include "laser_measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int LaserMeasurementPackage::n_z_ = 2;

// Laser measurement noise standard deviation position1 in m
double LaserMeasurementPackage::std_px_ = 0.15;

// Laser measurement noise standard deviation position2 in m
double LaserMeasurementPackage::std_py_ = 0.15;

/**
 * Measurement covariance (R)
 */
MatrixXd LaserMeasurementPackage::R_ = (
  MatrixXd(n_z_, n_z_) << std_px_ * std_px_, 0,
    0, std_py_ * std_py_
).finished();

/*
 * Constructor
 *
 * @param line Sensor data
 */
LaserMeasurementPackage::LaserMeasurementPackage(string line) {

  istringstream iss(line);
  sensor_type_ = MeasurementPackage::LASER;

  // Raw measurement
  raw_measurements_ = VectorXd(n_z_);
  double x;
  double y;
  string sensor_type;

  iss >> sensor_type >> x >> y >> timestamp_;
  raw_measurements_ << x, y;

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
 * Get cartesian measurement
 *
 * @return Measurement
 */
VectorXd LaserMeasurementPackage::getCartesianMeasurement() {
  return raw_measurements_;
}

/*
 * Get raw measurement
 *
 * @return Measurement
 */
VectorXd LaserMeasurementPackage::getRawMeasurement() {
  return raw_measurements_;
}

/**
 * Get error/diff between two vectors
 * @param pred
 * @param mean
 * @return error/diff
 */
MatrixXd LaserMeasurementPackage::getError(const VectorXd& pred, const VectorXd& mean) {
  VectorXd y = pred - mean;
  return y;
}

/**
 * Transform state sigma points into measurement space
 * @param sig - state sigma points
 * @return z_sig - measurement sigma points
 */
MatrixXd LaserMeasurementPackage::transformSigmaPoints(const MatrixXd& sig) {
  int n_sig = sig.cols();
  MatrixXd z_sig = MatrixXd(n_z_, n_sig);
  for (int i = 0; i < n_sig; i++) {
    z_sig.col(i) = sig.col(i).head(n_z_);
  }
  return z_sig;
}

/**
 * Get measurement covariance matrix (R)
 *
 * @return Measurement noise covariance matrix (R)
 */
MatrixXd LaserMeasurementPackage::getMeasurementNoiseCovariance() {
  return R_;
}
