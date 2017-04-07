#ifndef RADAR_MEASUREMENT_PACKAGE_H_
#define RADAR_MEASUREMENT_PACKAGE_H_

#include "measurement_package.h"
#include "Eigen/Dense"

class RadarMeasurementPackage : public MeasurementPackage {
public:

  /**
  * Constructor.
  */
  RadarMeasurementPackage(std::string);

  // Get Measurement (Cartesian)
  Eigen::VectorXd getCartesianMeasurement();

  // Get Measurement (Raw)
  Eigen::VectorXd getRawMeasurement();

  // Get Error/Diff
  Eigen::MatrixXd getError(const Eigen::VectorXd& pred, const Eigen::VectorXd& mean);

  // Transform sigma points into measurement space
  Eigen::MatrixXd transformSigmaPoints(const Eigen::MatrixXd&);

  // Get Measurement Noise Covariance Matrix (R)
  Eigen::MatrixXd getMeasurementNoiseCovariance();


private:

  // Measurement Covariance Matrix
  static Eigen::MatrixXd R_;

  // Measurement Size
  static int n_z_;

  // Radar measurement noise standard deviation radius in m
  static double std_r_;

  // Radar measurement noise standard deviation angle in rad
  static double std_phi_;

  // Radar measurement noise standard deviation radius change in m/s
  static double std_rd_;

  // Convert Measurement from Cartesian to Polar
  Eigen::VectorXd toPolar(const Eigen::VectorXd&);

  // Convert Measurement from Polar to Cartesian
  Eigen::VectorXd toCartesian(const Eigen::VectorXd&);

};

#endif /* RADAR_MEASUREMENT_PACKAGE_H_ */
