#ifndef LASER_MEASUREMENT_PACKAGE_H_
#define LASER_MEASUREMENT_PACKAGE_H_

#include "measurement_package.h"
#include "Eigen/Dense"

class LaserMeasurementPackage : public MeasurementPackage {
public:

  /**
  * Constructor.
  */
  LaserMeasurementPackage(std::string);

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

  // Measurement covariance matrix (R)
  static Eigen::MatrixXd R_;

  // Measurement Size
  static int n_z_;

  // Laser measurement noise standard deviation position1 in m
  static double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  static double std_laspy_;

};

#endif /* LASER_MEASUREMENT_PACKAGE_H_ */
