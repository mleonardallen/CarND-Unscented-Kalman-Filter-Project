#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:

  long timestamp_;

  enum SensorType{
    BOTH,
    LASER,
    RADAR
  } sensor_type_;

  // Raw Measurements
  Eigen::VectorXd raw_measurements_;

  // Ground Truth Values
  Eigen::VectorXd gt_values_;

  // Get Measurement (Raw)
  Eigen::VectorXd getGroundTruth();

  // MeasurementPackage Factory
  static MeasurementPackage *create(std::string);

  // Get Measurement (Cartesian)
  virtual Eigen::VectorXd getCartesianMeasurement() = 0;

  // Get Measurement (Raw)
  virtual Eigen::VectorXd getRawMeasurement() = 0;

  // Transform sigma points into measurement space
  virtual Eigen::MatrixXd transformSigmaPoints(const Eigen::MatrixXd&) = 0;

  // Get Measurement Noise Covariance Matrix (R)
  virtual Eigen::MatrixXd getMeasurementNoiseCovariance() = 0;

  virtual Eigen::MatrixXd getError(const Eigen::VectorXd& pred, const Eigen::VectorXd& mean) = 0;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
