#ifndef UKF_H
#define UKF_H
#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

class UKF {
public:

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  Eigen::VectorXd ProcessMeasurement(MeasurementPackage *meas_package);

  // Get estimated state (cartesian)
  Eigen::VectorXd getCartesianEstimate();

  // 
  double getNIS();

private:

  // Generate sigma points (augmented)
  Eigen::MatrixXd GenerateSigmaPoints();

  // Predict sigma points
  Eigen::MatrixXd SigmaPointPrediction(Eigen::MatrixXd Xsig_aug, double dt);

  // Get mean of sigma points
  Eigen::VectorXd GetSigmaMean(Eigen::MatrixXd sig);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long time_us_;

  // previous timestamp
  long previous_timestamp_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  double var_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  double var_yawdd_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Number of sigma points
  int n_sig_;

  // Sigma point spreading parameter
  double lambda_;

  // the current NIS
  double NIS_;

};

#endif /* UKF_H */
