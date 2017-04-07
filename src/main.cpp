#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>

#include "Eigen/Dense"
#include "ukf.h"
#include "tools.h"
#include "measurement_package.h"

using namespace std;
using Eigen::VectorXd;

void check_arguments(int argc, char* argv[]) {

  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3 || argc == 4) { // 3rd parameter is for measurement type
    has_valid_args = true;
  } else if (argc > 4) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  // measurement type argument
  int measurement_type = MeasurementPackage::BOTH;
  if (argc == 4) {
    std::istringstream iss(argv[3]);
    iss >> measurement_type;
  }

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  UKF ukf;
  MeasurementPackage *meas_package;
  VectorXd estimation;
  VectorXd measurement;
  VectorXd gt_values;
  string line;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  while (getline(in_file_, line)) {

    // Create measurement package from sensor data
    meas_package = MeasurementPackage::create(line);
    measurement = meas_package->getCartesianMeasurement();

    // Filter out measurement types other than passed in type.
    // LIDAR = 1, RADAR = 2
    if (measurement_type != MeasurementPackage::BOTH
      and meas_package->sensor_type_ != measurement_type
    ) {
      continue;
    }

    gt_values = meas_package->getGroundTruth();

    // Call the UKF-based fusion
    estimation = ukf.ProcessMeasurement(meas_package);

    // Output estimation and measurements
    out_file_ << estimation(0) << "\t"; // px
    out_file_ << estimation(1) << "\t"; // py
    out_file_ << estimation(2) << "\t"; // vel_abs
    out_file_ << estimation(3) << "\t"; // yaw_angle
    out_file_ << estimation(4) << "\t"; // yaw_rate

    out_file_ << measurement(0) << "\t"; // px
    out_file_ << measurement(1) << "\t"; // py

    // output the ground truth packages
    out_file_ << gt_values(0) << "\t"; // px
    out_file_ << gt_values(1) << "\t"; // py
    out_file_ << gt_values(2) << "\t"; // vx
    out_file_ << gt_values(3) << "\t"; // vy

    out_file_ << ukf.getNIS() << "\t"; // NIS

    out_file_ << "\n";

    // Used to calculate RMSE
    estimations.push_back(ukf.getCartesianEstimate());
    ground_truth.push_back(gt_values);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
