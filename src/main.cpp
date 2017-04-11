#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
//#include <stdlib.h>
#include <vector>
#include "Eigen/Dense"
//#include "FusionEKF.h"
#include "kalman_filter.h"
#include "ground_truth_package.h"
#include "measurement_package.h"


void check_arguments(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    std::cerr << usage_instructions << std::endl;
  } else if (argc == 2) {
    std::cerr << "Please include an output file.\n" << usage_instructions << std::endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}


void check_files(std::ifstream& in_file, std::string& in_name,
                 std::ofstream& out_file, std::string& out_name) {
  if (!in_file.is_open()) {
    std::cerr << "Cannot open input file: " << in_name << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    std::cerr << "Cannot open output file: " << out_name << std::endl;
    exit(EXIT_FAILURE);
  }
}


Eigen::VectorXf CalculateRmse(const std::vector<Eigen::VectorXf>& estimations,
                              const std::vector<Eigen::VectorXf>& ground_truth) {
  Eigen::VectorXf rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.empty()) {
    std::cout << "Invalid input!" << std::endl;
    return rmse;
  }

  //  The estimation vector size should not be zero
//  assert(!estimations.empty());

  // Accumulate squared differences
  for (auto i = 0; i < estimations.size(); ++i) {
    Eigen::VectorXf difference = estimations[i] - ground_truth[i];
    difference = difference.array() * difference.array();
    rmse += difference;
//    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().pow(2);
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
//  rmse = (rmse / estimations.size()).array().sqrt();

  return rmse;
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  std::string in_file_name_ = argv[1];
  std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);

  std::string out_file_name_ = argv[2];
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  std::vector<MeasurementPackage> measurement_pack_list;
  std::vector<GroundTruthPackage> gt_pack_list;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    std::string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    std::istringstream iss(line);
    long long timestamp;

    // Read the first element from the current line
    iss >> sensor_type;
    if (sensor_type == "L") {
      // LASER MEASUREMENT
      // Read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = Eigen::VectorXf(2);
      float x;
      float y;
      iss >> x >> y >> timestamp;
      meas_package.raw_measurements_ << x, y;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type == "R") {
      // RADAR MEASUREMENT
      // Read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = Eigen::VectorXf(3);
      float rho;
      float phi;
      float rho_dot;
      iss >> rho >> phi >> rho_dot >> timestamp;
      meas_package.raw_measurements_ << rho, phi, rho_dot;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // Read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    gt_package.gt_values_ = Eigen::VectorXf(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  KalmanFilter ekf_tracker;

  // used to compute the RMSE later
  std::vector<Eigen::VectorXf> estimations;
  std::vector<Eigen::VectorXf> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    ekf_tracker.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    Eigen::VectorXf x = ekf_tracker.GetState();
//    out_file_ << std::fixed << std::showpoint << std::setprecision(2) << std::setw(10);
    out_file_ << x(0) << "\t";
    out_file_ << x(1) << "\t";
    out_file_ << x(2) << "\t";
    out_file_ << x(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(x);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // Compute the accuracy (RMSE)
  std::cout << "RMSE" << std::endl;
  std::cout << CalculateRmse(estimations, ground_truth) << std::endl;

  // Close files
  if (out_file_.is_open()) {
    out_file_.close();
  }
  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
