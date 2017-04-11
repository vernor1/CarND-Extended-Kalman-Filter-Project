#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "ekf_tracker.h"


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

  // Accumulate squared differences
  for (auto i = 0; i < estimations.size(); ++i) {
    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().pow(2);
  }

  rmse = (rmse / estimations.size()).array().sqrt();

  return rmse;
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  std::string in_file_name_ = argv[1];
  std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);

  std::string out_file_name_ = argv[2];
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  std::vector<EkfTracker::Measurement> measurementSequence;
  std::vector<Eigen::VectorXf> groundTruthSequence;

  std::string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    std::string sensor_type;
    EkfTracker::Measurement measurement;
    std::istringstream iss(line);
    long long timestamp;

    // Read the first element from the current line
    iss >> sensor_type;
    if (sensor_type == "L") {
      // LASER MEASUREMENT
      // Read measurements at this timestamp
      measurement.sensor_type = EkfTracker::Measurement::SensorType::kLidar;
      measurement.value = Eigen::VectorXf(2);
      float x;
      float y;
      iss >> x >> y >> timestamp;
      measurement.value << x, y;
      measurement.timestamp = timestamp;
      measurementSequence.push_back(measurement);
    } else if (sensor_type == "R") {
      // RADAR MEASUREMENT
      // Read measurements at this timestamp
      measurement.sensor_type = EkfTracker::Measurement::SensorType::kRadar;
      measurement.value = Eigen::VectorXf(3);
      float rho;
      float phi;
      float rho_dot;
      iss >> rho >> phi >> rho_dot >> timestamp;
      measurement.value << rho, phi, rho_dot;
      measurement.timestamp = timestamp;
      measurementSequence.push_back(measurement);
    }

    // Read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    Eigen::VectorXf groundTruth(4);
    groundTruth << x_gt, y_gt, vx_gt, vy_gt;
    groundTruthSequence.push_back(groundTruth);
  }

  // Create a Fusion EKF instance
  EkfTracker ekf_tracker;

  // used to compute the RMSE later
  std::vector<Eigen::VectorXf> estimations;

  //Call the EKF-based fusion
  size_t N = measurementSequence.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    ekf_tracker.ProcessMeasurement(measurementSequence[k]);

    // output the estimation
    Eigen::VectorXf x = ekf_tracker.GetState();
//    out_file_ << std::fixed << std::showpoint << std::setprecision(2) << std::setw(10);
    out_file_ << x(0) << "\t";
    out_file_ << x(1) << "\t";
    out_file_ << x(2) << "\t";
    out_file_ << x(3) << "\t";

    // output the measurements
    if (measurementSequence[k].sensor_type
        == EkfTracker::Measurement::SensorType::kLidar) {
      // Output the estimation
      out_file_ << measurementSequence[k].value(0) << "\t";
      out_file_ << measurementSequence[k].value(1) << "\t";
    } else {
      // Output the estimation in the cartesian coordinates
      float ro = measurementSequence[k].value(0);
      float phi = measurementSequence[k].value(1);
      out_file_ << ro * cos(phi) << "\t";
      out_file_ << ro * sin(phi) << "\t";
    }

    // Output the ground truth
    out_file_ << groundTruthSequence[k](0) << "\t";
    out_file_ << groundTruthSequence[k](1) << "\t";
    out_file_ << groundTruthSequence[k](2) << "\t";
    out_file_ << groundTruthSequence[k](3) << "\n";

    estimations.push_back(x);
  }

  // Compute the accuracy (RMSE)
  std::cout << "RMSE" << std::endl;
  std::cout << CalculateRmse(estimations, groundTruthSequence) << std::endl;

  // Close files
  if (out_file_.is_open()) {
    out_file_.close();
  }
  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
