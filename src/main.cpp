#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "ekf_tracker.h"

// Local Types
// -----------------------------------------------------------------------------

typedef std::vector<EkfTracker::Measurement> MeasurementSequence;
typedef std::vector<Eigen::VectorXf> GroundTruthSequence;
typedef std::vector<Eigen::VectorXf> EstimationSequence;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks arguments of the program and exits if the check fails.
// @param[in] argc Number of arguments
// @param[in] argc Array of arguments
void CheckArguments(int argc, char* argv[]) {
  std::string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // Make sure the user has provided input and output files
  if (argc == 1) {
    std::cerr << usage_instructions << std::endl;
  } else if (argc == 2) {
    std::cerr << "Please include an output file." << std::endl
              << usage_instructions << std::endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    std::cerr << "Too many arguments." << std::endl << usage_instructions
              << std::endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

// Checks input and output files and exits if the check fails.
// @param[in] in_file       Input file
// @param[in] in_file_name  Input file name
// @param[in] out_file      Output file
// @param[in] out_file_name Output file name
void CheckFiles(const std::ifstream& in_file,
                const std::string& in_file_name,
                const std::ofstream& out_file,
                const std::string& out_file_name) {
  if (!in_file.is_open()) {
    std::cerr << "Cannot open input file: " << in_file_name << std::endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    std::cerr << "Cannot open output file: " << out_file_name << std::endl;
    exit(EXIT_FAILURE);
  }
}

// Loads input data.
// @param[in] in_file               Input file
// @param[out] measurement_sequence  Container for measurements data
// @param[out] ground_truth_sequence Container for ground truth data
void LoadData(std::ifstream& in_file,
              MeasurementSequence& measurement_sequence,
              GroundTruthSequence& ground_truth_sequence) {
  std::string line;
  while (std::getline(in_file, line)) {
    // Each line represents a measurement at a timestamp
    std::istringstream iss(line);
    long long timestamp;
    std::string sensor_type;
    EkfTracker::Measurement measurement;

    // Read the first element from the current line
    iss >> sensor_type;
    if (sensor_type == "L") {
      // Laser measurement
      measurement.sensor_type = EkfTracker::Measurement::SensorType::kLidar;
      measurement.value = Eigen::VectorXf(2);
      float x;
      float y;
      iss >> x >> y >> timestamp;
      measurement.value << x, y;
      measurement.timestamp = timestamp;
      measurement_sequence.push_back(measurement);
    } else if (sensor_type == "R") {
      // Radar measurement
      measurement.sensor_type = EkfTracker::Measurement::SensorType::kRadar;
      measurement.value = Eigen::VectorXf(3);
      float rho;
      float phi;
      float rho_dot;
      iss >> rho >> phi >> rho_dot >> timestamp;
      measurement.value << rho, phi, rho_dot;
      measurement.timestamp = timestamp;
      measurement_sequence.push_back(measurement);
    }

    // Read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
    Eigen::VectorXf groundTruth(4);
    groundTruth << x_gt, y_gt, vx_gt, vy_gt;
    ground_truth_sequence.push_back(groundTruth);
  }
}

// Dumps all estimations, measurements and ground truth data to a file.
// @param[in]  measurement_sequence  Container with measurements data
// @param[in]  ground_truth_sequence Container with ground truth data
// @param[in]  estimation_sequence   Container with esimations data
// @param[out] out_file              Output file
void DumpData(const EstimationSequence& estimation_sequence,
              const MeasurementSequence& measurement_sequence,
              const GroundTruthSequence& ground_truth_sequence,
              std::ofstream& out_file) {
  assert(estimation_sequence.size() == measurement_sequence.size()
      && measurement_sequence.size() == ground_truth_sequence.size());
  for (auto i = 0; i < measurement_sequence.size(); ++i) {
    auto estimation = estimation_sequence[i];
    auto measurement = measurement_sequence[i];
    auto groundTruth = ground_truth_sequence[i];

    // Output the estimation
    out_file << estimation(0) << "\t" << estimation(1) << "\t"
             << estimation(2) << "\t" << estimation(3) << "\t";

    // Output the measurements
    if (measurement.sensor_type
        == EkfTracker::Measurement::SensorType::kLidar) {
      out_file << measurement.value(0) << "\t" << measurement.value(1) << "\t";
    } else {
      auto rho = measurement.value(0);
      auto phi = measurement.value(1);
      out_file << rho * cos(phi) << "\t" << rho * sin(phi) << "\t";
    }

    // Output the ground truth
    out_file << groundTruth(0) << "\t" << groundTruth(1) << "\t"
             << groundTruth(2) << "\t" << groundTruth(3) << "\n";
  }
}

// Calculates RMSE of the estimations and ground truth data.
// @param[in] estimation_sequence   Container with esimations data
// @param[in] ground_truth_sequence Container with ground truth data
// @return The Eigen vector containing the RMSE values for px, py, vs, vy
Eigen::VectorXf CalculateRmse(
  const EstimationSequence& estimation_sequence,
  const GroundTruthSequence& ground_truth_sequence) {

  Eigen::VectorXf rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimation vector size should equal ground truth vector size
  if (estimation_sequence.size() != ground_truth_sequence.size()
      || estimation_sequence.empty()) {
    std::cout << "Invalid input!" << std::endl;
    return rmse;
  }
  // Accumulate squared differences
  for (auto i = 0; i < estimation_sequence.size(); ++i) {
    rmse = rmse.array()
      + (estimation_sequence[i] - ground_truth_sequence[i]).array().pow(2);
  }
  rmse = (rmse / estimation_sequence.size()).array().sqrt();

  return rmse;
}

// main
// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {

  CheckArguments(argc, argv);

  // Open and check files
  std::ifstream in_file(argv[1], std::ifstream::in);
  std::ofstream out_file(argv[2], std::ofstream::out);

  CheckFiles(in_file, argv[1], out_file, argv[2]);

  // Containers for loading measurements and ground truth data
  std::vector<EkfTracker::Measurement> measurement_sequence;
  std::vector<Eigen::VectorXf> ground_truth_sequence;

  // Load input data and close the file
  LoadData(in_file, measurement_sequence, ground_truth_sequence);
  if (in_file.is_open()) {
    in_file.close();
  }

  // Generate estimation data
  EkfTracker ekf_tracker;
  EstimationSequence estimation_sequence;
  std::transform(measurement_sequence.begin(), measurement_sequence.end(),
                 std::back_inserter(estimation_sequence),
                 ekf_tracker);

  // Dump all data and close the file
  DumpData(estimation_sequence, measurement_sequence, ground_truth_sequence,
           out_file);
  if (out_file.is_open()) {
    out_file.close();
  }

  // Compute the accuracy (RMSE)
  std::cout << "RMSE" << std::endl;
  std::cout << CalculateRmse(estimation_sequence, ground_truth_sequence)
            << std::endl;

  return 0;
}
