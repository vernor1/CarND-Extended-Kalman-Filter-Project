#include "ekf_tracker.h"
#include <iostream>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

const float kNoiseAx = 9;
const float kNoiseAy = 9;
const float kEpsilon = 1e-6;

// Measurement matrix and its transposed variant
const Eigen::MatrixXf kH((Eigen::MatrixXf(2, 4)
  << 1, 0, 0, 0,
     0, 1, 0, 0).finished());
const auto kHt = kH.transpose();

// Measurement covariance matrices
const Eigen::MatrixXf kRlidar((Eigen::MatrixXf(2, 2)
  << 0.0225, 0,
     0,      0.0225).finished());
const Eigen::MatrixXf kRradar((Eigen::MatrixXf(3, 3)
  << 0.09, 0,      0,
     0,    0.0009, 0,
     0,    0,      0.09).finished());

// Initial state covariance matrix
const Eigen::MatrixXf kPinitial((Eigen::MatrixXf(4, 4)
  << 1, 0, 0, 0,
     0, 1, 0, 0,
     0, 0, 1000, 0,
     0, 0, 0, 1000).finished());

// Local Helper-Functions
// -----------------------------------------------------------------------------

Eigen::MatrixXf CalculateJacobian(const Eigen::VectorXf& x_state) {
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  Eigen::MatrixXf hj(Eigen::MatrixXf::Zero(3, 4));
  auto sum_of_squares = px * px + py * py;

  if (std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon) {
    return hj;
  }

  auto sqrt_of_sum_of_squares = std::sqrt(sum_of_squares);
  auto sum_of_squares_to_3_by_2 = sum_of_squares * sqrt_of_sum_of_squares;
  auto px_by_sqrt = px / sqrt_of_sum_of_squares;
  auto py_by_sqrt = py / sqrt_of_sum_of_squares;

  // Compute the Jacobian matrix
  hj(0, 0) = px_by_sqrt;
  hj(0, 1) = py_by_sqrt;
  hj(1, 0) = - py / sum_of_squares;
  hj(1, 1) = px / sum_of_squares;
  hj(2, 0) = py * (vx * py - vy * px) / sum_of_squares_to_3_by_2;
  hj(2, 1) = px * (vy * px - vx * py) / sum_of_squares_to_3_by_2;
  hj(2, 2) = px_by_sqrt;
  hj(2, 3) = py_by_sqrt;

  return hj;
}

Eigen::VectorXf ConvertCartesianToPolar(const Eigen::VectorXf& cartesian) {
  auto px = cartesian(0);
  auto py = cartesian(1);
  auto vx = cartesian(2);
  auto vy = cartesian(3);

  Eigen::VectorXf polar(Eigen::VectorXf::Zero(3));

  if (std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon) {
    return polar;
  }

  auto sqrt_of_sum_of_squares = std::sqrt(px * px + py * py);

  polar(0) = sqrt_of_sum_of_squares;;
  polar(1) = std::atan2(py, px);
  polar(2) = (px * vx + py * vy) / sqrt_of_sum_of_squares;

  return polar;
}

Eigen::VectorXf ConvertPolarToCartesian(const Eigen::VectorXf& polar) {
  auto rho = polar(0);
  auto phi = polar(1);

  Eigen::VectorXf cartesian(4);
  cartesian << rho * std::cos(phi), rho * std::sin(phi), 0, 0;

  return cartesian;
}

} // namespace

// Public Members
// -----------------------------------------------------------------------------

EkfTracker::EkfTracker(const Measurement& measurement)
  : previous_timestamp_(measurement.timestamp),
    x_(Eigen::VectorXf::Zero(4)),
    p_(kPinitial) {

    // Initialize the state with the first measurement
    if (measurement.sensor_type == Measurement::SensorType::kRadar) {
      x_ = ConvertPolarToCartesian(measurement.value);
    }
    else {
      x_ << measurement.value(0), measurement.value(1), 0, 0;
    }
}

void EkfTracker::ProcessMeasurement(const Measurement& measurement) {
  // Time elapsed between the current and previous measurements in seconds
  float dt = (measurement.timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement.timestamp;

  //  Prediction
  Predict(dt);

  // Update
  if (measurement.sensor_type == Measurement::SensorType::kRadar) {
    RadarUpdate(measurement.value);
  } else {
    LidarUpdate(measurement.value);
  }

  std::cout << "x" << std::endl << x_ << std::endl;
  std::cout << "P" << std::endl << p_ << std::endl;
}

// Private Members
// -----------------------------------------------------------------------------

void EkfTracker::Predict(float dt) {
  // Define the state transition matrix
  Eigen::MatrixXf f(4, 4);
  f  << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Define the process covariance matrix Q
  Eigen::MatrixXf q(4, 4);
  auto dt2 = dt * dt;
  auto dt3 = dt2 * dt;
  auto dt4 = dt3 * dt;
  q << dt4 / 4 * kNoiseAx, 0, dt3 / 2 * kNoiseAx, 0,
       0, dt4 / 4 * kNoiseAy, 0, dt3 / 2 * kNoiseAy,
       dt3 / 2 * kNoiseAx, 0, dt2 * kNoiseAx, 0,
       0, dt3 / 2 * kNoiseAy, 0, dt2 * kNoiseAy;

  // Predict
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + q;
}

void EkfTracker::LidarUpdate(const Eigen::VectorXf& z) {
  Eigen::VectorXf y = z - kH * x_;
  Eigen::MatrixXf kHt = kH.transpose();
  Eigen::MatrixXf s = kH * p_ * kHt + kRlidar;
  Eigen::MatrixXf k = p_ * kHt * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  auto x_size = x_.size();
  auto i = Eigen::MatrixXf::Identity(x_size, x_size);
  p_ = (i - k * kH) * p_;
}

void EkfTracker::RadarUpdate(const Eigen::VectorXf& z) {
  // y = z - h(x)
  Eigen::VectorXf y = z - ConvertCartesianToPolar(x_);

  // Hj instead of H for calcualting S, K, P
  Eigen::MatrixXf hj = CalculateJacobian(x_);
  Eigen::MatrixXf hjt = hj.transpose();
  Eigen::MatrixXf s = hj * p_ * hjt + kRradar;
  Eigen::MatrixXf k = p_ * hjt * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  auto x_size = x_.size();
  Eigen::MatrixXf i = Eigen::MatrixXf::Identity(x_size, x_size);
  p_ = (i - k * hj) * p_;
}
