#include "kalman_filter.h"
#include <iostream>

namespace {

const float kNoiseAx = 9;
const float kNoiseAy = 9;
const float kEpsilon = 1e-6;

// Measurement matrix and its transposed variant
const Eigen::MatrixXf kH((Eigen::MatrixXf(2, 4)
  << 1, 0, 0, 0,
     0, 1, 0, 0).finished());
const Eigen::MatrixXf kHt = kH.transpose();

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

Eigen::MatrixXf CalculateJacobian(const Eigen::VectorXf& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  Eigen::MatrixXf hj(Eigen::MatrixXf::Zero(3, 4));
  float sum_of_squares = px * px + py * py;

  if (std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon) {
    return hj;
  }

  float sqrt_of_sum_of_squares = std::sqrt(sum_of_squares);
  float sum_of_squares_to_3_by_2 = sum_of_squares * sqrt_of_sum_of_squares;
  float px_by_sqrt = px / sqrt_of_sum_of_squares;
  float py_by_sqrt = py / sqrt_of_sum_of_squares;

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
  float px = cartesian(0);
  float py = cartesian(1);
  float vx = cartesian(2);
  float vy = cartesian(3);

  Eigen::VectorXf polar(Eigen::VectorXf::Zero(3));

  if (std::fabs(px) < kEpsilon && std::fabs(py) < kEpsilon) {
    return polar;
  }

  float sqrt_of_sum_of_squares = std::sqrt(px * px + py * py);

  polar(0) = sqrt_of_sum_of_squares;;
  polar(1) = std::atan2(py, px);
  polar(2) = (px * vx + py * vy) / sqrt_of_sum_of_squares;

  return polar;
}


Eigen::VectorXf ConvertPolarToCartesian(const Eigen::VectorXf& polar) {
  float rho = polar(0);
  float phi = polar(1);

  Eigen::VectorXf cartesian(4);
  cartesian << rho * std::cos(phi), rho * std::sin(phi), 0, 0;

  return cartesian;
}

} // namespace


// Public Members
// -----------------------------------------------------------------------------

KalmanFilter::KalmanFilter()
  : is_initialized_(false),
    previous_timestamp_(0),
    x_(Eigen::VectorXf::Zero(4)),
    p_(Eigen::MatrixXf::Zero(4, 4)) { }


void KalmanFilter::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  if (!is_initialized_) {
    // Initialize the state with the first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      x_ = ConvertPolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_(0),
            measurement_pack.raw_measurements_(1),
            0, 0;
    }

    // State covariance matrix
    p_ = kPinitial;

    // Set the initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;

    std::cout << "x0" << std::endl << x_ << std::endl;
    std::cout << "P0" << std::endl << p_ << std::endl;

    return;
  }


  // Time elapsed between the current and previous measurements in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  //  Prediction
  Predict(dt);

  // Update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    RadarUpdate(measurement_pack.raw_measurements_);
  } else {
    LidarUpdate(measurement_pack.raw_measurements_);
  }

  std::cout << "x" << std::endl << x_ << std::endl;
  std::cout << "P" << std::endl << p_ << std::endl;
}


// Private Members
// -----------------------------------------------------------------------------

void KalmanFilter::Predict(float dt) {
  // Define the state transition matrix
  Eigen::MatrixXf f(4, 4);
  f  << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Define the process covariance matrix Q
  Eigen::MatrixXf q(4, 4);
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  q << dt4 / 4 * kNoiseAx, 0, dt3 / 2 * kNoiseAx, 0,
       0, dt4 / 4 * kNoiseAy, 0, dt3 / 2 * kNoiseAy,
       dt3 / 2 * kNoiseAx, 0, dt2 * kNoiseAx, 0,
       0, dt3 / 2 * kNoiseAy, 0, dt2 * kNoiseAy;

  // Predict
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + q;
}


void KalmanFilter::LidarUpdate(const Eigen::VectorXf& z) {
  Eigen::VectorXf y = z - kH * x_;
  Eigen::MatrixXf kHt = kH.transpose();
  Eigen::MatrixXf s = kH * p_ * kHt + kRlidar;
  Eigen::MatrixXf k = p_ * kHt * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  size_t x_size = x_.size();
  Eigen::MatrixXf i = Eigen::MatrixXf::Identity(x_size, x_size);
  p_ = (i - k * kH) * p_;
}


void KalmanFilter::RadarUpdate(const Eigen::VectorXf& z) {
  // y = z - h(x)
  Eigen::VectorXf y = z - ConvertCartesianToPolar(x_);

  // Hj instead of H for calcualting S, K, P
  Eigen::MatrixXf hj = CalculateJacobian(x_);
  Eigen::MatrixXf hjt = hj.transpose();
  Eigen::MatrixXf s = hj * p_ * hjt + kRradar;
  Eigen::MatrixXf k = p_ * hjt * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  size_t x_size = x_.size();
  Eigen::MatrixXf i = Eigen::MatrixXf::Identity(x_size, x_size);
  p_ = (i - k * hj) * p_;
}
