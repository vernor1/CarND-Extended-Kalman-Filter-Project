#include "kalman_filter.h"
#include <iostream>

namespace {

const float kMaxDeviation = 1000;
const float kNoiseAx = 9;
const float kNoiseAy = 9;


Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) {
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  Eigen::MatrixXd hj(3, 4);
  double sum_of_squares = px * px + py * py;

  // Check division by zero
  if (sum_of_squares == 0) {
      // TODO: implement a better solution
      return hj;
  }

  double sqrt_of_sum_of_squares = std::sqrt(sum_of_squares);
  double sum_of_squares_to_3_by_2 = sum_of_squares * sqrt_of_sum_of_squares;
  double px_by_sqrt = px / sqrt_of_sum_of_squares;
  double py_by_sqrt = py / sqrt_of_sum_of_squares;

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


Eigen::VectorXd ConvertCartesianToPolar(const Eigen::VectorXd& cartesian) {
  double px = cartesian(0);
  double py = cartesian(1);
  double vx = cartesian(2);
  double vy = cartesian(3);

  Eigen::VectorXd polar(3);
  double sum_of_squares = px * px + py * py;

  // Check division by zero
  if (sum_of_squares == 0) {
      // TODO: implement a better solution
      return polar;
  }

  // TODO: consider changing to float
  double sqrt_of_sum_of_squares = std::sqrt(sum_of_squares);

  polar(0) = sqrt_of_sum_of_squares;;
  polar(1) = std::atan2(py, px);
  polar(2) = (px * vx + py * vy) / sqrt_of_sum_of_squares;

  return polar;
}


Eigen::VectorXd ConvertPolarToCartesian(const Eigen::VectorXd& polar) {
  double rho = polar(0);
  double phi = polar(1);

  Eigen::VectorXd cartesian(4);
  cartesian << rho * std::cos(phi), rho * std::sin(phi), 0, 0;

  return cartesian;
}

} // namespace


// Public Members
// -----------------------------------------------------------------------------

KalmanFilter::KalmanFilter()
  : is_initialized_(false),
    previous_timestamp_(0),
    x_(Eigen::VectorXd(4)),
    p_(Eigen::MatrixXd(4, 4)),
    h_(Eigen::MatrixXd(2 ,4)),
    r_lidar_(Eigen::MatrixXd(2, 2)),
    r_radar_(Eigen::MatrixXd(3, 3)) {

  // TODO: Consider moving the Eigen constans to the nameless namespace

  // Measurement matrix
  h_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  // Measurement covariance matrix for laser
  r_lidar_ << 0.0225, 0,
              0,      0.0225;

  // Measurement covariance matrix for radar
  r_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;
}


void KalmanFilter::ProcessMeasurement(const MeasurementPackage& measurement_pack) {

  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      x_ = ConvertPolarToCartesian(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_(0),
            measurement_pack.raw_measurements_(1),
            0,
            0;
    }

    // State covariance matrix
    p_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, kMaxDeviation, 0,
          0, 0, 0, kMaxDeviation;

    // Set the initial timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // Done initializing, no need to predict or update
    is_initialized_ = true;

    // Print the output
    std::cout << "EKF: " << std::endl;
    std::cout << "x_ = " << x_ << std::endl;
    std::cout << "P_ = " << p_ << std::endl;
    return;
  }


  // Time elapsed between the current and previous measurements in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  //  Prediction
  Predict(dt);

  // Update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    RadarUpdate(measurement_pack.raw_measurements_);
  } else {
    LidarUpdate(measurement_pack.raw_measurements_);
  }

  // Print the output
  std::cout << "x_ = " << x_ << std::endl;
  std::cout << "P_ = " << p_ << std::endl;
}


void KalmanFilter::Predict(double dt) {
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Define the state transition matrix
  Eigen::MatrixXd f(4, 4);
  f  << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
  // Define the process covariance matrix Q
  Eigen::MatrixXd q(4, 4);
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  q << dt4 / 4 * kNoiseAx, 0, dt3 / 2 * kNoiseAx, 0,
       0, dt4 / 4 * kNoiseAy, 0, dt3 / 2 * kNoiseAy,
       dt3 / 2 * kNoiseAx, 0, dt2 * kNoiseAx, 0,
       0, dt3 / 2 * kNoiseAy, 0, dt2 * kNoiseAy;
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + q;
}


void KalmanFilter::LidarUpdate(const Eigen::VectorXd& z) {
  Eigen::VectorXd y = z - h_ * x_;
  Eigen::MatrixXd ht = h_.transpose();
  Eigen::MatrixXd s = h_ * p_ * ht + r_lidar_;
  Eigen::MatrixXd k = p_ * ht * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  size_t x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k * h_) * p_;
}


void KalmanFilter::RadarUpdate(const Eigen::VectorXd& z) {
  // y = z - h(x)
  Eigen::VectorXd y = z - ConvertCartesianToPolar(x_);
  // Hj instead of H for calcualting S, K, P
  Eigen::MatrixXd hj = CalculateJacobian(x_);
  Eigen::MatrixXd hjt = hj.transpose();
  Eigen::MatrixXd s = hj * p_ * hjt + r_radar_;
  Eigen::MatrixXd k = p_ * hjt * s.inverse();

  // New estimate
  x_ = x_ + k * y;
  size_t x_size = x_.size();
  Eigen::MatrixXd i = Eigen::MatrixXd::Identity(x_size, x_size);
  p_ = (i - k * hj) * p_;
}
