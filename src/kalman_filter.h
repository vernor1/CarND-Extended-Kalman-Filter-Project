#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {
public:
  KalmanFilter();
  virtual ~KalmanFilter() { }
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);
  Eigen::VectorXd GetState() const { return x_; }

private:
  // Indicates whether the tracking was initiallized or not (first measurement)
  bool is_initialized_;

  // Previous timestamp
  long previous_timestamp_;

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd p_;

  // Measurement matrix
  Eigen::MatrixXd h_;

  // Measurement covariance matrices
  Eigen::MatrixXd r_lidar_;
  Eigen::MatrixXd r_radar_;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void LidarUpdate(const Eigen::VectorXd& z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void RadarUpdate(const Eigen::VectorXd& z);
};

#endif // KALMAN_FILTER_H_
