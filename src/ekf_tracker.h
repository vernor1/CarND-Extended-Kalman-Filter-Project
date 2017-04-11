#ifndef EKF_TRACKER_H_
#define EKF_TRACKER_H_

#include "Eigen/Dense"

class EkfTracker {
public:
  struct Measurement {
    enum class SensorType {
      kLidar,
      kRadar
    };

    long long timestamp;
    SensorType sensor_type;
    Eigen::VectorXf value;
  };

  EkfTracker();
  virtual ~EkfTracker() { }
  void ProcessMeasurement(const Measurement& measurement);
  Eigen::VectorXf GetState() const { return x_; }

private:
  // Indicates whether the tracking was initiallized or not (first measurement)
  bool is_initialized_;

  // Previous timestamp
  long previous_timestamp_;

  // State vector
  Eigen::VectorXf x_;

  // State covariance matrix
  Eigen::MatrixXf p_;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void LidarUpdate(const Eigen::VectorXf& z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void RadarUpdate(const Eigen::VectorXf& z);
};

#endif // EKF_TRACKER_H_
