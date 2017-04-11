#ifndef EKF_TRACKER_H_
#define EKF_TRACKER_H_

#include "Eigen/Dense"

class EkfTracker {
public:
  // Contains the sensor measurement data: timestamp, sensor type and value
  struct Measurement {
    // Indicates the sensor type
    enum class SensorType {
      kLidar,
      kRadar
    };

    long long timestamp;
    SensorType sensor_type;
    Eigen::VectorXf value;

    Measurement() : timestamp(0), sensor_type(SensorType::kLidar) { }
  };

  // Ctor
  // @param measurement The initial measurement
  EkfTracker(const Measurement& measurement);

  // Dtor
  virtual ~EkfTracker() { }

  // Processes a single measurement data
  // @param measurement The measurement data
  void ProcessMeasurement(const Measurement& measurement);

  // Returns the current state
  inline Eigen::VectorXf GetState() const { return x_; }

private:
  // Previous timestamp
  long previous_timestamp_;

  // State vector
  Eigen::VectorXf x_;

  // State covariance matrix
  Eigen::MatrixXf p_;

  // Predicts the state and the state covariance using the process model
  // @param dt Time between k and k+1 in s
  void Predict(float dt);

  // Updates the state by using standard Kalman Filter equations
  // @param z The measurement at k+1
  void LidarUpdate(const Eigen::VectorXf& z);

  // Updates the state by using Extended Kalman Filter equations
  // @param z The measurement at k+1
  void RadarUpdate(const Eigen::VectorXf& z);
};

#endif // EKF_TRACKER_H_
