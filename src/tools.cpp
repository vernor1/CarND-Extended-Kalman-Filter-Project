#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // The estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size())  {
      std::cout << "Invalid input" << std::endl;
      return rmse;
  }

  //  The estimation vector size should not be zero
  if (estimations.empty()) {
      std::cout << "Invalid input" << std::endl;
      return rmse;
  }

  // Accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().pow(2);
  }

  rmse = (rmse / estimations.size()).array().sqrt();

  return rmse;
}

//MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
//}
