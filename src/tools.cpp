#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  // Lecture Evaluating KF performance code
  VectorXd rmse(4);
  rmse.fill(0);

  if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimations or ground truths" << endl;
    return rmse;
  }
  for(int i=0;i<estimations.size();i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

double Tools::CalculateNIS(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth, const MatrixXd S) {
  double nis = 0.0;
  if(estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimations or ground truths" << endl;
    return nis;
  }
  for(int i=0;i<estimations.size();i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    double nis_at_ = residual.transpose() * S.inverse() * residual;
    nis += nis_at_;
  }
  return nis;
}