#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd difsqr = estimations[i] - ground_truth[i];
    difsqr = difsqr.array() * difsqr.array(); // element wise multiplication
    rmse = rmse + difsqr;
  }
  
  //calculate the mean
  rmse = rmse / estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt(); 

  return rmse;
}