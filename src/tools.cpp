#include <iostream>
#include <cmath>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  if (estimations.size() == 0) {
	    throw "the estimation vector size should not be zero";
	}
	if (estimations.size() != ground_truth.size()) {
	    throw "the estimation vector size should equal ground truth vector size";
	}

  // Initialization
	VectorXd rmse(estimations[0].size());
  for (int i = 0; i < rmse.size(); i++) {
    rmse(i) = 0;
  }

	// Accumulate squared residuals
	for(int i = 0; i < estimations.size(); i++) {
    VectorXd diff = ground_truth[i] - estimations[i];
    diff = diff.array() * diff.array();
    rmse += diff;
	}

	// Calculate the mean
	rmse /= estimations.size();

	// Calculate the squared root
	return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
	MatrixXd Hj(3, 4);
	// Recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = c1 * c2;

	// Check division by zero
	if (fabs(c1) < 0.0001) {
		cerr << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	// Compute the Jacobian matrix
	Hj << px / c2, py / c2, 0, 0,
		    -(py / c1), px / c1, 0, 0,
		    py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}
