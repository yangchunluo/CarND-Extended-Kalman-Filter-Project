#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Laser measurement covariance matrix (noise)
  // Laser has two measurements (px, py), hence the matrix is 2x2
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement function for laser (this is straightforward)
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar measurement covariance matrix (noise)
  // Radar has three measurements (ro, theta, ro_dot), hence the matrix is 3x3
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // There is not a matrix form for the measurement function for radar.
  // The non-linear measurement function will be linearized on the fly for each invocations.

  // State transition matrix
  ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &m) {
  if (m.sensor_type_ == MeasurementPackage::LASER) {
    return;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // First measurement
    cout << "EKF initialization" << endl;
    
    float px, py;
    switch (m.sensor_type_) {
      case MeasurementPackage::RADAR: {
        // Convert radar from polar to cartesian coordinates. Ignore velocity informaiton.
        px = m.raw_measurements_[0] * sin(m.raw_measurements_[1]);
        py = m.raw_measurements_[0] * cos(m.raw_measurements_[1]);
        break;
      }
      case MeasurementPackage::LASER: {
        // Laser has position (px, py) but no velocity information.
        px = m.raw_measurements_[0];
        py = m.raw_measurements_[1];
        break;
      }
      default:
        throw "Unknown sensor type";
    }
    // Initial state estimates.
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << px, py, 0, 0;

    // Initialize uncertainty covariance matrix. For velocity the values are large.
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
			         0, 1, 0, 0,
			         0, 0, 1000, 0,
			         0, 0, 0, 1000;

    // Record the first timestamp.
    previous_timestamp_ = m.timestamp_;
    
    // Done initializing, no need to predict or update.
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction (use noise_ax = 9 and noise_ay = 9)
   ****************************************************************************/

  // Get the elapsed time in seconds.
  float dt = (m.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = m.timestamp_;

  // Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

  // Set the process covariance matrix Q (noise)  .
  // Q = G * [[noise_ax, 0], [0, noise_ay]] * G', where G = [[dt2/2, 0], [0, dt2/2], [dt, 0], [0, dt]]
	float dt2 = dt * dt,
        dt3 = dt2 * dt,
        dt4 = dt3 * dt;
  const float noise_ax = 9,
              noise_ay = 9;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt4 * noise_ax / 4, 0, dt3 * noise_ax / 2, 0,
	           0, dt4 * noise_ay / 4, 0, dt3 * noise_ay / 2,
	           dt3 * noise_ax / 2, 0, dt2 * noise_ax, 0,
	           0, dt3 * noise_ay / 2, 0, dt2 * noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  switch (m.sensor_type_) {
    case MeasurementPackage::RADAR: {
      // Radar updates: use Jacobian matrix to linearize the measurement function.
      ekf_.H_ = Tools::CalculateJacobian(ekf_.x_); 
      ekf_.R_ = R_radar_;
      ekf_.Update(m.raw_measurements_);
      break;
    }
    case MeasurementPackage::LASER: {
      // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(m.raw_measurements_);
      break;
    }
    default:
      throw "unknown sensor type";
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
