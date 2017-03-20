#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF(VectorXd &x0, MatrixXd &P0, DynamicModel dynamicModel) {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  ekf_.Init(x0, P0, dynamicModel);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

/**
* Add new sensor definition.
*/
void FusionEKF::AddSensor(SensorType type, const Eigen::MatrixXd& R, const MeasurementFunc& h, const SensorJacobianFunc& H)
{
  sensors_[type] = std::make_tuple(R, h, H);
}

void FusionEKF::AddLinearSensor(SensorType type, const Eigen::MatrixXd& R, const MatrixXd& H)
{
  sensors_[type] = MakeLinearSensor(R, H);
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
      previous_timestamp_ = measurement_pack.timestamp_;
      // done initializing, no need to predict or update
      is_initialized_ = true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
//  float dt2, dt3, dt4;

//  ekf_.F_(0, 2) = dt;
//  ekf_.F_(1, 3) = dt;
//
//  dt2 = dt*dt;
//  dt3 = dt2*dt;
//  dt4 = dt3*dt;
//
//  ekf_.Q_ << noise_ax*dt4/4, 0, noise_ax*dt3/2, 0,
//              0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
//              noise_ax*dt3/2, 0, noise_ax*dt2,  0,
//              0, noise_ay*dt3/2, 0, noise_ay*dt2;

  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  ekf_.UpdateEKF(measurement_pack.raw_measurements_, sensors_[measurement_pack.sensor_type_]);

  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
