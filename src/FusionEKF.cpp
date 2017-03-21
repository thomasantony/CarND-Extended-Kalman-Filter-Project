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
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
}

void FusionEKF::Init(const VectorXd &x0, const MatrixXd &P0, DynamicModel dynamicModel, long timestamp)
{
  /****************************************************************************
   *  Initialization
   ****************************************************************************/

  ekf_.Init(x0, P0, dynamicModel);

  // first measurement
  previous_timestamp_ = timestamp;

  // done initializing, no need to predict or update
  is_initialized_ = true;
}
/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

/**
* Add new sensor definition.
*/
void FusionEKF::AddSensor(SensorType type, const Eigen::MatrixXd& R, const SensorFunc& h, const SensorJacobianFunc& H)
{
  sensors_[type] = std::make_tuple(R, h, H);
}
/**
* Add new linear sensor definition.
*/
void FusionEKF::AddLinearSensor(SensorType type, const Eigen::MatrixXd& R, const MatrixXd& H)
{
  sensors_[type] = MakeLinearSensor(R, H);
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  ekf_.UpdateEKF(measurement_pack.raw_measurements_, sensors_[measurement_pack.sensor_type_]);

  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
//  cout << "x_ = " << ekf_.x_ << endl;
//  cout << "P_ = " << ekf_.P_ << endl;
}
