#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, DynamicModel dynamicModel) {
  x_ = x_in;
  P_ = P_in;
//  F_ = F_in;
//  Q_ = Q_in;
  dynamicModel_ = dynamicModel;
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void ExtendedKalmanFilter::Predict(float delta_T) {
  // KF Prediction step
  ProcessNoiseFunc noiseFunc;
  ModelFunc dynamicFunc;
  ModelJacobianFunc jacobianFunc;
  std::tie(dynamicFunc, jacobianFunc, noiseFunc) = dynamicModel_;

  x_ = dynamicFunc(delta_T, x_);
  MatrixXd F_ = jacobianFunc(delta_T, x_);
//  x_ = F_ * x_;
  auto Q_ = noiseFunc(delta_T, x_);
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const SensorModel& sensor){
  /**
    * update the state by using Extended Kalman Filter equations
  */
  MeasurementFunc h_func;
  SensorJacobianFunc H_func;
  MatrixXd R;

  std::tie(R, h_func, H_func) = sensor;

  if(H_func == NULL)
  {
    H_func = MakeSensorJacobian(h_func);
  }
  VectorXd z_pred;
  z_pred = h_func(x_);
  MatrixXd H = H_func(x_);

  UpdateWithPrediction(z, z_pred, H, R);
}

void ExtendedKalmanFilter::UpdateWithPrediction(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &H, const MatrixXd &R)
{
  MatrixXd Ht = H.transpose();
  VectorXd y = z - z_pred;
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H) * P_;
}