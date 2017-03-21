#include "ExtendedKalmanFilter.h"
#include "tools.h"


ExtendedKalmanFilter::ExtendedKalmanFilter() {}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::Init(const VectorXd &x_in, const MatrixXd &P_in, DynamicModel dynamicModel) {
  x_ = x_in;
  P_ = P_in;
  dynamicModel_ = dynamicModel;
  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void ExtendedKalmanFilter::Predict(float delta_T) {
  // KF Prediction step
  ProcessNoiseFunc noiseFunc;
  ModelFunc dynamicFunc;
  MatrixXd F;

  std::tie(dynamicFunc, noiseFunc) = dynamicModel_;
  std::tie(x_, F) = dynamicFunc(delta_T, x_);

  auto Q_ = noiseFunc(delta_T, x_);
  P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const SensorModel& sensor){
  /**
    * update the state by using Extended Kalman Filter equations
  */
  SensorFunc h_func;
  SensorJacobianFunc H_func;
  MatrixXd R, H;
  VectorXd z_pred;

  std::tie(R, h_func, H_func) = sensor;

  z_pred = h_func(x_);
  if(H_func == NULL)
  {
    H = Tools::ComputeJacobian(h_func, x_);
  }else{
    H = H_func(x_);
  }

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