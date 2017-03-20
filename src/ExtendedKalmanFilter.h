#ifndef EXTENDEDKALMANFILTER_H_
#define EXTENDEDKALMANFILTER_H_
#include "Eigen/Dense"
#include <iostream>
#include <functional>

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum SensorType {
  LASER,
  RADAR
};

// function pointer for measurement function
typedef std::function<std::tuple<VectorXd, MatrixXd> (float dt, const VectorXd &x)> ModelFunc;
//typedef std::function<MatrixXd (float dt, const VectorXd &x)> ModelJacobianFunc;
typedef std::function<MatrixXd (float dt, const VectorXd &x)> ProcessNoiseFunc;

typedef std::function<VectorXd (const VectorXd &x)> MeasurementFunc;
typedef std::function<MatrixXd (const VectorXd &x)> SensorJacobianFunc;

typedef std::tuple<ModelFunc, ProcessNoiseFunc> DynamicModel;
typedef std::tuple<MatrixXd, MeasurementFunc, SensorJacobianFunc> SensorModel;

//ModelFunc MakeLinearSensor(const MatrixXd& F);
inline SensorModel MakeLinearSensor(const MatrixXd &R, const MatrixXd &H)
{
  auto measurementFunc = MeasurementFunc([H](const VectorXd& x){return H*x;});
  auto jacobianFunc = SensorJacobianFunc([H](const VectorXd& x){return H;});
  return std::make_tuple(R, measurementFunc, jacobianFunc);
}

inline SensorJacobianFunc MakeSensorJacobian(MeasurementFunc f, const float h = 1e-4)
{
  return SensorJacobianFunc([f, h](const VectorXd& x){
    auto n = x.size();

    MatrixXd I = MatrixXd::Identity(n, n)*h;
    auto fx = f(x);
    auto m = fx.size();

    MatrixXd jac = MatrixXd(m, n);
    for(auto i=0; i<x.size(); i++)
    {
      auto step = I.col(i);
      jac.col(i) = (f(x+step) - f(x-step))/(2*h);
    }
    return jac;
  });
}

class ExtendedKalmanFilter {
public:

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // identity matrix
  MatrixXd I_;

  DynamicModel dynamicModel_;
  /**
   * Constructor
   */
  ExtendedKalmanFilter();

  /**
   * Destructor
   */
  virtual ~ExtendedKalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, DynamicModel dynamicModel);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float delta_T);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const  VectorXd &z, const SensorModel& sensor);

  /**
   * Updates the state by using the given measurement and predicted measurement
   * @param z The measurement at k+1
   * @param z_pred The predicted measurement at k+1
   * @param H The jacobian of the measurement function
   */
  void UpdateWithPrediction(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &H, const MatrixXd &R);
};

#endif /* EXTENDEDKALMANFILTER_H_ */
