#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
#include <tuple>
#include <memory>

#include "measurement_package.h"
#include "ExtendedKalmanFilter.h"

typedef std::unordered_map<SensorType, SensorModel, enum_hash> SensorMap;

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  void Init(VectorXd &x0, MatrixXd &P0, DynamicModel dynamicModel);
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Add sensor definition to FusionEKF
   */
  void AddSensor(SensorType type, const Eigen::MatrixXd& R, const MeasurementFunc& h, const SensorJacobianFunc& H = NULL);

  /**
   * Add linear model
   * @param type
   * @param R
   * @param h
   * @param H
   */
  void AddLinearSensor(SensorType type, const Eigen::MatrixXd& R, const MatrixXd& H);
  /**
  * Kalman Filter update and prediction math lives in here.
  */
  ExtendedKalmanFilter ekf_;



private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // Map containing sensor definitions
  SensorMap sensors_;

//  MatrixXd F_;
//  MatrixXd R_;
//  MatrixXd Q_;

//  //acceleration noise components
//  float noise_ax;
//  float noise_ay;
};

#endif /* FusionEKF_H_ */
