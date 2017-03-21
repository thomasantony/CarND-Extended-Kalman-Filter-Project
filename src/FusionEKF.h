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

typedef std::unordered_map<SensorType, SensorModel> SensorMap;

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

  void Init(const VectorXd &x0, const MatrixXd &P0, DynamicModel dynamicModel, long timestamp);
  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Add sensor definition to FusionEKF
   * @param type
   * @param measCov
   * @param measModel
   * @param measJac
   */
  void AddSensor(SensorType type, const Eigen::MatrixXd& measCov, const SensorFunc& measModel, const SensorJacobianFunc& measJac = NULL);

  /**
   * Add linear model
   * @param type
   * @param measCov
   * @param measMat
   */
  void AddLinearSensor(SensorType type, const Eigen::MatrixXd& measCov, const MatrixXd& measMat);
  /**
  * Kalman Filter update and prediction math lives in here.
  */
  ExtendedKalmanFilter ekf_;

  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

private:
  // previous timestamp
  long previous_timestamp_;

  // Map containing sensor definitions
  SensorMap sensors_;
};

#endif /* FusionEKF_H_ */
