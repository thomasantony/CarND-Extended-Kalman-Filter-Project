#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"

#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/** Problem-specific configuration starts here **/
namespace {
  /* Motion model */
  const auto modelFunc = ModelFunc([](float dt, const VectorXd &x){
    MatrixXd F = MatrixXd(4,4);
    F << 1, 0, dt, 0,
        0, 1,  0, dt,
        0, 0,  1, 0,
        0, 0,  0, 1;
    return std::tuple<VectorXd, MatrixXd>(F*x, F);
  });

  // Process noise
  const float noise_ax = 7, noise_ay = 7;

  const auto processNoiseFunc =  ProcessNoiseFunc([](float dt, const VectorXd &x) mutable {
    MatrixXd Q = MatrixXd(4, 4);
    float dt2, dt3, dt4;
    dt2 = dt*dt;
    dt3 = dt2*dt;
    dt4 = dt3*dt;

    Q << noise_ax*dt4/4, 0, noise_ax*dt3/2, 0,
        0, noise_ay*dt4/4, 0, noise_ay*dt3/2,
        noise_ax*dt3/2, 0, noise_ax*dt2,  0,
        0, noise_ay*dt3/2, 0, noise_ay*dt2;
    return Q;
  });
  const DynamicModel motionModel = DynamicModel(modelFunc, processNoiseFunc);

  /* Sensor models */
  // Laser Sensor covariance and model matrix
  const MatrixXd H_laser = (MatrixXd(2,4) << 1, 0, 0, 0,
      0, 1, 0, 0).finished();
  const MatrixXd R_laser = (MatrixXd(2,2) << 0.0225, 0,
      0, 0.0225).finished();

  // Radar Sensor
  // Covariance
  const MatrixXd R_radar = (MatrixXd(3,3) << 0.09, 0, 0,
      0, 0.09, 0,
      0, 0, 0.09).finished();

  // The non-linear radar measurement model
  VectorXd RadarMeasurement(const VectorXd &x) {
    VectorXd z_out(3);
    float px = x[0];
    float py = x[1];
    float vx = x[2];
    float vy = x[3];

    z_out << sqrt(px*px + py*py),
        atan2(py, px),
        (px*vx + py*vy)/sqrt(px*px + py*py);

    return z_out;
  }

  // Compute the radar jacobian matrix
  MatrixXd RadarJacobian(const VectorXd &x) {
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    float rho = sqrt(px * px + py * py);
    float rho2 = rho * rho;
    float rho3 = rho2 * rho;

    //check division by zero
    if (rho < 1e-6) {
      cerr << "Divide by zero!! Oh noes!!" << endl;
      return Hj;
    }
    if (abs(rho) < 1e-4) {
      Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
    } else {
      //compute the Jacobian matrix
      Hj << px / rho, py / rho, 0, 0,
          -py / rho2, px / rho2, 0, 0,
          py * (vx * py - vy * px) / rho3, px * (vy * px - vx * py) / rho3, px / rho, py / rho;
    }

    return Hj;
  }

  // Initial state, covariance and sensors
  const MatrixXd P_0 = (MatrixXd(4,4) << 50, 0, 0, 0,
      0, 50, 0, 0,
      0, 0, 100, 0,
      0, 0, 0, 100).finished();

  /* Function to initialize the FusionEKF object using first measurement */
  // Returns true on success
  bool InitEKF(FusionEKF& fusionEKF, MeasurementPackage& first_measurement)
  {
    // Initialize the sensors
    fusionEKF.AddLinearSensor(SensorType::LASER, R_laser, H_laser);
    fusionEKF.AddSensor(SensorType::RADAR, R_radar, RadarMeasurement);

    // Initialize state from first measurement
    VectorXd x_0 = VectorXd(4);
    float rho;
    if (first_measurement.sensor_type_ == SensorType::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float phi, rhodot;
      rho = first_measurement.raw_measurements_[0];
      phi = first_measurement.raw_measurements_[1];
      x_0 << rho*cos(phi), rho*sin(phi), 0, 0;
    }
    else if (first_measurement.sensor_type_ == SensorType::LASER) {
      /**
      Initialize state.
      */
      float px, py;
      px = first_measurement.raw_measurements_[0];
      py = first_measurement.raw_measurements_[1];
      rho = sqrt(px*px + py*py);
      x_0 << px, py, 0, 0;
    }
    if(abs(rho) < 1e-4)
    {
      return false;
    }
    fusionEKF.Init(x_0, P_0, motionModel, first_measurement.timestamp_);

    return true;
  }
}
/***** End of configuration *****/
/********************************/

/*** Main Program begins here ***/
void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {

    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = SensorType::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = SensorType::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance and initialize it
  FusionEKF fusionEKF;


  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    if(!fusionEKF.is_initialized_)
    {
      auto result = InitEKF(fusionEKF, measurement_pack_list[k]);
      if(!result){
        // skip frame if initialization failed
        cout << "Skipping frame " << k << endl;
        continue;
      }
    }
    // start filtering from the second frame (assuming first frame was valid)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == SensorType::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == SensorType::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }
  // compute the accuracy (RMSE)
  cout << "\nAccuracy - RMSE:" << endl << Tools::CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
