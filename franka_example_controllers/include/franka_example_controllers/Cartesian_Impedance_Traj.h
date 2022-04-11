
#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class CartesianImpedanceTrajectory : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
//   void stopping(const ros::Time&) override;
  
 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);
  
  // Filter
  void Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input,  const Eigen::MatrixXd& input_prev, Eigen::MatrixXd& y);
  void GripperMove(double width, double speed); 
  void GripperGrasp(double width, double speed, int force, double epsilon);
  void SetLoad(double mass_old, double mass_new, std::array<double, 3> vec2CoG, double time, double t);
  
  //  Function to lad csv files; Source: https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
  template<typename M> M load_csv (const std::string & path, const std::string & filename) {
    std::ifstream indata;
    indata.open(path + filename);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
  }
  
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;
  std::vector<hardware_interface::JointHandle>  joint_handle;
  
  // Load MATLAB trajectory
  std::string filename = "BookTrajectory"; //"ArmSwing", "Hexagon", "BookTrajectory"

  std::string path = "../rospackages/catkin_ws/src/franka_ros/franka_example_controllers/MATLAB_Trajectories/" + filename + "/";
 // std::string path = "../ws/src/franka_ros/franka_example_controllers/MATLAB_Trajectories/" + filename + "/";

  Eigen::MatrixXd X       = load_csv<Eigen::MatrixXd>(path,            "x.csv");
  Eigen::MatrixXd dX      = load_csv<Eigen::MatrixXd>(path,           "dx.csv");
  Eigen::MatrixXd ddX     = load_csv<Eigen::MatrixXd>(path,          "ddx.csv");
  Eigen::MatrixXd Quats   = load_csv<Eigen::MatrixXd>(path,        "quats.csv");
  Eigen::MatrixXd omega   = load_csv<Eigen::MatrixXd>(path,        "omega.csv");
  Eigen::MatrixXd domega  = load_csv<Eigen::MatrixXd>(path,       "domega.csv");
  Eigen::MatrixXd q_null  = load_csv<Eigen::MatrixXd>(path,       "q_null.csv");
  Eigen::MatrixXd ts      = load_csv<Eigen::MatrixXd>(path,           "ts.csv");
  Eigen::MatrixXd gripper = load_csv<Eigen::MatrixXd>(path, "gripperTasks.csv");
  Eigen::MatrixXd K_mat   = load_csv<Eigen::MatrixXd>(path,          "K_p.csv");  // check if they are included in trajectory folder
  Eigen::MatrixXd D_mat   = load_csv<Eigen::MatrixXd>(path,        "D_eta.csv");
  Eigen::MatrixXd ExtLoad = load_csv<Eigen::MatrixXd>(path,     "Ext_load.csv");
  
  double i        = 0;
  double mytime   = 0;

  // for quintic trajectory
  double T;
  double a3;
  double a4;
  double a5;
  double s   = 0; 
  double ds  = 0;
  double dds = 0;
  
  // Errors
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> derror;
  Eigen::Matrix<double, 6, 1> dderror;
  Eigen::Vector3d quat_d;
  Eigen::Vector3d quat_c;

  bool notFirstRun;
  Eigen::Affine3d TransformationMatrix;
  Eigen::Matrix<double, 7, 7> mass_inv;
  Eigen::Matrix<double, 6, 6> Lambda;
  Eigen::Matrix<double, 6, 6> Lambda_prev;
  Eigen::Matrix<double, 6, 6> C_hat;
  
  Eigen::Matrix<double, 6, 6> K_p;
  Eigen::Matrix<double, 6, 6> K_d;
  Eigen::Matrix<double, 7, 7> K_N;
  Eigen::Matrix<double, 7, 7> D_N;
  Eigen::Matrix<double, 7, 7> I;
  Eigen::Matrix<double, 7, 1> q_nullspace;
  Eigen::Matrix<double, 7, 1> q_nullspace_init;
  Eigen::Matrix<double, 7, 1> q_nullspace_target;
  Eigen::Matrix<double, 7, 7> N;
  Eigen::Matrix<double, 6, 1> F_tau;
  Eigen::Matrix<double, 7, 1> tau_task;
  Eigen::Matrix<double, 7, 1> tau_nullspace; 
  Eigen::Matrix<double, 7, 1> tau_d;
  Eigen::Matrix<double, 6, 7> djacobian;
  Eigen::Matrix<double, 6, 7> djacobian_filtered;
  Eigen::Matrix<double, 6, 7> jacobian_prev;
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  Eigen::Matrix<double, 5, 1> gripper_command;
  Eigen::Matrix<double, 6, 1> external_load;
  
    // Damping Desgin
  Eigen::Matrix<double, 6, 6> D_eta;
  Eigen::Matrix<double, 6, 6> K_p1;
  Eigen::Matrix<double, 6, 6> A;
  
  Eigen::Vector3d    curr_position;
  Eigen::Matrix<double, 6, 1> curr_velocity;
  Eigen::Matrix<double, 6, 1> ddx;
  Eigen::Quaterniond curr_orientation;
  Eigen::Vector3d    position_d;
  Eigen::Vector3d    position_d_target;
  Eigen::Vector3d    position_init;
  Eigen::Vector3d    angles_d_target;
  Eigen::Vector3d    velocity_d;
  Eigen::Vector3d    acceleration_d;
  Eigen::Quaterniond orientation_d;
  Eigen::Quaterniond orientation_init;
  Eigen::Quaterniond orientation_d_target;
  Eigen::Vector3d    omega_d;
  Eigen::Vector3d    domega_d;

  const double delta_tau_max_{1.0};  
  int j = 0;
};

}  // namespace franka_example_controllers
