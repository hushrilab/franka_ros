// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class HenningImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
  
  // Filter
    void Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input,  const Eigen::MatrixXd& input_prev,
    const Eigen::MatrixXd& y_prev,  Eigen::MatrixXd& y);  // NOLINT (readability-identifier-naming)  

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // for trajectory
  double r {0.1};
  double omega {1};
  
  double filter_params_{0.005};
  double nullspace_stiffness_{2.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> K_cartesian;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> D_cartesian;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  
  // for PID controller
  Eigen::Matrix<double, 6, 6> K_P;
  Eigen::Matrix<double, 6, 6> K_D;
  Eigen::Matrix<double, 6, 6> K_I;
  Eigen::Matrix<double, 6, 1> eint;
  
  
  bool flag;
  Eigen::Matrix<double, 6, 6> Lambda;
  Eigen::Matrix<double, 7, 6> J_plus;
  Eigen::Matrix<double, 6, 7> J_T_plus;
  Eigen::Matrix<double, 6, 6> S_P;
  Eigen::Matrix<double, 6, 6> S_f;
  Eigen::Matrix<double, 7, 1> tau_0;
  Eigen::Matrix<double, 7, 6> J_dash;
  Eigen::Matrix<double, 6, 1> f;
  
  Eigen::Matrix<double, 6, 6> K_p;
  Eigen::Matrix<double, 6, 6> K_d;
  Eigen::Matrix<double, 6, 6> M_d;
  Eigen::Matrix<double, 6, 6> M_r;
  Eigen::Matrix<double, 6, 6> I;
  Eigen::Matrix<double, 7, 7> K_N;
  Eigen::Matrix<double, 7, 7> D_N;
  Eigen::Matrix<double, 7, 7> D_eta;
  Eigen::Matrix<double, 7, 7> K_p0;
  Eigen::Matrix<double, 7, 1> q_nullspace;
  Eigen::Matrix<double, 7, 1> N;
  Eigen::Matrix<double, 6, 1> F_tau;
  Eigen::Matrix<double, 6, 7> djacobian;
  Eigen::Matrix<double, 6, 7> jacobian_prev;
  Eigen::Matrix<double, 7, 1> dq_prev;
  Eigen::Matrix<double, 7, 1> ddq;
  Eigen::Matrix<double, 7, 1> ddq_max;
  Eigen::Matrix<double, 7, 1> dq_max;
  Eigen::Matrix<double, 7, 1> tau_max;
  Eigen::Matrix<double, 7, 1> tau_min;
  

  
  // For filtering
  Eigen::Matrix<double, 6, 1> F_ext_filtered;
  Eigen::Matrix<double, 6, 1> F_ext_prev;
  Eigen::Matrix<double, 7, 1> ddq_prev;
  Eigen::Matrix<double, 7, 1> ddq_filtered;

  
  double alpha, beta, gamma; // in degrees
  
  Eigen::Vector3d position_d;
  Eigen::Vector3d position_d_target;
  Eigen::Vector3d position_init;
  Eigen::Vector3d angles_d;
  Eigen::Vector3d angles_init;
  Eigen::Vector3d velocity_d;
  Eigen::Vector3d acceleration_d;
  Eigen::Quaterniond orientation_d;
  Eigen::Quaterniond orientation_init;
  Eigen::Quaterniond orientation_d_target;
  Eigen::Vector3d omega_d_local;
  Eigen::Vector3d omega_d_global;
  Eigen::Vector3d domega_d_local;
  Eigen::Vector3d domega_d_global;
  std::mutex position_and_orientation_d_target_mutex;
//   Eigen::Vector3d position_d_target_;
//   Eigen::Quaterniond orientation_d_target_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_example_controllers
