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

class MyImpedanceController : public controller_interface::MultiInterfaceController<
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
  
  Eigen::Matrix<double, 3, 3> K_d;
  Eigen::Matrix<double, 3, 3> D_d;
  Eigen::Matrix<double, 3, 3> M_d;
  Eigen::Matrix<double, 7, 7> K_n;
  Eigen::Matrix<double, 6, 6> M_r;
  Eigen::Matrix<double, 6, 6> Lambda;
  Eigen::Matrix<double, 7, 6> J_plus;
  Eigen::Matrix<double, 6, 7> J_T_plus;
  Eigen::Matrix<double, 6, 6> S_P;
  Eigen::Matrix<double, 6, 6> S_f;
  Eigen::Matrix<double, 7, 1> tau_0;
  Eigen::Matrix<double, 7, 7> I;
  Eigen::Matrix<double, 6, 1> f;
  Eigen::Matrix<double, 7, 1> gamma;
  Eigen::Matrix<double, 7, 1> dgamma;
  Eigen::Matrix<double, 7, 1> gamma_prev;
  Eigen::Matrix<double, 7, 1> vec;
  Eigen::Matrix<double, 7, 1> e_n;
  Eigen::Matrix<double, 7, 1> Phi;
  Eigen::Matrix<double, 6, 7> dJ;
  Eigen::Matrix<double, 7, 6> J_inverse;
  Eigen::Matrix<double, 7, 6> J_inverse_prev;
  Eigen::Matrix<double, 7, 6> dJ_inverse;
  Eigen::Matrix<double, 6, 7> J_prev;
  Eigen::Matrix<double, 6, 7> J_prev_prev;
  Eigen::Matrix<double, 7, 1> ddq;
  Eigen::Matrix<double, 7, 1> dq_prev;
  Eigen::Matrix<double, 7, 1> dq_prev_prev;
  
  Eigen::Vector3d position_d;
  Eigen::Vector3d velocity_d;
  Eigen::Vector3d acceleration_d;
  Eigen::Quaterniond orientation_d;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

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
