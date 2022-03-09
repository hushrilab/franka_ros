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

class CartesianImpedanceP2P : public controller_interface::MultiInterfaceController<
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
//   double r {0.1};
//   double omega_ {1};
  
  const double delta_tau_max_{1.0};   
  
  // Errors
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> derror;
  Eigen::Matrix<double, 6, 1> dderror;
  // for quaternion error
  Eigen::Vector3d quat_d;
  Eigen::Vector3d quat_c;

  bool flag;
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
  Eigen::Matrix<double, 7, 1> N;
  Eigen::Matrix<double, 6, 1> F_tau;
  Eigen::Matrix<double, 6, 7> djacobian;
  Eigen::Matrix<double, 6, 7> djacobian_prev;
  Eigen::Matrix<double, 6, 7> djacobian_filtered;
  Eigen::Matrix<double, 6, 7> jacobian_prev;
  Eigen::Matrix<double, 7, 1> dq_prev;
  Eigen::Matrix<double, 7, 1> ddq;
  Eigen::Matrix<double, 7, 1> ddq_max;
  Eigen::Matrix<double, 7, 1> dq_max;
  Eigen::Matrix<double, 7, 1> tau_max;
  Eigen::Matrix<double, 7, 1> tau_min;
  
  // For filtering
  Eigen::Matrix<double, 7, 1> ddq_prev;
  Eigen::Matrix<double, 7, 1> ddq_filtered;
  
  // for quintic trajectory
  double T;
  double a3;
  double a4;
  double a5;
  double s; 
  double ds;
  double dds;
  
  Eigen::Vector3d    curr_position;
  Eigen::Matrix<double, 6, 1>    curr_velocity;
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
  Eigen::Vector3d    omega_d_local;
  Eigen::Vector3d    omega_d_global;
  Eigen::Vector3d    domega_d_local;
  Eigen::Vector3d    domega_d_global;
  std::mutex position_and_orientation_d_target_mutex_;

//   // Equilibrium pose subscriber
//   ros::Subscriber sub_equilibrium_pose_;
//   void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_example_controllers
