// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hennings_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool HenningImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

//   sub_equilibrium_pose_ = node_handle.subscribe(
//       "equilibrium_pose", 20, &HenningImpedanceController::equilibriumPoseCallback, this,
//       ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("HenningImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "HenningImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HenningImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "HenningImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HenningImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "HenningImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "HenningImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HenningImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);
//   dynamic_server_compliance_param_->setCallback(
//       boost::bind(&HenningImpedanceController::complianceParamCallback, this, _1, _2));

  // Variable Initialization
  position_d.setZero();
  orientation_d.coeffs() << 0.0, 0.0, 0.0, 1.0;
//   position_d_target_.setZero();
//   orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  K_cartesian.setZero(); // set stiffness matrix to zero
  D_cartesian.setZero(); // set damping matrix to zero
  
  
  // For PID Controller
  eint.setZero();
  
  K_P.setIdentity();
  K_I.setIdentity();
  K_D.setIdentity();
  
  K_P.diagonal() << 300, 300, 300, 60, 60, 60;
  K_D.diagonal() << 10, 10, 10, 5, 5, 5;
  K_I.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
  
//   For Impedance Controller
  K_d.setIdentity();
  D_d.setIdentity();
  M_d.setIdentity();
  I.setIdentity();

  M_d << M_d * 500;
  D_d << D_d * 10;
  K_d << K_d * 300;
  
  return true;
}

void HenningImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
      
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  jacobian_prev << jacobian;
  jacobian_prev_prev << jacobian;
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
  dq_prev << dq;
  dq_prev_prev << dq;
  
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d = initial_transform.translation();
  orientation_d = Eigen::Quaterniond(initial_transform.linear());
//   position_d_target_ = initial_transform.translation();
//   orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void HenningImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
    
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  
  ddq << ((dq_prev - dq_prev_prev) / period.toSec() + (dq - dq_prev) / period.toSec()) / 2;
  djacobian << ((jacobian_prev - jacobian_prev_prev) / period.toSec() + (jacobian - jacobian_prev) / period.toSec()) / 2;
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d curr_position(transform.translation());
  Eigen::Quaterniond curr_orientation(transform.linear());
  
  Eigen::VectorXd curr_velocity(6);
  curr_velocity << jacobian * dq;
  
  Eigen::VectorXd curr_acceleration(6);
  curr_acceleration << jacobian * ddq + djacobian * dq;
  
  // desired position and velocity
//   position_d <<  r * cos(omega * time.toSec()) + 0.55 - r,
//                  r * sin(omega * time.toSec()), 
//                  0.52;
//                  
//   velocity_d <<  -r * omega * sin(omega * time.toSec()), 
//                   r * omega * cos(omega * time.toSec()),
//                   0;
//                   
//   acceleration_d <<  -r * omega * omega * cos(omega * time.toSec()), 
//                      -r * omega * omega * sin(omega * time.toSec()), 
//                       0;
  
  // Steady position
  position_d <<  0.2,
                 0.2, 
                 0.52;
                 
  velocity_d.setZero();
                  
  acceleration_d.setZero();
  
                  
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> derror;
  Eigen::Matrix<double, 6, 1> dderror;
  error.setZero();
  derror.setZero();
  dderror.setZero();
  
  // Position Error
  error.head(3) << curr_position - position_d;
  
  // orientation error
  if (orientation_d.coeffs().dot(curr_orientation.coeffs()) < 0.0) {
    curr_orientation.coeffs() << -curr_orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(curr_orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);
  
  // Velocity Error
  derror << curr_velocity;
  derror.head(3) << derror.head(3) - velocity_d;
  
  // Acceleration Error
  dderror << curr_acceleration;
  dderror.head(3) << dderror.head(3) - acceleration_d;
  
  
  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  
// Force Control
// External force on TCP

  f << D_d * derror + K_d * error;  
  
  M_r << (jacobian * mass.inverse() * jacobian.transpose()).inverse();
  
  tau_task << jacobian.transpose() * M_r * (M_d.inverse() * (D_d * derror + K_d * error)
            + jacobian * mass.inverse() * (coriolis /*+ gravity*/) - djacobian * dq) 
            + jacobian.transpose() * (I - M_r * M_d.inverse()) * f;
   
  
//   // PID controller
//   eint << eint + error * period.toSec();
//   
//   
//   // Cartesian PID control with damping ratio = 1
//   tau_task << jacobian.transpose() * (-K_P * error -K_D * derror -K_I * eint);
  
 std::cout << "Error" <<std::endl<< error <<std::endl; 
     
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
                       
                       
  // Desired torque
  tau_d << tau_task /*+ coriolis*/ + tau_nullspace;
  
  
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
  dq_prev << dq;
  dq_prev_prev << dq_prev;
  jacobian_prev << jacobian;
  jacobian_prev_prev << jacobian_prev;
  

//   // update parameters changed online either through dynamic reconfigure or through the interactive
//   // target by filtering
//   cartesian_stiffness_ =
//       filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
//   cartesian_damping_ =
//       filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
//   nullspace_stiffness_ =
//       filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
//   std::lock_guard<std::mutex> position_dtarget_mutex_lock(
//       position_and_orientation_d_target_mutex_);
//   position_d = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d;
//   orientation_d = orientation_d.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> HenningImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// void HenningImpedanceController::complianceParamCallback(
//     franka_example_controllers::compliance_paramConfig& config,
//     uint32_t /*level*/) {
//   cartesian_stiffness_target_.setIdentity();
//   cartesian_stiffness_target_.topLeftCorner(3, 3)
//       << config.translational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_stiffness_target_.bottomRightCorner(3, 3)
//       << config.rotational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.setIdentity();
//   // Damping ratio = 1
//   cartesian_damping_target_.topLeftCorner(3, 3)
//       << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.bottomRightCorner(3, 3)
//       << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
//   nullspace_stiffness_target_ = config.nullspace_stiffness;
// }

// void HenningImpedanceController::equilibriumPoseCallback(
//     const geometry_msgs::PoseStampedConstPtr& msg) {
//   std::lock_guard<std::mutex> position_dtarget_mutex_lock(
//       position_and_orientation_d_target_mutex_);
//   position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
//   orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//       msg->pose.orientation.z, msg->pose.orientation.w;
//   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//   }
// }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HenningImpedanceController,
                       controller_interface::ControllerBase)
