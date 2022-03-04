// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hennings_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool HenningImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &HenningImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

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

  // Variable Initialization
  position_d.setZero();
  orientation_d.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target.setZero();
  orientation_d_target.coeffs() << 0.0, 0.0, 0.0, 1.0;  
  
  error.setZero();
  derror.setZero();
  dderror.setZero();
  
//   For Impedance Controller
    
// for Controller without F_ext  
  K_p.diagonal() << 600, 600, 600, 30, 30, 10;
  K_d.diagonal() << 30, 30, 30, 1.5, 1.5, 1.5;
  
// for controller with F_ext
//   M_d.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
//   K_p.diagonal() << 50, 50, 50, 100, 100, 100;
//   K_d.diagonal() << 25, 25, 25, 30, 30, 30;  
  
// For the easiest controller  
//   K_p.diagonal() << 500, 500, 400, 18, 18, 8;
//   K_d.diagonal() << 30, 30, 30, 0.3, 0.3, 0.3;
  
  // Nullspace stiffness and damping
  K_N.setIdentity();
  D_N.setIdentity();
  K_N << K_N * 15;
  D_N << D_N * 0.5 * sqrt(K_N(0,0));  
  
  // To check max values
  ddq_max << 15, 7.5, 10, 12.5, 15, 20, 20;
  dq_max << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;
  tau_max << 87, 87, 87, 87, 12, 12, 12;
  tau_min << -tau_max;
  
  flag = true;
  
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
  
  djacobian_prev.setZero();
  djacobian.setZero();
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
  dq_prev << dq;
  ddq_filtered << dq;
  
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_init = initial_transform.translation();
  orientation_init = Eigen::Quaterniond(initial_transform.rotation());
  position_d << position_init; 
  orientation_d = orientation_init;
  position_d_target << position_init; 
  orientation_d_target = orientation_init;
  
  // set nullspace equilibrium configuration to initial q
  q_nullspace = q_initial;
}

void HenningImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
    
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();
  
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
      
  Eigen::Matrix<double, 7, 7> mass_inv;
  mass_inv << mass.inverse();
  
  // in first run, period.toSec() is zero
  if (flag){
        djacobian << (jacobian - jacobian_prev) / 0.001;
        ddq << (dq - dq_prev) / 0.001;  
  }
  else {
        djacobian << (jacobian - jacobian_prev) / period.toSec();
        ddq << (dq - dq_prev) / period.toSec();
  }
  // Filter signals
  Eigen::MatrixXd y(7,7);
  
  Filter(0.001, ddq.rows(), ddq.cols(), ddq, ddq_prev, ddq_filtered, y);
  ddq_filtered << y;
  
  Filter(0.001, djacobian.rows(), djacobian.cols(), djacobian, djacobian_prev, djacobian_filtered, y);
  djacobian_filtered << y;

  
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d curr_position(transform.translation());
  Eigen::Quaterniond curr_orientation(transform.rotation());
  
  Eigen::VectorXd curr_velocity(6);
  curr_velocity << jacobian * dq;
  
  Eigen::VectorXd curr_acceleration(6);
  curr_acceleration << jacobian * ddq_filtered + djacobian_filtered * dq;
  
//  //desired position and velocity

//  // Point to Point movements

  position_d_target << 0.4, 0, 0.5;
                        
//   position_d_target << position_init;
                        
  velocity_d.setZero();
  acceleration_d.setZero();
  
  angles_d <<  0  * M_PI/180 + M_PI,  // x-axis (roll) (points forward)
               0  * M_PI/180,         // y-axis (pitch) (points to the right)
               0  * M_PI/180;         // z-axis (yaw) compared to base frame in intial position (points downwards)

  orientation_d_target =    Eigen::AngleAxisd(angles_d(0), Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(angles_d(1), Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(angles_d(2), Eigen::Vector3d::UnitZ()); 
                          
  omega_d_global.setZero();
  domega_d_global.setZero();
  
  // Damp the motion between the points

  double motion_damp = 0.001;   /*((position_d_target - position_init).norm() - (error.head(3)).norm()); */  // or just motion damp = 0.0008;

  position_d << motion_damp * position_d_target + (1.0 - motion_damp) * position_d;                       
  
  orientation_d = orientation_d.slerp(motion_damp, orientation_d_target);
                           
// Circular Motion
  
//   position_d <<  r * cos(omega * time.toSec()) + position_init(0) - r,
//                  r * sin(omega * time.toSec()) + position_init(1), 
//                  position_init(2);
//             
//                             
  
  Eigen::VectorXd ddx(6);
  ddx.setZero();
  ddx.head(3) << acceleration_d; 
  ddx.tail(3) << domega_d_global;    
  
  // compute errors to desired pose
  
  // Position Error
  error.head(3) << curr_position - position_d;
  
  // Orientation error 
  if (orientation_d.coeffs().dot(curr_orientation.coeffs()) < 0.0) {
    curr_orientation.coeffs() << -curr_orientation.coeffs(); // take short way around the circle and by using positve dot product of quaternions
  }
  
   // orientation Error (According to Silciano)
  
  Eigen::Vector3d quat_d, quat_c;
  
  quat_d << orientation_d.x(),
            orientation_d.y(),
            orientation_d.z();
            
  quat_c << curr_orientation.x(),
            curr_orientation.y(),
            curr_orientation.z();
            
  error.tail(3) << -( curr_orientation.w() * quat_d - orientation_d.w() * quat_c - quat_d.cross(quat_c));  
  
  // Velocity Error
  derror << curr_velocity;
  derror.head(3) << derror.head(3) - velocity_d;
  derror.tail(3) << derror.tail(3) - omega_d_global;
  
  // Compute Control
  // Allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  
   
///////////////////  Paper: Cartesian Impedance Control of Redundant Robots:       /////////////////////////
//////////////////          Recent Results with the DLR-Light-Weight-Arms, DLR    /////////////////////////
  
// Comment: Works good but cannot find the optimal nullspace joint angles
  

  Lambda << (jacobian * mass_inv * jacobian.transpose()).inverse();
  
  if (flag) {
        C_hat.setZero();
  }
  else {
    C_hat << 0.5 * (Lambda - Lambda_prev) / period.toSec();
  } 
  
  F_tau <<    Lambda * ddx - K_d * derror - K_p * error - C_hat * derror - Lambda * djacobian_filtered * dq;
  
//   F_tau <<    Lambda.inverse() * ddx - Lambda.inverse() * M_d.inverse() * (K_d * derror + K_p * error) - Lambda * djacobian_filtered * dq;
           
//    F_tau <<   -(K_d * derror + K_p * error);
   
  tau_task << jacobian.transpose() * F_tau;
  
  // nullspace PD control
 
  tau_nullspace << -K_N * (q - q_nullspace) - D_N * dq;
     
  N << Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * Lambda * jacobian * mass_inv;
  
  // Desired torque
  tau_d << tau_task + coriolis + N * tau_nullspace;
  
  q_nullspace << q;
 
/////////////////////////////////////////// end of controller ///////////////////////////////////////////////  
  
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  
  std::cout << "Error" <<std::endl<< error * 1000 <<std::endl; 

// Compare all values

  for (size_t i = 0; i < 7; ++i) {
    if (std::abs(tau_d[i]) > tau_max[i]) {
        std::cout << "Error: Torque at joint: "<< i <<" is too big!! ("<< tau_d[i] <<" Nm)"<<std::endl;
    }
    if (std::abs(ddq_filtered[i]) > ddq_max[i]) {
        std::cout << "Error: ddq at joint: "<< i <<" is too big!! ("<< ddq_filtered[i] <<" m/s^2)"<<std::endl;
    }
    if (std::abs(dq[i]) > dq_max[i]) {
        std::cout << "Error: dq at joint: "<< i <<" is too big!! ("<< dq[i] <<" m/s^2)"<<std::endl;
    }
  }

  // Update values
  
  jacobian_prev << jacobian;
  djacobian_prev << djacobian;
  Lambda_prev << Lambda;
  ddq_prev << ddq;
  dq_prev << dq;
  flag = false;
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
  
void HenningImpedanceController::Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input, 
  const Eigen::MatrixXd& input_prev,  const Eigen::MatrixXd& y_prev,   Eigen::MatrixXd& y) {
        
  y.resize(rows,cols);
  y << (1 - filter_param) * y_prev + filter_param * (input + input_prev) / 2;
}

void HenningImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target);
  orientation_d_target.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target.coeffs()) < 0.0) {
    orientation_d_target.coeffs() << -orientation_d_target.coeffs();
  }
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HenningImpedanceController,
                       controller_interface::ControllerBase)
