// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hennings_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// #include <franka_example_controllers/pseudo_inversion.h>

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
  position_d_target.setZero();
  orientation_d_target.coeffs() << 0.0, 0.0, 0.0, 1.0;

  
  // For PID Controller
  eint.setZero();
  
  K_P.setIdentity();
  K_I.setIdentity();
  K_D.setIdentity();
  
  K_P.diagonal() << 300, 300, 300, 60, 60, 60;
  K_D.diagonal() << 10, 10, 10, 5, 5, 5;
  K_I.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
  
//   For Impedance Controller
  
  K_p.setIdentity(); 
  K_d.setIdentity(); 
  M_d.setIdentity(); 
  
  S_P.setIdentity();
  
  M_d.diagonal() << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  K_p.diagonal() << 350, 350, 350, 200, 200, 200;
//     K_p.diagonal() << 35, 35, 35, 20, 20, 20;  // works better for osqp eigen
  K_d.diagonal() << 20, 20, 20, 30, 30, 30;
  
  // Nullspace stiffness and damping
  K_N.setIdentity();
  D_N.setIdentity();
  K_N << K_N * 30;
  D_N << D_N * 0.5 * sqrt(K_N(0,0));  
  
  // To check max values
  ddq_max << 15, 7.5, 10, 12.5, 15, 20, 20;
  dq_max << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;
  tau_max << 87, 87, 87, 87, 12, 12, 12;
  tau_min << -tau_max;
  
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
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
  dq_prev << dq;
  ddq_filtered << dq;
  
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_init = initial_transform.translation();
  orientation_init = Eigen::Quaterniond(initial_transform.linear());
  position_d << position_init; 
  orientation_d = orientation_init;
  
  angles_init << orientation_init.toRotationMatrix().eulerAngles(0, 1, 2);
//   std::cout<<std::endl<< "Angles" << std::endl << curr_angles << std::endl;
//   std::cout<<"Initial Orient" << std::endl<<orientation_d.coeffs()<<std::endl;
  
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(initial_state.O_F_ext_hat_K.data());
  F_ext_prev << F_ext;
  F_ext_filtered << F_ext;
  
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void HenningImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
    
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(robot_state.O_F_ext_hat_K.data());
  
  Eigen::Matrix<double, 7, 7> mass_inv;
  mass_inv << mass.inverse();
  
  djacobian << (jacobian - jacobian_prev) / period.toSec();
  ddq << (dq - dq_prev) / period.toSec();
  
  // Filter signals
  
  Eigen::MatrixXd y(7,7);
  Filter(0.001, F_ext.rows(), F_ext.cols(), F_ext, F_ext_prev, F_ext_filtered, y);
  F_ext_filtered << y;
  
  Filter(0.001, ddq.rows(), ddq.cols(), ddq, ddq_prev, ddq_filtered, y);
  ddq_filtered << y;

  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d curr_position(transform.translation());
  Eigen::Quaterniond curr_orientation(transform.linear());
//   Eigen::Vector3d curr_angles;
//   curr_angles << curr_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  
  Eigen::VectorXd curr_velocity(6);
  curr_velocity << jacobian * dq;
  
  Eigen::VectorXd curr_acceleration(6);
  curr_acceleration << jacobian * ddq_filtered + djacobian * dq;
  
//  //desired position and velocity

//  // Point to Point movements

  position_d_target << 0.3,
                       0,
                       0.6;
//   position_d_target << position_init;
  
  angles_d <<  0  * M_PI/180 + M_PI,  // x-axis (roll)
               0  * M_PI/180,         // y-axis (pitch)
               0  * M_PI/180;         // z-axis (yaw) compared to base frame in intial position

  orientation_d_target =    Eigen::AngleAxisd(angles_d(0), Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(angles_d(1), Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(angles_d(2), Eigen::Vector3d::UnitZ()); 
  
  // Damp the motion between the points
  double motion_damp = 0.5 * 1e-3 * (2 - (position_d_target - position_init).norm());   // or just motion damp = 0.0008;

  position_d << motion_damp * position_d_target + (1.0 - motion_damp) * position_d;                       
  
  orientation_d = orientation_d.slerp(0.0008, orientation_d_target);
                          
// For time dependent Trajectoires
  
//   velocity_d.setZero();
//   acceleration_d.setZero();
//   position_d << position_init(0) + velocity_d(0) * time.toSec() + 0.5 * acceleration_d(0) * time.toSec() * time.toSec(),
//                 position_init(1) + velocity_d(1) * time.toSec() + 0.5 * acceleration_d(1) * time.toSec() * time.toSec(), 
//                 position_init(2) + velocity_d(2) * time.toSec() + 0.5 * acceleration_d(2) * time.toSec() * time.toSec();
  
 // Circular Motion
  
//   position_d <<  r * cos(omega * time.toSec()) + position_init(0) - r,
//                  r * sin(omega * time.toSec()) + position_init(1), 
//                  position_init(2);
//             
//                  
//   velocity_d <<  -r * omega * sin(omega * time.toSec()), 
//                   r * omega * cos(omega * time.toSec()),
//                   0;
//                   
//   acceleration_d <<  -r * omega * omega * cos(omega * time.toSec()), 
//                      -r * omega * omega * sin(omega * time.toSec()), 
//                       0;
  
//   alpha = 0 + omega_d_global(0) * time.toSec() + 0.5 * domega_d_global(0) * time.toSec() * time.toSec();   // x-axis (roll)
//   beta  = 0 + omega_d_global(1) * time.toSec() + 0.5 * domega_d_global(1) * time.toSec() * time.toSec();   // y-axis (pitch)
//   gamma = 0 + omega_d_global(2) * time.toSec() + 0.5 * domega_d_global(2) * time.toSec() * time.toSec();   // z-axis (yaw) 
//   
//   angles_d <<  M_PI + M_PI/180 * alpha, 
//                     + M_PI/180 * beta,
//                     + M_PI/180 * gamma;
// 
//   orientation_d =    Eigen::AngleAxisd(angles_d(0), Eigen::Vector3d::UnitX())
//                    * Eigen::AngleAxisd(angles_d(1), Eigen::Vector3d::UnitY())
//                    * Eigen::AngleAxisd(angles_d(2), Eigen::Vector3d::UnitZ());              
               
  
  Eigen::VectorXd ddx(6);
  ddx.setZero();
  ddx.head(3) << acceleration_d; 
  ddx.tail(3) << domega_d_global;    
  
  // compute errors to desired pose
 
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> derror;
  Eigen::Matrix<double, 6, 1> dderror;
  error.setZero();
  derror.setZero();
  dderror.setZero();
  
  // Position Error
  error.head(3) << curr_position - position_d;
  
  // Orientation error 
  if (orientation_d.coeffs().dot(curr_orientation.coeffs()) < 0.0) {
      // take short way around the circle and by using positve dot product of quaternions
    curr_orientation.coeffs() << -curr_orientation.coeffs();
  }
  
  Eigen::Quaterniond error_quaternion(curr_orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  
  // Velocity Error
  derror << curr_velocity;
  derror.head(3) << derror.head(3) - velocity_d;
  derror.tail(3) << derror.tail(3) - omega_d_global;

  
  // Acceleration Error  // Might be unused
  dderror << curr_acceleration;
  dderror.head(3) << dderror.head(3) - acceleration_d;
  dderror.tail(3) << dderror.tail(3) - domega_d_global;
  
  // Compute Control
  // Allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  
   
///////////////////  Paper: Cartesian Impedance Control of Redundant Robots:       /////////////////////////
//////////////////          Recent Results with the DLR-Light-Weight-Arms, DLR    /////////////////////////
  
// Comment: Works good but cannot find the optimal nullspace joint angles
  
 
//   std::cout <<"External Force:"<<std::endl<< F_ext_filtered <<std::endl;
  F_ext_filtered.setZero();

  Lambda << (jacobian * mass_inv * jacobian.transpose()).inverse();
   
  F_tau = Lambda * ddx - Lambda * M_d.inverse() * (K_d * derror + K_p * error) + (Lambda * M_d.inverse() - I) * F_ext_filtered - Lambda * djacobian * dq;
   
  tau_task = jacobian.transpose() * F_tau;
  
  // nullspace PD control
 
  tau_nullspace << -K_N * (q - q_nullspace) - D_N * dq;
     
  N << Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * Lambda * jacobian * mass_inv;
  
  // Desired torque
  tau_d << tau_task + coriolis + N * tau_nullspace;
  
  q_nullspace << q;
 
  
  
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
//    std::cout << "Error" <<std::endl<< error <<std::endl; 

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
  F_ext_prev << F_ext;
  ddq_prev << ddq;
  dq_prev << dq;


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

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HenningImpedanceController,
                       controller_interface::ControllerBase)
