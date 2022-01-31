// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/mc_rtc_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool MCRTCExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MCRTCExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MCRTCExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("MCRTCExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MCRTCExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("MCRTCExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MCRTCExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("MCRTCExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("MCRTCExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void MCRTCExampleController::starting(const ros::Time& /*time*/) {
// get the current state of the robot and all the joint angles, joint velocities and torques
    
  franka::RobotState robot_state = state_handle_->getRobotState(); 

//  
/*  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(robot_state.q.data());      // initial angles
    std::cout << q_initial << std::endl;
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  Eigen::Vector3d TCP_initial_pos = initial_transform.translation();
      std::cout <<"Initial TCP position"<<std::endl<< TCP_initial_pos<< std::endl;
 
  Eigen::Quaterniond TCP_init_orient = Eigen::Quaterniond(initial_transform.linear());
  Eigen::Vector3d TCP_target_pos = initial_transform.translation();
  Eigen::Quaterniond TCP_target_orient = Eigen::Quaterniond(initial_transform.linear());
   std::cout << TCP_target_orient.w() << std::endl 
             << TCP_target_orient.x() << std::endl
             << TCP_target_orient.y() << std::endl
             << TCP_target_orient.z() << std::endl;
             
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(robot_state.dq.data());            
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_m_initial(robot_state.tau_J.data()); 
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity((model_handle_->getGravity()).data()); 
  
  */
//      
  dt = mcrtccontroller.timestep(); // might not be used
  std::vector<double> q(robot_state.q.begin(), robot_state.q.end());
  std::vector<double> dq(robot_state.dq.begin(), robot_state.dq.end());
  std::vector<double> tau_m(robot_state.tau_J.begin(), robot_state.tau_J.end());
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  
  for(size_t i = 0; i < 7; i++)
  {
      tau_m[i] = tau_m[i];// - gravity_array[i];
  }

  // Update sensor data for mc_rtc
  mcrtccontroller.setEncoderValues(q);              // here we sent the data to the PandaExampleController
  mcrtccontroller.setEncoderVelocities(dq);
  mcrtccontroller.setJointTorques(tau_m);
  // Initialize the controller
  mcrtccontroller.running = true;
  mcrtccontroller.init(q); // to investigate: causing [error] Already logging an entry named perf_FSM_UpdateContacts
  std::cout << "[MCRTC] Starting the mc rtc controller.";
}

void MCRTCExampleController::update(const ros::Time& /*time*/, const ros::Duration& period) {
 // get the current state of the robot and all the joint angles, joint velocities and torques  
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::vector<double> q(robot_state.q.begin(), robot_state.q.end());
  std::vector<double> dq(robot_state.dq.begin(), robot_state.dq.end());
  std::vector<double> tau_m(robot_state.tau_J.begin(), robot_state.tau_J.end());
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  
/*  
// My PD controller
   std::vector<double> error;   
   std::vector<double> errord;
   std::vector<double> qd_des = {0,0,0,0,0,0,0};
   std::vector<double> tau;
   std::vector<double> q_des = { -0.00144983,-0.00836253,  0.00164896,    -1.58354,-0.000519467,     1.58743,    0.786379};
   double Kp = 0.8;
   double Kd = 0.5;
   double torques_new = 0.0;
   
   for(size_t i = 0; i < 7; ++i)
   {
     
     error[i] = q_des[i] - q[i];
     errord[i] = qd_des[i] - dq[i];
     tau[i] = Kp * error[i] + Kd * errord[i];
     
     torques_new = tau_m[i] + tau[i];
    joint_handles_[i].setCommand(torques_new);
   }
*/


  
  for(size_t i = 0; i < 7; i++)
  {
      tau_m[i] = tau_m[i];// - gravity_array[i];
  }
  
  // Update sensor data for mc_rtc
  mcrtccontroller.setEncoderValues(q);          // here we sent the data to the PandaExampleController??
  mcrtccontroller.setEncoderVelocities(dq);
  mcrtccontroller.setJointTorques(tau_m);
  
  auto & real = mcrtccontroller.controller().realRobots().robot();          
  
  // Map joint ids to the right indexes
   const auto & rjo = mcrtccontroller.robot().refJointOrder();          
//   for(size_t i = 0; i < rjo.size(); ++i)
//   {
//     auto jIndex = mcrtccontroller.robot().jointIndexByName(rjo[i]);
//     real.mbc().jointTorque[jIndex][0] = tau_m[i];
//   }
  
  // run the controller
  // TODO: check if the controller is running properly
  mcrtccontroller.run();
  
  // Retrieve the desired joint torques from mc_rtc
  auto & tau_d = mcrtccontroller.robot().mbc().jointTorque;         // get the torques computed by the controller
  
  // Debug outputs
//   for (size_t i = 0; i < rjo.size(); i++)
//   {
//       std::cout << "[MCRTC] Joint " << i << " " << rjo[i] << "\n";
//       std::cout << "[MCRTC] JointIndexByName " << rjo[i] << " " << mcrtccontroller.robot().jointIndexByName(rjo[i]) << "\n";
//   }


  double tau_d_temp = 0.0;
  int idx = 0;
  for (size_t i = 0; i < 7; ++i) {
    idx = mcrtccontroller.robot().jointIndexByName(rjo[i]);
    tau_d_temp = tau_d[idx][0];
//     std::cout << "[MCRTC] Desired torque: " << i << ", " << tau_d_temp << "\n";
    joint_handles_[i].setCommand(tau_d_temp);// + gravity_array[i]);
  }
  
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MCRTCExampleController,
                       controller_interface::ControllerBase)
