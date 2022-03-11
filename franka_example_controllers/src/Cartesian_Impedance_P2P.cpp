// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/Cartesian_Impedance_P2P.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianImpedanceP2P::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

//     sub_equilibrium_pose_ = node_handle.subscribe(
//         "equilibrium_pose", 20, &CartesianImpedanceP2P::equilibriumPoseCallback, this,
//         ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceP2P: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "CartesianImpedanceP2P: Invalid or no joint_names parameters provided, "
            "aborting controller init!");
        return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handle.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceP2P: Exception getting joint handles: " << ex.what());
        return false;
        }
    }

    // Variable Initialization
    position_d                    << 0.5,   0, 0.5;
    orientation_d.coeffs()        << 0.0, 1.0, 0.0, 0.0;
    position_d_target             << 0.5,   0, 0.5;
    orientation_d_target.coeffs() << 0.0, 1.0, 0.0, 0.0;  
    
    error.setZero();
    derror.setZero();
    dderror.setZero();
    
    //   For Impedance Controller
    K_p.diagonal() << 600, 600, 600, 30, 30, 10;
    K_d.diagonal() << 30, 30, 30, 1.5, 1.5, 1.5;
    
//     For the easiest controller  
//     K_p.diagonal() << 500, 500, 400, 18, 18, 8;
//     K_d.diagonal() << 30, 30, 30, 0.3, 0.3, 0.3;
    
    C_hat.setZero();
    
    // Nullspace stiffness and damping
    K_N.setIdentity();
    D_N.setIdentity();
    K_N << K_N * 15;
    D_N << D_N * 0.5 * sqrt(K_N(0,0));  
    I.setIdentity();
    
    // To check max values
    dq_max << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;
    tau_max << 87, 87, 87, 87, 12, 12, 12;
    tau_min << -tau_max;
    
    notFirstRun = false;
    
    // define time of quintic trajectory
    T   =   10;
    a3  =   10 / pow(T, 3);
    a4  = - 15 / pow(T, 4);
    a5  =    6 / pow(T, 5);
    
    s   =    0;
    ds  =    0;
    dds =    0;
    
    return true;
}

void CartesianImpedanceP2P::starting(const ros::Time& /*time*/) {

    franka::RobotState initial_state = state_handle->getRobotState();

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    TransformationMatrix = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
    
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
        
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    jacobian_prev << jacobian;
    djacobian.setZero();
    
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());

    // set equilibrium point to current state
    position_init        =   TransformationMatrix.translation();
    orientation_init     =   TransformationMatrix.rotation();
    position_d           <<  position_init; 
    orientation_d        =   orientation_init;
    position_d_target    <<  position_init; 
    orientation_d_target =   orientation_init;
    q_nullspace          <<  q_initial;
}

void CartesianImpedanceP2P::update(const ros::Time& /*time*/, const ros::Duration& period) {
    
    mytime = mytime + period.toSec();
    
    // get state variables
    franka::RobotState robot_state = state_handle->getRobotState();
    std::array<double, 7>  coriolis_array = model_handle->getCoriolis();
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 49> mass_array = model_handle->getMass();
    
    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
        
    mass_inv << mass.inverse();
    
    TransformationMatrix = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    curr_position        =  TransformationMatrix.translation();
    curr_orientation     =  TransformationMatrix.rotation();
    curr_velocity        << jacobian * dq;
   
    if (notFirstRun){ // in first run, period.toSec() is zero
            djacobian << (jacobian - jacobian_prev) / period.toSec();
    }
    
    // FILTER SIGNALS 
    Eigen::MatrixXd y(7,7);
    Filter(0.005, djacobian.rows(), djacobian.cols(), djacobian, djacobian_filtered, y);
    djacobian_filtered << y;

//////////////////////////////////////////////   POINT to POINT MOVEMENT  /////////////////////////////////////////
    
    position_d_target << 0.3, 0, 0.8; 
//     position_d_target << position_init;
    
    angles_d_target   <<   0, 60,  0;  // x-axis (roll, points forward)// y-axis (pitch, points to the right)// z-axis (yaw, points downwards)
                            
    orientation_d_target =    Eigen::AngleAxisd(angles_d_target(0) * M_PI/180 + M_PI, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(angles_d_target(1) * M_PI/180       , Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(angles_d_target(2) * M_PI/180       , Eigen::Vector3d::UnitZ()); 
                            
    omega_d_global.setZero();
    domega_d_global.setZero();
    
    if (s <= 1) {
        s =       a3 * pow(mytime, 3) +      a4 * pow(mytime, 4) +      a5 * pow(mytime, 5);
        ds =  3 * a3 * pow(mytime, 2) +  4 * a4 * pow(mytime, 3) +  5 * a5 * pow(mytime, 4);
        dds = 6 * a3 *         mytime + 12 * a4 * pow(mytime, 2) + 20 * a5 * pow(mytime, 3); 
        
        //  // Point to Point movements
        position_d     << position_init + s * (position_d_target - position_init);
        velocity_d     << ds * (position_d_target - position_init);
        acceleration_d << dds * (position_d_target - position_init);  
        orientation_d  =  orientation_d.slerp(10/T * s/1000, orientation_d_target);
    }
    else {
        position_d  << position_d_target;
        velocity_d.setZero();
        acceleration_d.setZero();
        orientation_d = orientation_d.slerp(0.008, orientation_d_target);
        
    }
    
///////////////////////////////////// COMPUTE ERRORS //////////////////////////////////////////////////////
    
    // POSITION ERROR
    error.head(3)  << curr_position - position_d;
    
    // ORIENTATION ERROR (According to Silciano)
    if (orientation_d.coeffs().dot(curr_orientation.coeffs()) < 0.0) {
        curr_orientation.coeffs() << -curr_orientation.coeffs(); // take short way around the circle and by using positve dot product of quaternions
    }
    
    quat_d << orientation_d.x(), orientation_d.y(), orientation_d.z();
    quat_c << curr_orientation.x(), curr_orientation.y(), curr_orientation.z();
                
    error.tail(3)  << -( curr_orientation.w() * quat_d - orientation_d.w() * quat_c - quat_d.cross(quat_c));  
    
    // VELOCITY ERROR
    derror         << curr_velocity;
    derror.head(3) << derror.head(3) - velocity_d;
    derror.tail(3) << derror.tail(3) - omega_d_global;

    // ACCELERATION VECTOR
    Eigen::VectorXd ddx(6);
    ddx.setZero();
    ddx.head(3)   << acceleration_d; 
    ddx.tail(3)   << domega_d_global;    
    
////////////////////////////////////////// CONTROLLER //////////////////////////////////////////////////////
    
///////////////////  Paper: Cartesian Impedance Control of Redundant Robots:       /////////////////////////
//////////////////          Recent Results with the DLR-Light-Weight-Arms, DLR    //////////////////////////

    Lambda << (jacobian * mass_inv * jacobian.transpose()).inverse();
    
    if (notFirstRun) {
        C_hat << 0.5 * (Lambda - Lambda_prev) / period.toSec();
    } 
    
    F_tau << Lambda * ddx - K_d * derror - K_p * error - C_hat * derror - Lambda * djacobian_filtered * dq;
            
//     F_tau <<   -(K_d * derror + K_p * error);
    
    tau_task << jacobian.transpose() * F_tau;
    
//     nullspace PD control
    N << I - jacobian.transpose() * Lambda * jacobian * mass_inv;
    tau_nullspace << N * (-K_N * (q - q_nullspace) - D_N * dq);
    
//     Desired torque
    tau_d << tau_task + coriolis + tau_nullspace;

    q_nullspace << q;
    
/////////////////////////////////////////// end of controller ///////////////////////////////////////////////  
    
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
        joint_handle[i].setCommand(tau_d(i));
    }
    
//     std::lock_guard<std::mutex> position_d_target_mutex_lock(
//         position_and_orientation_d_target_mutex_);
    
//     std::cout << "Error" <<std::endl<< error * 1000 <<std::endl; 

    // UPDATE VALUES FOR FINITE DIFFERENCES
    jacobian_prev << jacobian;
    Lambda_prev << Lambda;
    notFirstRun = true;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceP2P::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
    
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}
  
void CartesianImpedanceP2P::Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input, 
    const Eigen::MatrixXd& y_prev,   Eigen::MatrixXd& y) {
        
    y.resize(rows,cols);
    y << (1 - filter_param) * y_prev + filter_param * input;
}

// void CartesianImpedanceP2P::equilibriumPoseCallback(
//     const geometry_msgs::PoseStampedConstPtr& msg) {
//     std::lock_guard<std::mutex> position_d_target_mutex_lock(
//         position_and_orientation_d_target_mutex_);
//     position_d_target << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//     Eigen::Quaterniond last_orientation_d_target(orientation_d_target);
//     orientation_d_target.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//         msg->pose.orientation.z, msg->pose.orientation.w;
//     if (last_orientation_d_target.coeffs().dot(orientation_d_target.coeffs()) < 0.0) {
//     orientation_d_target.coeffs() << -orientation_d_target.coeffs();
//     }
// }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceP2P,
                       controller_interface::ControllerBase)
