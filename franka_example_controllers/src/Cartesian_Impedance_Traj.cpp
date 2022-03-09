// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/Cartesian_Impedance_Traj.h>

#include <cmath>
#include <memory>
#include <fstream>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_example_controllers {

bool CartesianImpedanceTrajectory::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "CartesianImpedanceTrajectory: Invalid or no joint_names parameters provided, "
            "aborting controller init!");
        return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceTrajectory: Exception getting joint handles: " << ex.what());
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
    
    notFirstRun = false;
    
    // define time of quintic trajectory
    T   =   3;
    a3  =   10 / pow(T, 3);
    a4  = - 15 / pow(T, 4);
    a5  =    6 / pow(T, 5);
    
    s   =    0;
    ds  =    0;
    dds =    0;
    
    return true;
}

void CartesianImpedanceTrajectory::starting(const ros::Time& /*time*/) {

    franka::RobotState initial_state = state_handle_->getRobotState();

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    TransformationMatrix= Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
    
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        
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

void CartesianImpedanceTrajectory::update(const ros::Time& time, const ros::Duration& period) {
    
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7>  coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 49> mass_array = model_handle_->getMass();
    
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


///////////////////////////////////////////////   FOLLOW TRAJECTORY  /////////////////////////////////////////
 
    position_d_target << X(0,0),   X(0,1),   X(0,2);

    orientation_d_target.coeffs() << Quats(0,1),  Quats(0,2),  Quats(0,3), Quats(0,0);
                          
    omega_d_global.setZero();
    domega_d_global.setZero();   
                            
    if (s <= 1) {
        s =       a3 * pow(time.toSec(), 3) +      a4 * pow(time.toSec(), 4) +      a5 * pow(time.toSec(), 5);
        ds =  3 * a3 * pow(time.toSec(), 2) +  4 * a4 * pow(time.toSec(), 3) +  5 * a5 * pow(time.toSec(), 4);
        dds = 6 * a3 *         time.toSec() + 12 * a4 * pow(time.toSec(), 2) + 20 * a5 * pow(time.toSec(), 3); 
        
        // Slowly move to start of Trajectory
        position_d     << position_init + s * (position_d_target - position_init);
        velocity_d     << ds * (position_d_target - position_init);
        acceleration_d << dds * (position_d_target - position_init);  
        orientation_d  =  orientation_d.slerp(10/T * s/1000, orientation_d_target);
    }
    else {
        // FOLLOW TRAJECTORY FROM MATLAB
            
        position_d     <<   X(i,0),   X(i,1),   X(i,2);  //X.row(i) does not work
        velocity_d     <<  dX(i,0),  dX(i,1),  dX(i,2);
        acceleration_d << ddX(i,0), ddX(i,1), ddX(i,2);
        
        orientation_d.coeffs() <<  Quats(i,1),  Quats(i,2),  Quats(i,3), Quats(i,0);
        omega_d_global         <<  omega(i,0),  omega(i,1),  omega(i,2);
        domega_d_global        << domega(i,0), domega(i,1), domega(i,2);
    
        if (time.toSec() >= i * ts(0,0) + T && time.toSec() >= ts(0,0) + T && i < X.rows() - 1) {
            i++;
        } 
    }
    
/////////////////////////////////////////// COMPUTE ERRORS ///////////////////////////////////////////////////
    
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

//     q_nullspace << q;
    
/////////////////////////////////////////// end of controller ///////////////////////////////////////////////  
    
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d(i));
    }
    
//     std::cout << "Error" <<std::endl<< error * 1000 <<std::endl; 

    // UPDATE VALUES FOR FINITE DIFFERENCES
    jacobian_prev << jacobian;
    Lambda_prev << Lambda;
    notFirstRun = true;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceTrajectory::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
    
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}
  
void CartesianImpedanceTrajectory::Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input, 
    const Eigen::MatrixXd& y_prev,   Eigen::MatrixXd& y) {
        
    y.resize(rows,cols);
    y << (1 - filter_param) * y_prev + filter_param * input;
}

// Function to lad csv files; Source: https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
template<typename M> M CartesianImpedanceTrajectory::load_csv (const std::string & path) {
    std::ifstream indata;
    indata.open(path);
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

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceTrajectory,
                       controller_interface::ControllerBase)
