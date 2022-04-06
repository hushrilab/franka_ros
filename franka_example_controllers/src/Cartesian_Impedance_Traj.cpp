
#include <franka_example_controllers/Cartesian_Impedance_Traj.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

namespace franka_example_controllers {

bool CartesianImpedanceTrajectory::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
    
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR("CartesianImpedanceTrajectory: Invalid or no joint_names parameters provided, "
            "aborting controller init!");
        return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Exception getting model handle from interface: "<< ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    } 
    catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Exception getting state handle from interface: "<< ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handle.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("CartesianImpedanceTrajectory: Exception getting joint handles: " << ex.what());
        return false;
        }
    }

    // Variable Initialization
    position_d                    << 0.5,   0, 0.5;
    orientation_d.coeffs()        << 0.0, 1.0, 0.0, 0.0;
    position_d_target             << 0.5,   0, 0.5;
    orientation_d_target.coeffs() << 0.0, 1.0, 0.0, 0.0;  
    ddx.setZero();
    error.setZero();
    derror.setZero();
    dderror.setZero();
    
    //   For Impedance Controller
    K_p.diagonal() << 700, 700, 700,  40,  40,  10;
    K_d.diagonal() <<  40,  40,  40, 1.5, 1.5, 0.1;
    
    // Factorization Damping Desgin
    D_eta.diagonal() << 0.3, 0.3, 0.3, 0.4, 0.4, 0.4;
    
    C_hat.setZero();
    
    // Nullspace stiffness and damping
    K_N.setIdentity();
    D_N.setIdentity();
    K_N << K_N * 15;
    D_N << D_N * sqrt(K_N(0,0));  
    I.setIdentity();
    
    external_load.setZero();
    
    notFirstRun = false;
    
    // define time of quintic trajectory
    T   =   5;
    a3  =   10 / pow(T, 3);
    a4  = - 15 / pow(T, 4);
    a5  =    6 / pow(T, 5);

    return true;
}

//////////////////////////////////////////// Gripper //////////////////////////////////////////////
 
    actionlib::SimpleActionClient<franka_gripper::MoveAction>   move1( "franka_gripper/move", true);
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp1("franka_gripper/grasp", true);

void CartesianImpedanceTrajectory::starting(const ros::Time& /*time*/) {

    franka::RobotState initial_state = state_handle->getRobotState();

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
    TransformationMatrix = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());
    
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    // set equilibrium point to current state
    position_init        =   TransformationMatrix.translation();
    orientation_init     =   TransformationMatrix.rotation();
    position_d           <<  position_init; 
    orientation_d        =   orientation_init;
    position_d_target    <<  position_init; 
    orientation_d_target =   orientation_init;
    q_nullspace          <<  q_initial;
    q_nullspace_init     <<  q_initial;
    jacobian_prev        <<  jacobian;
    djacobian.setZero(); 

}

void CartesianImpedanceTrajectory::update(const ros::Time& /*time*/, const ros::Duration& period) {
    
    mytime = mytime + period.toSec();
    
    // get state variables
    franka::RobotState robot_state        = state_handle->getRobotState();
    std::array<double, 7>  coriolis_array = model_handle->getCoriolis();
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 49> mass_array     = model_handle->getMass();
   
    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());  
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
    
    mass_inv             << mass.inverse();
    TransformationMatrix =  Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
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
                            
    if (s <= 1) {
        position_d_target             <<      X(0,0),      X(0,1),      X(0,2);
        orientation_d_target.coeffs() <<  Quats(0,1),  Quats(0,2),  Quats(0,3),  Quats(0,0);
        q_nullspace_target            << q_null(0,0), q_null(0,1), q_null(0,2), q_null(0,3), q_null(0,4), q_null(0,5), q_null(0,6);
        omega_d.setZero();
        domega_d.setZero();  
    
        s =       a3 * pow(mytime, 3) +      a4 * pow(mytime, 4) +      a5 * pow(mytime, 5);
        ds =  3 * a3 * pow(mytime, 2) +  4 * a4 * pow(mytime, 3) +  5 * a5 * pow(mytime, 4);
        dds = 6 * a3 *         mytime + 12 * a4 * pow(mytime, 2) + 20 * a5 * pow(mytime, 3); 

        // Slowly move to start of trajectory
        position_d     <<    position_init + s * (position_d_target - position_init);
        velocity_d     <<                   ds * (position_d_target - position_init);
        acceleration_d <<                  dds * (position_d_target - position_init);  
        orientation_d  =  orientation_d.slerp(10/T * s/1000, orientation_d_target);
        q_nullspace    << q_nullspace_init + s * (q_nullspace_target - q_nullspace_init);
    }
    else {
        // FOLLOW TRAJECTORY FROM MATLAB
        position_d             <<       X(i,0),       X(i,1),       X(i,2);  //X.row(i) does not work
        velocity_d             <<      dX(i,0),      dX(i,1),      dX(i,2);
        acceleration_d         <<     ddX(i,0),     ddX(i,1),     ddX(i,2);
        orientation_d.coeffs() <<   Quats(i,1),   Quats(i,2),   Quats(i,3),  Quats(i,0);
        omega_d                <<   omega(i,0),   omega(i,1),   omega(i,2);
        domega_d               <<  domega(i,0),  domega(i,1),  domega(i,2);
        q_nullspace            <<  q_null(i,0),  q_null(i,1),  q_null(i,2),  q_null(i,3),  q_null(i,4),  q_null(i,5),  q_null(i,6); 
        gripper_command        << gripper(i,0), gripper(i,1), gripper(i,2), gripper(i,3), gripper(i,4);
        K_p.diagonal()         <<   K_mat(i,0),   K_mat(i,1),   K_mat(i,2),   K_mat(i,3),   K_mat(i,4),   K_mat(i,5);
        D_eta.diagonal()       <<   D_mat(i,0),   D_mat(i,1),   D_mat(i,2),   D_mat(i,3),   D_mat(i,4),   D_mat(i,5);
        external_load          << ExtLoad(i,0), ExtLoad(i,1), ExtLoad(i,2), ExtLoad(i,3), ExtLoad(i,4), ExtLoad(i,5);
        
        if (mytime >= i * ts(0,0) + T && mytime >= ts(0,0) + T && i < X.rows() - 1) {
            i++;
            
            if (gripper_command(0) == 1){
                GripperMove(gripper_command(1), gripper_command(2));
            }
            if (gripper_command(0) == 2){
                GripperGrasp(gripper_command(1), gripper_command(2), gripper_command(3),gripper_command(4));
            }
        } 
    }
    if (i >= X.rows() - 1){    //free nullspace movement, when trajectory finished
        //q_nullspace << q;
        i = 0;
        mytime = 0;
    } 

/////////////////////////////////////////// COMPUTE ERRORS ///////////////////////////////////////////////////
    
    // POSITION ERROR
    error.head(3)  << curr_position - position_d;
    
    // ORIENTATION ERROR (According to Silciano)
    if (orientation_d.coeffs().dot(curr_orientation.coeffs()) < 0.0) {
        curr_orientation.coeffs() << -curr_orientation.coeffs(); // take short way around the circle and by using positve dot product of quaternions
    }
    
    quat_d         <<    orientation_d.x(),    orientation_d.y(),    orientation_d.z();
    quat_c         << curr_orientation.x(), curr_orientation.y(), curr_orientation.z();
                
    error.tail(3)  << -(curr_orientation.w() * quat_d - orientation_d.w() * quat_c - quat_d.cross(quat_c));  
    
    // VELOCITY ERROR
    derror.head(3) << curr_velocity.head(3) - velocity_d;
    derror.tail(3) << curr_velocity.tail(3) - omega_d;

    // ACCELERATION VECTOR
    ddx.head(3)    << acceleration_d; 
    ddx.tail(3)    << domega_d;  
    
////////////////////////////////////////// CONTROLLER //////////////////////////////////////////////////////
///////////////////  Paper: Cartesian Impedance Control of Redundant Robots:       /////////////////////////
//////////////////          Recent Results with the DLR-Light-Weight-Arms, DLR    //////////////////////////

    Lambda         << (jacobian * mass_inv * jacobian.transpose()).inverse();
    
    // Find best damping matrix: Factorization Damping Design
    K_p1           << K_p.sqrt();
    A              << Lambda.sqrt();
    K_d            << A * D_eta * K_p1 + K_p1 * D_eta * A;
    
    if (notFirstRun) {
        C_hat      << 0.5 * (Lambda - Lambda_prev) / period.toSec();
    } 
    
    F_tau          << Lambda * ddx - K_d * derror - K_p * error - C_hat * derror - Lambda * djacobian_filtered * dq - external_load;
    
    //     F_tau <<   -(K_d * derror + K_p * error) - external_load;
    tau_task       << jacobian.transpose() * F_tau;
    
//     nullspace PD control
    N              << I - jacobian.transpose() * Lambda * jacobian * mass_inv;
    tau_nullspace  << N * (-K_N * (q - q_nullspace) - D_N * dq);
    
//     Desired torque
    tau_d          << tau_task + coriolis + tau_nullspace;
    
/////////////////////////////////////////// end of controller ///////////////////////////////////////////////  
    
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    
    for (size_t i = 0; i < 7; ++i) {
        joint_handle[i].setCommand(tau_d(i));
    }

    // UPDATE VALUES FOR FINITE DIFFERENCES
    jacobian_prev << jacobian;
    Lambda_prev   << Lambda;
    notFirstRun   =  true; 
    
    // PRINT ERRORS
    Eigen::Quaterniond Error_quats;
    Error_quats.x() = error(3);
    Error_quats.y() = error(4);
    Error_quats.z() = error(5);
    Error_quats.w() = 1 - error(3) - error(4) - error(5);
    Eigen::Vector3d error_angles;
    error_angles = Error_quats.toRotationMatrix().eulerAngles(0, 1, 2);
    
    for(int j = 0; j < 3;j++){
        if(error_angles(j) > M_PI/2){
            error_angles(j) = error_angles(j) - M_PI;
        }
        if(error_angles(j) < -M_PI/2){
            error_angles(j) = error_angles(j) + M_PI;
        }
    }
//    std::cout << "POSITION ERROR in [mm]:" <<std::endl<< error.head(3) * 1000 <<std::endl<<std::endl; 
//    std::cout << "ORIENTATION ERROR in [deg]:" <<std::endl<< error_angles * 180/M_PI<<std::endl<<std::endl;
   
    // STREAM DATA
    if (true && j >= 100) {
        std::cout << curr_position.transpose()<<std::endl;
        std::cout << position_d.transpose()<<std::endl;
        std::cout << curr_orientation.coeffs().transpose()<<std::endl;
        std::cout << orientation_d.coeffs().transpose()<<std::endl;
        std::cout << error.head(3).transpose() * 1000 <<std::endl;
        std::cout << error_angles.transpose() * 180/M_PI<<std::endl;\
        std::cout << (jacobian.transpose() * (-(K_d * derror + K_p * error) - external_load)).transpose() <<std::endl;
        std::cout << tau_task.transpose() <<std::endl;
        std::cout << tau_d.transpose() <<std::endl;
        j = 0;
    }
    j++;
}

// void CartesianImpedanceTrajectory::stopping(const ros::Time& /*time*/) {
//     //std::cout<<"Stopping"<<std::endl;
//     GripperMove(0.07, 0.01);
// }

Eigen::Matrix<double, 7, 1> CartesianImpedanceTrajectory::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
    
    for (size_t i = 0; i < 7; i++) {
        double difference  = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}
  
void CartesianImpedanceTrajectory::Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input, 
    const Eigen::MatrixXd& y_prev, Eigen::MatrixXd& y) {
        
    y.resize(rows,cols);
    y << (1 - filter_param) * y_prev + filter_param * input;
}

void CartesianImpedanceTrajectory::GripperMove(double width, double speed) {

    franka_gripper::MoveGoal  move_goal;
    move_goal.width = width;
    move_goal.speed = speed;
    move1.sendGoal(move_goal);
}

void CartesianImpedanceTrajectory::GripperGrasp(double width, double speed, int force, double epsilon) {
    
    franka_gripper::GraspGoal grasp_goal;
    grasp_goal.width = width;
    grasp_goal.speed = speed;
    grasp_goal.force = force;
    grasp_goal.epsilon.inner = epsilon;
    grasp_goal.epsilon.outer = epsilon;
    grasp1.sendGoal(grasp_goal);
}

void CartesianImpedanceTrajectory::SetLoad(double mass_new, double mass_old, std::array<double, 3> vec2CoG ,double time, double t){
    
    if (m <= 1 && time < t && time > 0.002) {
        m             = 10 / pow(t, 3) * pow(time, 3) - 15 / pow(t, 4) * pow(time, 4) + 6 / pow(t, 5) * pow(time, 5);
        double F_g    = -9.81 * (mass_old + m * (mass_new - mass_old));
        external_load << 0, 0, F_g, vec2CoG[1] * F_g, - vec2CoG[0] * F_g, 0;
    }
    else {
        m = 0;
    }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceTrajectory,
                       controller_interface::ControllerBase)
