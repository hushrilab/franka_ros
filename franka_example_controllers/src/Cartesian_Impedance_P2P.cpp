
#include <franka_example_controllers/Cartesian_Impedance_P2P.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka/gripper_state.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <ros/node_handle.h>
#include <thread>

namespace franka_example_controllers {

bool CartesianImpedanceP2P::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

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
        model_handle = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
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
        state_handle = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
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
    ddx.setZero();
    error.setZero();
    derror.setZero();
    dderror.setZero();
    
    //   For Impedance Controller
    K_p.diagonal() << 700, 700, 700,  40,  60,  15;
    K_d.diagonal() <<  40,  40,  40, 0.5, 1.0, 0.2;
    
    C_hat.setZero();
    
    // Nullspace stiffness and damping
    K_N.setIdentity();
    D_N.setIdentity();
    K_N << K_N * 25;
    D_N << D_N * 0.5 * sqrt(K_N(0,0));  
    I.setIdentity();
    
    notFirstRun = false;
    
    // define time of quintic trajectory
    s   =    0;
    ds  =    0;
    dds =    0;
    
    return true;
}

void CartesianImpedanceP2P::starting(const ros::Time& /*time*/) {

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
    jacobian_prev        <<  jacobian;
    djacobian.setZero();
}

  ///////////////////////////// Gripper /////////////////////////////////
 
  actionlib::SimpleActionClient<franka_gripper::MoveAction>   move( "franka_gripper/move", true);
  actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp("franka_gripper/grasp", true);
  actionlib::SimpleActionClient<franka_gripper::StopAction>   stop( "franka_gripper/stop", true);
  actionlib::SimpleActionClient<franka_gripper::HomingAction> home( "franka_gripper/home", true);
    
void CartesianImpedanceP2P::update(const ros::Time& /*time*/, const ros::Duration& period) {
    
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

//////////////////////////////////////////////   POINT to POINT MOVEMENT  /////////////////////////////////////////
    
    if(waypoint == 1) {
        position_d_target << 0.4, 0, 0.5;
        angles_d_target   <<   0,    20,   0;
        P2PMovement(position_d_target, angles_d_target, position_init, mytime, 5);
        // home Gripper
        GripperMove(0.01, 0.06);
        // home.sendGoal(franka_gripper::HomingGoal());
    }
    else if(waypoint == 2) {
        position_d_target << 0.4, 0, 0.1;
        angles_d_target   <<   0,   0,   0;
        P2PMovement(position_d_target, angles_d_target, position_init, mytime, 5);
        // open Gripper
        GripperMove(0.06, 0.03);
    }  
    else if(waypoint == 3) { // Grasp object here
        position_d_target << 0.4, 0, 0.1;
        angles_d_target   <<   0,  0,   0;
        P2PMovement(position_d_target, angles_d_target, position_init, mytime, 4);
        // Grasp
        if(GraspOnlyOnce) {
	    stop.sendGoal(franka_gripper::StopGoal());
            GripperGrasp(0.035, 0.03, 30, 0.05);
            std::cout << "Executed" <<std::endl;
            GraspOnlyOnce = false;
        }
    } 
    else if(waypoint == 4) { // Hold object while moving
        position_d_target << 0.4, 0, 0.5;
        angles_d_target   <<   0,  0,   0;
        P2PMovement(position_d_target, angles_d_target, position_init, mytime, 5);
    } 
    else if(waypoint == 5) { // Drop object
        position_d_target << 0.4, 0, 0.5;
        angles_d_target   <<   0,  0,   0;
        P2PMovement(position_d_target, angles_d_target, position_init, mytime, 2);
        // open Gripper
        GripperMove(0.05, 0.05);
    }
    else if(waypoint == 6) { // Repeat motion
	stop.sendGoal(franka_gripper::StopGoal());
        waypoint = 1;
        GraspOnlyOnce = true;
    } 
/*        
     else if(waypoint == 2) { // to get steady pose at final waypoint
         position_d     << position_d_target;
         velocity_d.setZero();
         acceleration_d.setZero();
         
         orientation_d_target =    Eigen::AngleAxisd(angles_d_target(0) * M_PI/180 +   M_PI, Eigen::Vector3d::UnitX())
                                 * Eigen::AngleAxisd(angles_d_target(1) * M_PI/180         , Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(angles_d_target(2) * M_PI/180         , Eigen::Vector3d::UnitZ()); 
 
         orientation_d  = orientation_d.slerp(0.01, orientation_d_target);
     }
*/
    else {
        position_d     << curr_position;
        velocity_d.setZero();
        acceleration_d.setZero();
        orientation_d  = curr_orientation;
        ROS_ERROR_STREAM("CartesianImpedanceP2P: No waypoint defined!");
    }

    
///////////////////////////////////// COMPUTE ERRORS //////////////////////////////////////////////////////
    
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
    
    if (notFirstRun) {
        C_hat      << 0.5 * (Lambda - Lambda_prev) / period.toSec();
    } 
    
    F_tau          << Lambda * ddx - K_d * derror - K_p * error - C_hat * derror - Lambda * djacobian_filtered * dq;
    tau_task       << jacobian.transpose() * F_tau;
    
//     nullspace control
    N              << I - jacobian.transpose() * Lambda * jacobian * mass_inv;
    tau_nullspace  << N * (-K_N * (q - q_nullspace) - D_N * dq);
    
//     Desired torque
    tau_d          << tau_task + coriolis + tau_nullspace;
    q_nullspace    << q;
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

  //  std::cout << "Position Error in [mm]:" <<std::endl<< error.head(3) * 1000 <<std::endl; 
  //  std::cout << "ORIENTATION Error in [deg]:" <<std::endl<< error_angles * 180/M_PI<<std::endl;
}

void CartesianImpedanceP2P::P2PMovement(const Eigen::Vector3d& target_position, const Eigen::Vector3d& target_angles, const Eigen::Vector3d& position_start, double time, double T){
    
    a3  =   10 / pow(T, 3);
    a4  = - 15 / pow(T, 4);
    a5  =    6 / pow(T, 5);
    
    orientation_d_target =    Eigen::AngleAxisd(target_angles(0) * M_PI/180 +   M_PI, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(target_angles(1) * M_PI/180         , Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(target_angles(2) * M_PI/180         , Eigen::Vector3d::UnitZ()); 
    omega_d.setZero();
    domega_d.setZero();
    
    if (s <= 1) {
        s =       a3 * pow(time, 3) +      a4 * pow(time, 4) +      a5 * pow(time, 5);
        ds =  3 * a3 * pow(time, 2) +  4 * a4 * pow(time, 3) +  5 * a5 * pow(time, 4);
        dds = 6 * a3 *         time + 12 * a4 * pow(time, 2) + 20 * a5 * pow(time, 3); 
        
        // Point to Point movements
        position_d     <<  position_start + s * (target_position - position_start);
        velocity_d     <<                  ds * (target_position - position_start);
        acceleration_d <<                 dds * (target_position - position_start);  
        orientation_d  =  orientation_d.slerp(10/T * s/1000, orientation_d_target);
    }
    else {
        waypoint++;
        mytime = 0;
        s = 0;
        ds = 0;
        dds = 0;
        position_init << position_d;
    }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceP2P::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
    
    for (size_t i = 0; i < 7; i++) {
        double difference  = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}
  
void CartesianImpedanceP2P::Filter(double filter_param, int rows, int cols, const Eigen::MatrixXd& input, 
    const Eigen::MatrixXd& y_prev,   Eigen::MatrixXd& y) {
        
    y.resize(rows,cols);
    y << (1 - filter_param) * y_prev + filter_param * input;
}

void CartesianImpedanceP2P::GripperMove(double width, double speed) {
    
    franka_gripper::MoveGoal goal;
    goal.width = width;
    goal.speed = speed;
    move.sendGoal(goal);
}

void CartesianImpedanceP2P::GripperGrasp(double width, double speed, int force, double epsilon) {
    
    franka_gripper::GraspGoal goal2;
    goal2.width = width;
    goal2.speed = speed;
    goal2.force = force;
    goal2.epsilon.inner = epsilon;
    goal2.epsilon.outer = epsilon;
    grasp.sendGoal(goal2);
}

void CartesianImpedanceP2P::GripperHome() {
    
//     std::cout<<*home.getResult()<<std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceP2P,
                       controller_interface::ControllerBase)
