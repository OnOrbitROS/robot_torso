#ifndef CARTESIAN_CONTROLLER_BASIC_HPP_INCLUDED
#define CARTESIAN_CONTROLLER_BASIC_HPP_INCLUDED

//Project
#include <talos_control/cartesian_controller_basic.h>
#include <talos_control/plot_msg.h>
#include <talos_control/plot_jnt.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>

namespace controller_ns{

bool cartesian_controller_class::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh){

    std::string robot_description;
    std::string base_link;
    std::string end_effector_link;
    urdf::Model robot_model;

    // Get specific configuration of the robot -------------------------------
    if (!nh.getParam("/robot_description",robot_description)){

        ROS_ERROR("Failed to load '/robot_description' from parameter server");
        return false;
    }
    if (!nh.getParam("base_link",base_link)){
        
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/base_link" << " from parameter server");
        return false;
    }
    if (!nh.getParam("end_effector_link",end_effector_link)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
        return false;
    }
    if (!nh.getParam("goal_tolerance",tolerance_)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/goal_tolerance" << " from parameter server");
        return false;
    }
    if (!nh.getParam("kp_value",kp_value_)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/kp_value" << " from parameter server");
        return false;
    }
    if (!nh.getParam("kv_value",kv_value_)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/kv_value" << " from parameter server");
        return false;
    }
    // Robot configuration done ----------------------------------------------


    // Build the kinematic chain ---------------------------------------------
    if (!robot_model.initString(robot_description))
    {

        ROS_ERROR("Failed to parse urdf model from 'robot_description'");
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree_))
    {

        const std::string error = ""
        "Failed to parse KDL tree from urdf model";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    if (!robot_tree_.getChain(base_link,end_effector_link,robot_chain_)){

        const std::string error = ""
        "Failed to parse robot chain from urdf model. "
        "Are you sure that both your 'base_link' and 'end_effector_link' exist?";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    // Chain built ----------------------------------------------------------------

    // Get Joints specific names and its handles ----------------------------------
    if (!nh.getParam("joints",joint_names_)){
        const std::string error = ""
        "Failed to load " + nh.getNamespace() + "/joints" + " from parameter server";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        joint_handles_.push_back(hw->getHandle(joint_names_[i]));
    }
    // Joints Configured -----------------------------------------------------------

    // Initialize solvers ----------------------------------------------------------
    // To remove two joints of the torso but the base_link is still in the 0,0,0
    for(size_t i = 0; i < robot_chain_.getNrOfSegments(); i++){
        if(i != 1 && i != 2) kdl_chain.addSegment(robot_chain_.getSegment(i));
    }

    // kdl_chain.addChain(robot_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain));
    // -----------------------------------------------------------------------------

    // Resizes the joint state vectors in non-realtime -----------------------------
    jnt_pos_.resize(kdl_chain.getNrOfJoints());
    jnt_vel_.resize(kdl_chain.getNrOfJoints());
    jnt_effort_.resize(kdl_chain.getNrOfJoints());
    jacobian_.resize(kdl_chain.getNrOfJoints());

    // Set the desired pose --------------------------------------------------------
    // El punto KDL::Rotation::RPY(0,0,3.14), KDL::Vector(0.5, -0.2, 0.2) funciona
    // El punto KDL::Rotation::RPY(0,0,3.14), KDL::Vector(0.5, -0.2, -0.2) funciona
    // target_frame_ = KDL::Frame(KDL::Rotation::RPY(0,0,3.14), KDL::Vector(0.4, -0.1, 0.2));
    // -----------------------------------------------------------------------------

    // ROS_INFO("Cadena cinematica incializada correctamente ...");
    // ROS_INFO("Handles Size: %i", joint_handles_.size());
    // ROS_INFO("Names Size: %i", joint_names_.size());
    // ROS_INFO("Number of Segments: %i", kdl_chain.getNrOfSegments());
    // ROS_INFO("Number of Joints: %i", kdl_chain.getNrOfJoints());

    // ROS_INFO("Number of Segments: %i", kdl_chain.getNrOfSegments());
    // ROS_INFO("Number of Joints: %i", kdl_chain.getNrOfJoints());
    // ROS_INFO("Cadena cinemática creada: ");
    // for(int i = 0; i < kdl_chain.getNrOfSegments(); i++){
    //     std::cout << "Segment : " << i << " --> " << kdl_chain.getSegment(i).getName() << std::endl;
    //     std::cout << "Segments' Joint name : " << kdl_chain.getSegment(i).getJoint().getName() << std::endl;
    //     std::cout << "Segments' Joint Type : " << kdl_chain.getSegment(i).getJoint().getTypeName() << std::endl;
    //     std::cout << " ------------------------- " << std::endl;
    // }

    target_frame_subscr_ = nh.subscribe("/cartesian_target_frame", 3, &cartesian_controller_class::targetFrameCallback, this);
    tolerance_publisher_ = nh.advertise<std_msgs::Bool>("/goal_tolerance", 1000);
    control_error_pub_ = nh.advertise<talos_control::plot_msg>("/control_error", 1000);
    velocity_error_pub_ = nh.advertise<talos_control::plot_msg>("/velocity_error", 1000);
    joint_value_pub_ = nh.advertise<talos_control::plot_jnt>("/joint_value", 1000);
    final_error_pub_ = nh.advertise<talos_control::plot_msg>("/final_error", 1000);
    frames_pub_ = nh.advertise<talos_control::plot_msg>("/frames_trajectory", 1000);

    return true;

}

void cartesian_controller_class::update(const ros::Time &time, const ros::Duration &period){
    
    // Reinitialize solvers
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain));

    //Get initial joints position
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_pos_(i) = joint_handles_[i].getPosition();
    }
        
    //Get initial joints velocity
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_vel_(i) = joint_handles_[i].getVelocity();
    }

    KDL::Frame current_pose;
    fk_status = jnt_to_pose_solver_->JntToCart(jnt_pos_,current_pose);
    if (fk_status < 0) 
        ROS_ERROR_STREAM("Forward Kinematics could not be calculated ... ");

    KDL::Twist desired_vel = KDL::Twist::Zero(); // Initiliz it to zeros;

    if(start_trajectory_){
        //Get the actual time
        actual_time_ = ros::Time::now() - begin_time_; // That's 0 sec
        double now = actual_time_.toSec();
        target_frame_ = trajectory_->Pos(now); 
        desired_vel = trajectory_->Vel(now); 
        if(now > final_time_) finish_trajectory_ = true;
    }
     
    // get the actual error
    KDL::Twist control_error;
    // get the final error
    KDL::Twist final_error;

    // Target frame is already with ISS reference
    control_error = KDL::diff(current_pose, target_frame_);

    final_error = KDL::diff(current_pose, final_frame_);
    //check if the tool has arrive to the desired point
    goal_reached.data = compareTolerance(final_error);

    jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

    // get actual cartesian velocities
    KDL::Twist cart_vel = KDL::Twist::Zero(); // Initiliz it to zeros
    for(unsigned int i = 0; i < 6 ; i++){
        for (unsigned int j=0; j<jnt_vel_.rows(); j++){
            cart_vel(i) += jacobian_(i,j) * jnt_vel_(j);
        }
    }

    KDL::Twist velocity_error;
    velocity_error = KDL::diff(cart_vel, desired_vel);

    // jnt_effort_ = Jac^transpose * cart_wrench
    for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
    {
        jnt_effort_(i) = 0;
        for (unsigned int j=0; j<6; j++){
            jnt_effort_(i) += (jacobian_(j,i) * (kp_ * control_error(j) + kv_*velocity_error(j)));
        }
    }
    
    writeJointCommand(jnt_effort_);

    // WRITE MSGS _________________________________
    talos_control::plot_msg plot_vel_error;
    plot_vel_error.x_err = velocity_error(0);
    plot_vel_error.y_err = velocity_error(1);
    plot_vel_error.z_err = velocity_error(2);
    plot_vel_error.roll_err = velocity_error(3);
    plot_vel_error.pitch_err = velocity_error(4);
    plot_vel_error.yaw_err = velocity_error(5);
    //_____________________________________________
    talos_control::plot_msg plot_error;
    plot_error.x_err = control_error(0);
    plot_error.y_err = control_error(1);
    plot_error.z_err = control_error(2);
    plot_error.roll_err = control_error(3);
    plot_error.pitch_err = control_error(4);
    plot_error.yaw_err = control_error(5);
    //_____________________________________________
    talos_control::plot_msg plot_fin_error;
    plot_fin_error.x_err = final_error(0);
    plot_fin_error.y_err = final_error(1);
    plot_fin_error.z_err = final_error(2);
    plot_fin_error.roll_err = final_error(3);
    plot_fin_error.pitch_err = final_error(4);
    plot_fin_error.yaw_err = final_error(5);
    //_____________________________________________
    talos_control::plot_jnt joint_value;
    joint_value.jnt_0 = jnt_effort_(0);
    joint_value.jnt_1 = jnt_effort_(1);
    joint_value.jnt_2 = jnt_effort_(2);
    joint_value.jnt_3 = jnt_effort_(3);
    joint_value.jnt_4 = jnt_effort_(4);
    joint_value.jnt_5 = jnt_effort_(5);
    joint_value.jnt_6 = jnt_effort_(6);
    //_____________________________________________
    talos_control::plot_msg frames_traj;
    frames_traj.x_err = target_frame_.p.x();
    frames_traj.y_err = target_frame_.p.y();
    frames_traj.z_err = target_frame_.p.z();
    frames_traj.roll_err = current_pose.p.x();
    frames_traj.pitch_err = current_pose.p.y();
    frames_traj.yaw_err = current_pose.p.z();
    //_____________________________________________
    // PUBLISH ____________________________________
    tolerance_publisher_.publish(goal_reached);
    control_error_pub_.publish(plot_error);
    final_error_pub_.publish(plot_fin_error);
    velocity_error_pub_.publish(plot_vel_error);
    joint_value_pub_.publish(joint_value);
    frames_pub_.publish(frames_traj);
    // ____________________________________________


    // If it arrives to the desired point it has to stop there
    if((goal_reached.data and start_trajectory_) or finish_trajectory_){ 
        // target_frame_ = current_pose;
        start_frame_ = current_pose;
        start_trajectory_ = false;
    }

}

void cartesian_controller_class::writeJointCommand(KDL::JntArray joint_command){

    for(size_t i = 0; i < joint_handles_.size(); i++){
        joint_handles_[i].setCommand(joint_command(i));
    }
    
}

void cartesian_controller_class::starting(const ros::Time &time) {
    
    //Some initializtions
    kp_ = kp_value_; // 750
    kv_ = kv_value_; // 5 
    goal_reached.data = false;
    diff_frame_ = true;
    start_trajectory_ = false;
    finish_trajectory_ = false;

    //Get initial joints position
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_pos_(i) = joint_handles_[i].getPosition();
    }

    KDL::Frame current_pose;
    fk_status = jnt_to_pose_solver_->JntToCart(jnt_pos_,current_pose);
    if (fk_status == -1) 
        ROS_ERROR_STREAM("No se ha podido calcular la cinematica directa ... ");

    // Send current pose to the effecto to not to move
    target_frame_ = current_pose;
    // Save first cartesian positión to detect when a differente one has arrived
    local_frame_ = current_pose;
    // Save inital pose to create the trajectory
    start_frame_ = current_pose;
    // Save final frame as objective
    final_frame_ = current_pose;

}

void cartesian_controller_class::stopping(const ros::Time &time) {}

bool cartesian_controller_class::compareTolerance(KDL::Twist error){

    // Point reached
    if(not diff_frame_){ // To not check when it has just started
        if(fabs(error(0)) < tolerance_ and fabs(error(1)) < tolerance_ and fabs(error(2)) < tolerance_){
            return true;
        }
    }
    // Not reached yet
    return false;

}

bool cartesian_controller_class::diffTargetFrame(const talos_control::target_frame& target_frame){

    double roll, pitch, yaw;
    local_frame_.M.GetRPY(roll, pitch, yaw);
    double x = local_frame_.p.x();
    double y = local_frame_.p.y();
    double z = local_frame_.p.z();

    double x_diff = fabs(target_frame.x - x);
    double y_diff = fabs(target_frame.y - y);
    double z_diff = fabs(target_frame.z - z);
    double roll_diff = fabs(target_frame.roll - roll);
    double pitch_diff = fabs(target_frame.pitch - pitch);
    double yaw_diff = fabs(target_frame.yaw - yaw);
    double threshold = 0.0001;

    // If no position has changed more than 0.0001 it is the same frame
    if((x_diff < threshold) and (y_diff < threshold) and (z_diff < threshold) and
        (roll_diff < threshold) and (pitch_diff < threshold) and (yaw_diff < threshold)){
            return false; // Almost the same frame has arrived
    }

    return true;

}

void cartesian_controller_class::targetFrameCallback(const talos_control::target_frame& target_frame){

    if(diffTargetFrame(target_frame)){
        // The desired point is at ISS reference
        float x = target_frame.x;
        float y = target_frame.y;
        float z = target_frame.z;
        double roll, pitch, yaw;
        roll = target_frame.roll;
        pitch = target_frame.pitch;
        yaw = target_frame.yaw;
        // Set duration for the desired trajectory
        final_time_ = ros::Duration(target_frame.duration).toSec();
        
        // Save starting time for the trajectory
        begin_time_ = ros::Time::now();

        // Save final frame for the trajectory
        final_frame_ = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));
        // Create the trajectory with 7.0s duration
        trajectory_ = trajectoryPlanner(start_frame_, final_frame_, final_time_);
        // Update localframe to check if when a different frame has arrived
        local_frame_ = KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));
        // Set diff_frame_ a true to start checking if the end effector has arrived
        diff_frame_ = true;
        // Start the trajectory in the main loop
        start_trajectory_ = true;
    }

    else{
        // I it is the same frame do not update a thing
        diff_frame_ = false;
    }   

}

// Create a cartesian trajectory with a quintic spline interpolation 
KDL::Trajectory* cartesian_controller_class::trajectoryPlanner(const KDL::Frame start, const KDL::Frame ending, double duration){

    // Geometrical path
    KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis()); 
    path->Add(start); 
    path->Add(ending);
    path->Finish();

    KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();
    //(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, Duration)
    velprof->SetProfileDuration(0, 0.0, 0.0, path->PathLength(), 0, 0.0,  duration); 
    KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velprof);

    return traject;

}
    
}; // namespace

#endif