#ifndef CARTESIAN_CONTROLLER_TORSO_EFFORTS_HPP_INCLUDED
#define CARTESIAN_CONTROLLER_TORSO_EFFORTS_HPP_INCLUDED

//Project
#include <talos_control/cartesian_controller_torso_efforts.h>
#include <talos_control/plot_msg.h>
#include <talos_control/plot_jnt.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>

namespace controller_ns{

bool cartesian_controller_torso_efforts_class::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh){

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
    jnt_effort_ = std::vector<float>(7,0);
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

    target_frame_subscr_ = nh.subscribe("/cartesian_target_frame", 3, &cartesian_controller_torso_efforts_class::targetFrameCallback, this);
    tolerance_publisher_ = nh.advertise<std_msgs::Bool>("/goal_tolerance", 1000);
    control_error_pub_ = nh.advertise<talos_control::plot_msg>("/control_error", 1000);
    velocity_error_pub_ = nh.advertise<talos_control::plot_msg>("/velocity_error", 1000);
    joint_value_pub_ = nh.advertise<talos_control::plot_jnt>("/joint_value", 1000);
    final_error_pub_ = nh.advertise<talos_control::plot_msg>("/final_error", 1000);
    frames_pub_ = nh.advertise<talos_control::plot_msg>("/frames_trajectory", 1000);

    return true;

}

void cartesian_controller_torso_efforts_class::update(const ros::Time &time, const ros::Duration &period){
    
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
    if (fk_status == -1) 
        ROS_ERROR_STREAM("No se ha podido calcular la cinematica directa ... ");

    KDL::Twist desired_vel = KDL::Twist::Zero(); // Initiliz it to zeros
    KDL::Twist desired_acc = KDL::Twist::Zero(); // Initiliz it to zeros

    if(start_trajectory_){
        //Get the actual time
        actual_time_ = ros::Time::now() - begin_time_; // That's 0 sec
        double now = actual_time_.toSec();
        if (now < final_time_)
        {
            target_frame_ = trajectory_->Pos(now); 
            desired_vel = trajectory_->Vel(now);
            desired_acc = trajectory_->Acc(now);
        } else 
            finish_trajectory_ = true;
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
    // Joint efforts in each iteration ____________
    if(compute_efforts_){
        jnt_effort_ = calculate_jnt_efforts(model, desired_acc, velocity_error, control_error, jnt_vel_, jnt_pos_, kp_, kv_);
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
    joint_value.jnt_0 = jnt_effort_[0];
    joint_value.jnt_1 = jnt_effort_[1];
    joint_value.jnt_2 = jnt_effort_[2];
    joint_value.jnt_3 = jnt_effort_[3];
    joint_value.jnt_4 = jnt_effort_[4];
    joint_value.jnt_5 = jnt_effort_[5];
    joint_value.jnt_6 = jnt_effort_[6];
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
        start_frame_ = current_pose;
        start_trajectory_ = false;
    }

}

void cartesian_controller_torso_efforts_class::writeJointCommand(std::vector<float> joint_command){
    // Send joint position to joints
    for(size_t i = 0; i < joint_handles_.size(); i++){
        joint_handles_[i].setCommand(joint_command[i]);
    }
}


void cartesian_controller_torso_efforts_class::starting(const ros::Time &time) {
    
    //Some initializtions
    kp_ = kp_value_;
    kv_ = kv_value_; 
    goal_reached.data = false;
    diff_frame_ = true;
    compute_efforts_= false;
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

    //Initialize pinnochio models______________________________________________________________________________________________________________________
    std::string urdf_path = ros::package::getPath("talos_control") + std::string("/src/talos_arms_down.urdf");

    // Load robot model in pinnochio
    std::cout << "Loading file: " << urdf_path << "\n";
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_complete);
    std::cout << "Complete! Robot's name: " << model_complete.name << '\n';

    // list of the joint that will be blocked _________________________________________________________________________________________________________
    std::vector<std::string> articulaciones_bloqueadas{
        "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", 
        "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "hip_waist_joint", 
        "waist_chest_joint", "updown_head_joint", "leftright_head_joint",
    };
    // blocked joints id's ____________________________________________________________________________________________________________________________
    std::vector<pinocchio::JointIndex> articulaciones_bloqueadas_id;
    articulaciones_bloqueadas_id.reserve(articulaciones_bloqueadas.size());
    for (const auto& str : articulaciones_bloqueadas)
        articulaciones_bloqueadas_id.emplace_back(model_complete.getJointId(str));

    // IF YOU WANT THE INMOBILITZED ARM TO HAVE ANOTHER DIFFERENT CONFIGURATION
    // YOU MUST DO IT HERE!!
    auto qc = pinocchio::neutral(model_complete);

    // CREATE REDUCED MODEL ___________________________________________________________________________________________________________________________
    pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_id, qc, model);
    std::cout << "Reduced model complet!" << std::endl;
    //_________________________________________________________________________________________________________________________________________________


}

void cartesian_controller_torso_efforts_class::stopping(const ros::Time &time) {}

bool cartesian_controller_torso_efforts_class::compareTolerance(KDL::Twist error){

    // Point reached
    if(not diff_frame_){ // To not check when it has just started
        if(fabs(error(0)) < tolerance_ and fabs(error(1)) < tolerance_ and fabs(error(2)) < tolerance_){
            return true;
        }
    }
    // Not reached yet
    return false;

}

bool cartesian_controller_torso_efforts_class::diffTargetFrame(const talos_control::target_frame& target_frame){

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

void cartesian_controller_torso_efforts_class::targetFrameCallback(const talos_control::target_frame& target_frame){

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
        compute_efforts_ = true;
    }

    else{
        // I it is the same frame do not update a thing
        diff_frame_ = false;
    }   

}

// Create a cartesian trajectory with a quintic spline interpolation 
KDL::Trajectory* cartesian_controller_torso_efforts_class::trajectoryPlanner(const KDL::Frame start, const KDL::Frame ending, double duration){

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

// Method for calculating the pseudo-Inverse as recommended by Eigen developers (Moore-Penrose Pseudo-Inverse)
template<class _Matrix_Type_>
_Matrix_Type_ cartesian_controller_torso_efforts_class::pseudoInverse(const _Matrix_Type_ &a){

    double epsilon = std::numeric_limits<double>::epsilon();

    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    // For a square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

std::vector<float> cartesian_controller_torso_efforts_class::calculate_jnt_efforts(pinocchio::Model model, KDL::Twist dot_dot_Xd, KDL::Twist dot_X_err, KDL::Twist x_err, KDL::JntArray jnt_vel, 
                                                                        KDL::JntArray jnt_pos, float kp, float kv){
    
    // Initialitations _____________________________________________
    std::vector<float> joint_effort;
    pinocchio::Data data;
    Eigen::VectorXd dot_q(7);
    Eigen::VectorXd acc_d(6);
    Eigen::VectorXd vel_err(6);
    Eigen::VectorXd pos_err(6);
    Eigen::VectorXd q(14);

    for(int i = 0; i < 6;i++){
        acc_d(i) = dot_dot_Xd(i);
        vel_err(i) = dot_X_err(i);
        pos_err(i) = x_err(i);
        q(i) = 0;
    }

    q(6) = 1;
    for(int i = 0; i < jnt_vel.rows(); i++) {
        dot_q(i) = jnt_vel(i);
        q(i+7) = jnt_pos(i);
    }
    // _____________________________________________________________

    // robot's state _______________________________________________
    data = pinocchio::Data(model);

    // Calculate complete Jacobian _________________________________
    pinocchio::computeJointJacobians(model, data, q);
    // std::cout << "2 -----" << std::endl;
    pinocchio::updateFramePlacements(model, data);

    // Jacobian with respect the end effector  ______________________
    Eigen::MatrixXd J(6, model.nv); J.setZero();
    pinocchio::FrameIndex frame_id = model.getFrameId("arm_right_7_link");

    pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    Eigen::MatrixXd Jb = J.block<6, 6>(0,0);
    Eigen::MatrixXd Jm = J.block(0, 6, 6, 7);

    // derivated Jacobian____________________________________________
    auto v = Eigen::VectorXd::Random(model.nv);
    Eigen::MatrixXd dJ(6, model.nv); dJ.setZero();
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);
    pinocchio::getFrameJacobianTimeVariation(model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    auto dJb = dJ.block<6, 6>(0, 0);
    auto dJm = dJ.block(0, 6, 6, 7);

    // Inertia Matrix _______________________________________________
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    // Intertia subMatrix
    auto Hb  = data.M.block<6, 6>(0, 0);
    auto Hbm = data.M.block(0, 6, 6, 7);
    auto Hm  = data.M.block(6, 6, 7, 7);

    // Clculate desired Jacobians ___________________________________
    Eigen::MatrixXd Jg  = Jm - Jb * (Hb.inverse() * Hbm);
    Eigen::MatrixXd dJg = dJm - dJb * (Hb.inverse() * Hbm);

    // Control Ecuation _____________________________________________
    auto Hm_ = Hm - (Hbm.transpose() * Hb.inverse() * Hbm);
    auto Jg_ps =  pseudoInverse(Jg);

    // Control ecuation _____________________________________________
    auto effort = Hm_*Jg_ps*(acc_d + kv*vel_err + kp*pos_err - (dJg*dot_q));

    // Covenrt Eigen to std::vector _________________________________
    for(int i = 0; i < 7; i++){
        joint_effort.push_back(effort(i));
    }

    return joint_effort;   

}
    
}; // namespace

#endif