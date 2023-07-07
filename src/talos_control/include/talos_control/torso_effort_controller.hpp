#ifndef TORSO_EFFORT_CONTROLLER_HPP_INCLUDED
#define TORSO_EFFORT_CONTROLLER_HPP_INCLUDED

// PROJECT
#include <talos_control/torso_effort_controller.h>
#include <talos_control/plot_msg.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>

namespace controller_ns{

bool torso_effort_controller_class::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh){

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
    if (!nh.getParam("trajectory_duration",time_param_)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/trajectory_duration" << " from parameter server");
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

    aruco_subscr_ = nh.subscribe("/aruco_single/pose", 10000, &torso_effort_controller_class::transformationCallback, this);
    tolerance_publisher_ = nh.advertise<std_msgs::Bool>("/goal_tolerance", 1000);
    control_error_pub_ = nh.advertise<talos_control::plot_msg>("/control_error", 1000);
    velocity_error_pub_ = nh.advertise<talos_control::plot_msg>("/velocity_error", 1000);
    joint_value_pub_ = nh.advertise<talos_control::plot_jnt>("/joint_value", 1000);
    final_error_pub_ = nh.advertise<talos_control::plot_msg>("/final_error", 1000);
    frames_pub_ = nh.advertise<talos_control::plot_msg>("/frames_trajectory", 1000);
    quintic_pub_ = nh.advertise<talos_control::plot_msg>("/quintic_profile", 1000);

    return true;
}

void torso_effort_controller_class::update(const ros::Time &time, const ros::Duration &period){

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
        ROS_ERROR_STREAM("Forward kinematics failed ... ");

    KDL::Twist desired_vel = KDL::Twist::Zero(); // Initiliz it to zeros
    KDL::Twist desired_acc = KDL::Twist::Zero(); // Initiliz it to zeros

    if(start_trajectory_){
        // Get the actual time
        actual_time_ = ros::Time::now() - begin_time_; // That's start in sec 0
        now_ = actual_time_.toSec();
        target_frame_ = trajectory_->Pos(now_);
        desired_vel = trajectory_->Vel(now_);
        desired_acc = trajectory_->Acc(now_);

        vel_i_ = global_velPof_->Vel(now_); // Save actual velocity to start next trajectory
        acc_i_ = global_velPof_->Acc(now_); // Save actual accelertion to start next trajectory
        start_frame_ = current_pose; // Update de start frame to start the folowing trajectory
    }

    // Get the each pose control error
    KDL::Twist control_error, final_error;
    // Target frame is the next nearest pose
    control_error = KDL::diff(current_pose, target_frame_);
    // Calculate the error of the final pose in the trajectory
    final_error = KDL::diff(current_pose, final_frame_);
    // Check if the tool has arrive to the desired point
    goal_reached.data = compareTolerance(final_error);
    // Get jacobian matrix 
    jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

    // Get actual cartesian velocities
    KDL::Twist cart_vel = KDL::Twist::Zero(); // Initilize it to zeros
    for(unsigned int i = 0; i < 6 ; i++){
        for (unsigned int j=0; j<jnt_vel_.rows(); j++){
            cart_vel(i) += jacobian_(i,j) * jnt_vel_(j);
        }
    }

    KDL::Twist velocity_error;
    velocity_error = KDL::diff(cart_vel, desired_vel);

    // Joint efforts in each iteration _______________
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
    talos_control::plot_msg quintic_prof;
    quintic_prof.x_err = acc_i_;
    quintic_prof.y_err = vel_i_;
    quintic_prof.z_err = duration_time_;
    //_____________________________________________
    // PUBLISH ____________________________________
    tolerance_publisher_.publish(goal_reached);
    control_error_pub_.publish(plot_error);
    final_error_pub_.publish(plot_fin_error);
    velocity_error_pub_.publish(plot_vel_error);
    joint_value_pub_.publish(joint_value);
    frames_pub_.publish(frames_traj);
    quintic_pub_.publish(quintic_prof);
    // ____________________________________________

    // If it arrives to the desired point it has to stop there
    if(goal_reached.data and start_trajectory_){
        target_frame_ = current_pose;
        start_frame_ = current_pose;
        start_trajectory_ = false;
        finish_trajectory_ = true;
    }

}

void torso_effort_controller_class::writeJointCommand(std::vector<float> joint_command){
    // Send joint efforts to all joints
    for(size_t i = 0; i < joint_handles_.size(); i++){
        joint_handles_[i].setCommand(joint_command[i]);
    }
}

void torso_effort_controller_class::starting(const ros::Time &time) {
    
    // Some initializtions ____________________________________________________________________________________________________________________________
    kp_ = kv_value_;
    kv_ = kv_value_;
    // ________________________________________________________________________________________________________________________________________________
    diff_frame_ = false;
    goal_reached.data = false;
    // Transformation from the aruco marker to the target point _______________________________________________________________________________________
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(1.555, 0.1558, -3.008), KDL::Vector(-0.491, 0.310, -0.090)); // HEY-5 Point
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(-0.116, 1.514, 1.444), KDL::Vector(-0.441, 0.301, -0.135)); // GRIPPERS Point
    aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(0.0, 0.05, -1.5), KDL::Vector(-0.161, -0.50, 0.3)); // Pìnza
    // Trjectory inicializations_______________________________________________________________________________________________________________________
    now_ = 0.0;
    vel_i_ = 0.0;
    acc_i_ = 0.0;
    duration_time_ = 0.0; 
    compute_efforts_= false;
    start_trajectory_ = false;
    finish_trajectory_ = false;
    take_start_distance_ = true;
    ref_time_ = ros::Duration(time_param_).toSec(); //9 sec trajectory
    global_velPof_ = new KDL::VelocityProfile_Spline();
    // ________________________________________________________________________________________________________________________________________________
    
    //Get initial joints position _____________________________________________________________________________________________________________________
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_pos_(i) = joint_handles_[i].getPosition();
    }

    KDL::Frame current_pose;
    fk_status = jnt_to_pose_solver_->JntToCart(jnt_pos_,current_pose);
    if (fk_status == -1) 
        ROS_ERROR_STREAM("Forward kinematics failed ... ");

    // Send current pose to the end effector to not to move ___________________________________________________________________________________________
    target_frame_ = current_pose;
    // ________________________________________________________________________________________________________________________________________________
    // Save the first cartesian positión to detect when a differente one has arrived __________________________________________________________________
    local_frame_ = current_pose;
    start_frame_ = current_pose;
    // ________________________________________________________________________________________________________________________________________________
    
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
    auto q = pinocchio::neutral(model_complete);

    // CREATE REDUCED MODEL ___________________________________________________________________________________________________________________________
    pinocchio::buildReducedModel(model_complete, articulaciones_bloqueadas_id, q, model);
    std::cout << "Reduced model complet!" << std::endl;
    //_________________________________________________________________________________________________________________________________________________

}

void torso_effort_controller_class::stopping(const ros::Time &time) {}

bool torso_effort_controller_class::compareTolerance(KDL::Twist error){
    
    if(diff_frame_){ // To not check when it has just started
        if(fabs(error(0)) < tolerance_ and fabs(error(1)) < tolerance_ and fabs(error(2)) < tolerance_){
            return true; // Point reached
        }
    }
    // Not reached yet
    return false;

}

bool torso_effort_controller_class::diffTargetFrame(KDL::Frame target_frame){

    if(target_frame != local_frame_){
            return true; // A different frame has arrived!!
    }
    return false;
}

void torso_effort_controller_class::transformationCallback(const geometry_msgs::PoseStamped& data){
    
    float x, y, z, q_x, q_y, q_z, q_w;

    x = data.pose.position.x;
    y = data.pose.position.y;
    z = data.pose.position.z;
    q_x = data.pose.orientation.x;
    q_y = data.pose.orientation.y;
    q_z = data.pose.orientation.z;
    q_w = data.pose.orientation.w;

    if(data.header.frame_id == "base_link"){

        talos_2_aruco_ = KDL::Frame(KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w), KDL::Vector(x, y, z));
        KDL::Frame temp_frame = talos_2_aruco_ * aruco_2_target_; // Transformation from base link to position using aruco reference.

        if (diffTargetFrame(temp_frame) and not finish_trajectory_) {
            diff_frame_ = true;
            final_frame_ = temp_frame;

            if(take_start_distance_){
                start_distance_ = distanceBetweenFrames(start_frame_, final_frame_);
            }

            double frame_distance = distanceBetweenFrames(start_frame_, final_frame_);
            // The time is escalated having the reference of the first distance
            duration_time_ = (frame_distance * ref_time_)/start_distance_;

            // Save starting time for the trajectory
            begin_time_ = ros::Time::now();

            // If a direfent frame is recalculated it won't do a 9 sec trajectory
            // It use the velocity and acceleration of the previous trajectory to start the next one
            trajectory_ = trajectoryPlanner(start_frame_, final_frame_,vel_i_, acc_i_, duration_time_);
            local_frame_ = temp_frame;
            start_trajectory_ = true;
            compute_efforts_ = true;
            take_start_distance_ = false;
        }
    }

}

// Create a cartesian trajectory with a quintic spline interpolation 
KDL::Trajectory* torso_effort_controller_class::trajectoryPlanner(const KDL::Frame start, const KDL::Frame ending, double vel_i, double acc_i, double duration){

    // Geometrical path
    KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis()); 
    path->Add(start); 
    path->Add(ending);
    path->Finish();

    KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();
    //(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, Duration)
    velprof->SetProfileDuration(0, vel_i, acc_i, path->PathLength(), 0, 0.0,  duration);
    global_velPof_->SetProfileDuration(0, vel_i, acc_i, path->PathLength(), 0, 0.0,  duration); // To get the velocity and accelration of the previous trajectory
    KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velprof);

    return traject;

}

double torso_effort_controller_class::distanceBetweenFrames(const KDL::Frame start, const KDL::Frame ending){

    double angle_distance;
    double module_distance;
    double distance;

    KDL::Frame R = ending.Inverse()*start;
    angle_distance = acos((trace(R)-1)/2); // Distance beetwen start matrix rotation and end matrix rotation

    double x_s = start.p.x();
    double y_s = start.p.y();
    double z_s = start.p.z();
    double x_e = ending.p.x();
    double y_e = ending.p.y();
    double z_e = ending.p.z();
    
    // Distance beetwen start position and ending position
    module_distance = sqrt(pow(x_s - x_e, 2) + pow(y_s - y_e, 2) + pow(z_s - z_e, 2)); 

    // Calculate the mean of this two
    distance = (angle_distance + module_distance)/2;

    return distance;

}

// Obtain the trace of the matrix rotation instade a KDL::Frame
double torso_effort_controller_class::trace(const KDL::Frame frame){

    double trace;
    
    trace = frame(0,0) + frame(1,1) + frame(2,2);
    
    return trace;

}

// Method for calculating the pseudo-Inverse as recommended by Eigen developers (Moore-Penrose Pseudo-Inverse)
template<class _Matrix_Type_>
_Matrix_Type_ torso_effort_controller_class::pseudoInverse(const _Matrix_Type_ &a){

    double epsilon = std::numeric_limits<double>::epsilon();

    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    // For a square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

std::vector<float> torso_effort_controller_class::calculate_jnt_efforts(pinocchio::Model model, KDL::Twist dot_dot_Xd, KDL::Twist dot_X_err, KDL::Twist x_err, KDL::JntArray jnt_vel, 
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