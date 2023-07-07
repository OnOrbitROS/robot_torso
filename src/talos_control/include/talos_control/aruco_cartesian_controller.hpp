#ifndef ARUCO_CARTESIAN_CONTROLLER_HPP_INCLUDED
#define ARUCO_CARTESIAN_CONTROLLER_HPP_INCLUDED

// PROJECT
#include <talos_control/aruco_cartesian_controller.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>

namespace controller_ns{

bool aruco_cartesian_controller_class::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh){

    std::string robot_description;
    std::string base_link;
    std::string end_effector_link;
    urdf::Model robot_model;

    ROS_INFO("Initializing Aruco Cartesian Controller ...");

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
    if (!nh.getParam("kp_value",kp_)){
    
        ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/kp_value" << " from parameter server");
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
    jnt_effort_.resize(kdl_chain.getNrOfJoints());
    jacobian_.resize(kdl_chain.getNrOfJoints());

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

    aruco_subscr_ = nh.subscribe("/aruco_single/pose", 3, &aruco_cartesian_controller_class::transformationCallback, this);
    tolerance_publisher_ = nh.advertise<std_msgs::Bool>("/goal_tolerance", 1000);

    return true;

}

void aruco_cartesian_controller_class::update(const ros::Time &time, const ros::Duration &period){

    // Reinitialize solvers
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain));

    //Get initial joints position
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_pos_(i) = joint_handles_[i].getPosition();
    }

    KDL::Frame current_pose;
    fk_status = jnt_to_pose_solver_->JntToCart(jnt_pos_,current_pose);
    if (fk_status == -1) 
        ROS_ERROR_STREAM("No se ha podido calcular la cinematica directa ... ");

    // calculate_transformations(current_pose);
// -------------------------------------------------------------------------------------------    
    // Descomentar para imprimir por pantalla la posición del extremo
    /* ROS_INFO("Current: ");
    ROS_INFO("x : %f",current_pose.p.x());
    ROS_INFO("y : %f",current_pose.p.y());
    ROS_INFO("z : %f",current_pose.p.z());
    ROS_INFO("Target: ");
    ROS_INFO("x : %f",target_frame_.p.x());
    ROS_INFO("y : %f",target_frame_.p.y());
    ROS_INFO("z : %f",target_frame_.p.z()); */
// ------------------------------------------------------------------------------------------- 
    
    // get the pose error
    KDL::Twist error;
    // Target frame is already with ISS reference
    error = KDL::diff(current_pose, target_frame_);
    //check if the tool has arrive to the desired point
    goal_reached.data = compareTolerance(error);

    jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

    // jnt_effort_ = Jac^transpose * cart_wrench
    for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
    {
        jnt_effort_(i) = 0;
        for (unsigned int j=0; j<6; j++){
            jnt_effort_(i) += (jacobian_(j,i) * kp_ * error(j));
        }
    }

    writeJointCommand(jnt_effort_);
    tolerance_publisher_.publish(goal_reached);

}

void aruco_cartesian_controller_class::writeJointCommand(KDL::JntArray joint_command){
    // Send joint position to joints
    for(size_t i = 0; i < joint_handles_.size(); i++){
        joint_handles_[i].setCommand(joint_command(i));
    }
}

void aruco_cartesian_controller_class::starting(const ros::Time &time) {
    
    //Some initializtions
    goal_reached.data = false;
    diff_frame_ = false;
    // Transformation from the aruco marker to the target point
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(1.517, 0.218, 3.104), KDL::Vector(-0.440, 0.328, -0.081)); Funciona
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(1.5443, 0.2231, -3.057), KDL::Vector(-0.491, 0.310, -0.090)); // mejor
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(1.555, 0.1558, -3.008), KDL::Vector(-0.491, 0.310, -0.090)); // HEY-5 Point
    // aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(1.498, 0.254, 3.101), KDL::Vector(-0.441, 0.301, -0.135)); // GRIPPERS Point
    aruco_2_target_ = KDL::Frame(KDL::Rotation::RPY(0.0, 0.05, -1.5), KDL::Vector(-0.161, -0.50, 0.3)); // Pìnza

    //Get initial joints position 
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
        jnt_pos_(i) = joint_handles_[i].getPosition();
    }

    KDL::Frame current_pose;
    fk_status = jnt_to_pose_solver_->JntToCart(jnt_pos_,current_pose);
    if (fk_status == -1) 
        ROS_ERROR_STREAM("No se ha podido calcular la cinematica directa ... ");

    // Send current pose to the end effector to not to move
    target_frame_ = current_pose;
    // Save first cartesian positión to detect when a differente one has arrived
    local_frame_ = current_pose;
}

void aruco_cartesian_controller_class::stopping(const ros::Time &time) {}

bool aruco_cartesian_controller_class::compareTolerance(KDL::Twist error){

    // Point reached

    

    if(diff_frame_){ // To not check when it has just started
        if(fabs(error(0)) < tolerance_ and fabs(error(1)) < tolerance_ and fabs(error(2)) < tolerance_ and fabs(error(3)) < tolerance_ and fabs(error(4)) < tolerance_ and fabs(error(5)) < tolerance_){
            return true;
        }
    }
    // Not reached yet
    return false;
}

bool aruco_cartesian_controller_class::diffTargetFrame(KDL::Frame target_frame){

    if(target_frame != local_frame_){
            return true; // A different frame has arrived
    }
    return false;
}

void aruco_cartesian_controller_class::transformationCallback(const geometry_msgs::PoseStamped& data){
    
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
        target_frame_ = talos_2_aruco_ * aruco_2_target_;

        if (diffTargetFrame(target_frame_)) {
            diff_frame_ = true;
        }
    }

} 
    
}; // namespace


#endif