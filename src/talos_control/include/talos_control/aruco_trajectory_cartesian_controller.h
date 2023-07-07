#ifndef ARUCO_TRAJECTORY_CARTESIAN_CONTROLLER_H_INCLUDED
#define ARUCO_TRAJECTORY_CARTESIAN_CONTROLLER_H_INCLUDED

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// STD
#include <vector>
#include <string>
#include <math.h>

// Project
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

//Boost
#include <boost/scoped_ptr.hpp>

//Eigen
#include <Eigen/SVD>

//KDL
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>

// KDL Trajectory
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

// GEOMETRY MSGS
#include <geometry_msgs/PoseStamped.h>

// STD_MSGS
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace controller_ns{

    class aruco_trajectory_cartesian_controller_class : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
        
        public:

            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh);
            void update(const ros::Time &time, const ros::Duration &period);
            void starting(const ros::Time &time);
            void stopping(const ros::Time &time);
            void writeJointCommand(KDL::JntArray joint_command);

        private:
            
            KDL::Trajectory*  trajectoryPlanner(const KDL::Frame start, const KDL::Frame ending, double vel_i, double acc_i, double duration);
            double distanceBetweenFrames(const KDL::Frame start, const KDL::Frame ending);
            double trace(const KDL::Frame frame);
            bool diffTargetFrame(KDL::Frame target_frame);
            bool compareTolerance(KDL::Twist error);

            ros::Subscriber aruco_subscr_;
            void transformationCallback(const geometry_msgs::PoseStamped& data);

            ros::Publisher tolerance_publisher_;
            float tolerance_;
            bool diff_frame_;
            std_msgs::Bool goal_reached;

            std::vector<hardware_interface::JointHandle>      joint_handles_;
            std::vector<std::string>                          joint_names_;

            KDL::Chain robot_chain_;
            KDL::Tree robot_tree_;
            KDL::Chain kdl_chain;

            // KDL Solvers performing the actual computations
            int fk_status;
            boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
            boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

            //Variables
            float kp_, kv_, kp_value_, kv_value_;
            KDL::JntArray jnt_pos_, jnt_effort_, jnt_vel_;
            KDL::Jacobian jacobian_;
            KDL::Frame target_frame_;
            KDL::Frame talos_2_aruco_;
            KDL::Frame local_frame_;
            KDL::Frame aruco_2_target_;
              
            //Trejctories Variables
            KDL::Trajectory* trajectory_;
            KDL::VelocityProfile_Spline* global_velPof_;   
            KDL::Frame start_frame_;
            KDL::Frame final_frame_;
            double vel_i_;
            double acc_i_;
            double start_distance_;
            bool start_trajectory_;
            bool finish_trajectory_;
            bool take_start_distance_;

            //Time variables
            ros::Time begin_time_;
            float time_param_;
            double duration_time_;
            double ref_time_;
            double now_;
            ros::Duration actual_time_;

            //Data publishers
            ros::Publisher control_error_pub_;
            ros::Publisher velocity_error_pub_;
            ros::Publisher joint_value_pub_;
            ros::Publisher final_error_pub_;
            ros::Publisher frames_pub_;
            ros::Publisher quintic_pub_;

    };

}; //namespace

//Project
#include <talos_control/aruco_trajectory_cartesian_controller.hpp>

#endif


