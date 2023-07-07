#include <ros/ros.h>
#include <iostream>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>
#include "talos_control/hand_operations.h"


ros::Publisher right_pub, left_pub;
std::string right_topic = "/right_gripper_controller/command";
std::string left_topic = "/left_gripper_controller/command";


bool serviceCallback(talos_control::hand_operations::Request &req, talos_control::hand_operations::Response &res){

    trajectory_msgs::JointTrajectory jt;
    trajectory_msgs::JointTrajectoryPoint p;
    std::string joint_name;

    if(req.hand == "right" || req.hand == "left"){
        joint_name = "gripper_" + req.hand + "_joint";
    }

    else{

        const std::string error = "Selected hand is not valid. Valid options are: right or left";
        ROS_ERROR_STREAM(error);
        res.succes = false;
        return false;
    }
    
    if (req.operation == "close"){

        p.positions.push_back(-1.00);
        p.velocities.push_back(0.0);
        p.accelerations.push_back(0.0); 

    }

    else if(req.operation == "open"){
        
        p.positions.push_back(0.0);
        p.velocities.push_back(0.0);
        p.accelerations.push_back(0.0); 

    }
    
    else{

        const std::string error = "Selected operations is not valid. Valid options are: open or close";
        ROS_ERROR_STREAM(error);
        res.succes = false;
        return false;
    }
    
    jt.header.stamp = ros::Time::now();
    jt.joint_names.push_back(joint_name);
    p.time_from_start = ros::Duration(2.5);
    jt.points.push_back(p);

    if (req.hand == "right") {
        const std::string info1 = req.operation + " right hand ... ";
        ROS_INFO_STREAM(info1);
        right_pub.publish(jt);
    }

    if (req.hand == "left") {
        const std::string info2 = req.operation + " left hand ... ";
        ROS_INFO_STREAM(info2);
        left_pub.publish(jt);
    }

    res.succes = true;
    return true;

}

int main(int argc, char **argv) {
  
    ros::init(argc, argv, "gripper_service_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("hand_service_server", serviceCallback);
    right_pub = n.advertise<trajectory_msgs::JointTrajectory>(right_topic, 10);
    left_pub = n.advertise<trajectory_msgs::JointTrajectory>(left_topic, 10);

    ros::spin();

    return 0;

}