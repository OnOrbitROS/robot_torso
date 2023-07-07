#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "talos_control/hand_operations.h"

bool close_hand = false;
talos_control::hand_operations hand_operation;

void goalReachedCallback(const std_msgs::Bool& goal_reached){
    hand_operation.request.operation = "close";
    hand_operation.request.hand = "right";
    close_hand = goal_reached.data;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "look_to_aruco_node");
    ros::NodeHandle n;

    ros::Publisher head_pan_publisher = n.advertise<std_msgs::Float64>("/talos/head_pan_controller/command", 1000);
    ros::Publisher head_tilt_publisher = n.advertise<std_msgs::Float64>("/talos/head_tilt_controller/command", 1000);
    ros::Subscriber goal_subscr = n.subscribe("/goal_tolerance", 1000, goalReachedCallback);
    ros::ServiceClient hand_client = n.serviceClient<talos_control::hand_operations>("hand_service_server");

    std_msgs::Float64 head_pan_move;
    std_msgs::Float64 head_tilt_move;
    head_pan_move.data = -0.50;
    head_tilt_move.data = -0.05;
    bool stop = false;

    ros::Duration(0.5).sleep(); //Sleep for 5 secs
    head_pan_publisher.publish(head_pan_move);
    head_tilt_publisher.publish(head_tilt_move);

    while(ros::ok){ 

        if(close_hand and not stop){
            hand_client.call(hand_operation);
            if(hand_operation.response.succes){
                stop = true;
            }
        }

        ros::spinOnce();
    }

    return 0;
}