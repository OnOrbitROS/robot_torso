// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/satt_arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("joint1");
  goal.trajectory.joint_names.push_back("joint2");
  goal.trajectory.joint_names.push_back("joint3");
  goal.trajectory.joint_names.push_back("joint4");
  goal.trajectory.joint_names.push_back("joint5");
  goal.trajectory.joint_names.push_back("joint6");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(4);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = 0.0;
  goal.trajectory.points[index].positions[3] = -0.397772713;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for (int j = 0; j < 6; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  //goal.trajectory.points[index].time_from_start = ros::Duration(682.0);
  goal.trajectory.points[index].time_from_start = ros::Duration(5.0);

  // Second trajectory point
  // Positions
  index = 1;
  goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.26382884;
  goal.trajectory.points[index].positions[2] = 0.324712419;
  goal.trajectory.points[index].positions[3] = -0.34094804;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for (int j = 0; j < 6; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  //goal.trajectory.points[index].time_from_start = ros::Duration(705.0);
  goal.trajectory.points[index].time_from_start = ros::Duration(10.0);
  // Third trajectory point
  // Positions
  index = 2;
 goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.26382884;
  goal.trajectory.points[index].positions[2] = 0.324712419;
  goal.trajectory.points[index].positions[3] = -0.34094804;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for (int j = 0; j < 6; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  //goal.trajectory.points[index].time_from_start = ros::Duration(744.0);
  goal.trajectory.points[index].time_from_start = ros::Duration(15.0);

  // Fourth trajectory point
  // Positions
  index = 3;
  goal.trajectory.points[index].positions.resize(6);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = 0.0;
  goal.trajectory.points[index].positions[3] = -0.397772713;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = 0.0;
  
  // Velocities
  goal.trajectory.points[index].velocities.resize(6);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  //goal.trajectory.points[index].time_from_start = ros::Duration(767.0);
  goal.trajectory.points[index].time_from_start = ros::Duration(20.0);

}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting run_traj_control application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_goal(arm_goal);

  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(30).sleep(); // sleep for ten seconds
  }

  return EXIT_SUCCESS;
}