#include "ros/ros.h"
#include "std_msgs/Float64.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Float64>("setValue", 10);

  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
  
    std_msgs::Float64 sv;

    sv.data = 1.0;
   
    chatter_pub.publish(sv);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
