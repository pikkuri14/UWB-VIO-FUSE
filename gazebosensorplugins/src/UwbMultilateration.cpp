
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gtec_msgs/Ranging.h"

void rngclbk(const gtec_msgs::Ranging::ConstPtr& msg)
{
  ROS_INFO("Anchor ID: [%d]", msg->anchorId);
  ROS_INFO("Tag ID: [%d]", msg->tagId);
  ROS_INFO("Range: [%d]", msg->range);
  ROS_INFO("Sequence: [%d]", msg->seq);
  ROS_INFO("RSS: [%f]", msg->rss);
  ROS_INFO("Error Estimation: [%f]", msg->errorEstimation);
}

int main (int argc, char **argv)
{  
  ros::init(argc, argv, "multilateration_node");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/multilateration_odom", 25);
  ros::Subscriber sub = n.subscribe("ranging", 1000, rngclbk);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "tag_0";
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 2.0; 
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 0.1;
    odom.twist.twist.linear.y = -0.1;
    odom.twist.twist.angular.z = 0.1;

    odom_pub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}