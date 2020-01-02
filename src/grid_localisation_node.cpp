#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"

// Define global variables
nav_msgs::Odometry current_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom.pose.pose = msg->pose.pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("odom", 100, odomCallback);

  ros::ServiceClient map_srv_client = n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap map_srv;

  ROS_INFO("Waiting for map...");
  if(map_srv_client.call(map_srv))
    ROS_INFO("Map received!");
  else
    ROS_INFO("Failed to call map service");

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
