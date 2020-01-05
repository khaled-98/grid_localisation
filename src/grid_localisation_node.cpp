#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Transform.h"
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

// Define global variables
nav_msgs::Odometry current_odom;
nav_msgs::Odometry previous_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom.pose.pose = msg->pose.pose;
}

void ind2sub(int index, int N1, int N2, int N3, int* i, int* j, int* k)
{
  // Don't event ask
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  int n1, n2, n3;
  n3 = index % N3;
  n2 = ((index-n3)/N3)%N2;
  n1 = (((index-n3)/N3)-n2)/N2;
  *i = n1;
  *j = n2;
  *k = n3;
}

double angle_from_orientation(geometry_msgs::Quaternion orientation)
{
  // orientation (geometry_msgs::Quaternion) is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  return yaw;
}

bool isNoMovement(tf::StampedTransform prev, tf::StampedTransform curr)
{
  double linear_tol=0.1, angular_tol=0.052; // 3 degrees in radians
  double x = prev.getOrigin().getX();
  double y = prev.getOrigin().getY();
  double theta = tf::getYaw(prev.getRotation());

  double x_prime = curr.getOrigin().getX();
  double y_prime = curr.getOrigin().getY();
  double theta_prime = tf::getYaw(curr.getRotation());

  // Check if the distance travelled is bigger than the tolerance
  if(sqrt((pow(x-x_prime, 2))+pow(y-y_prime, 2)) < linear_tol)
    return true;

  // Check if the angle travelled is bigger than the tolerance
  if(fabs(theta - theta_prime) < angular_tol)
    return true;

  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;

  ros::ServiceClient map_srv_client = n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap map_srv;

  ROS_INFO("Waiting for map...");
  if(map_srv_client.call(map_srv))
    ROS_INFO("Map received!");
  else
    ROS_INFO("Failed to call map service");

  // Define grid resolution
  double map_width = map_srv.response.map.info.resolution*map_srv.response.map.info.width;
  double map_height = map_srv.response.map.info.resolution*map_srv.response.map.info.height;

  double linear_resolution = 0.15; // 15 cm
  double angular_resolution = 0.0873; // 5 degress in radians

  int grid_width = std::floor(map_width/linear_resolution);
  int grid_length = std::floor(map_height/linear_resolution);
  int grid_depth = std::floor((2*M_PI)/angular_resolution);

  long number_of_grid_cells = grid_width*grid_length*grid_depth;

  // Initialise distribution uniformly
  std::vector<std::vector<std::vector<double> > > previous_dist (grid_width,std::vector<std::vector<double> >(grid_length,std::vector <double>(grid_depth,1/number_of_grid_cells)));
  std::vector<std::vector<std::vector<double> > > current_dist;
  current_dist = previous_dist;

  // Initliase with zeros
  std::vector<std::vector<std::vector<double> > > p_bar_kt (grid_width,std::vector<std::vector<double> >(grid_length,std::vector <double>(grid_depth,0)));

  tf::TransformListener tf_listener;
  tf::StampedTransform current_transform;
  tf::StampedTransform previous_transform;

  tf_listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), previous_transform); // get initial transform

  ros::Rate loop_rate(10); // 10 Hz
  while(ros::ok())
  {
    try{
      tf_listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), current_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if(isNoMovement(previous_transform, current_transform)) // Don't do anything if the robot didn't move
      continue;

    ROS_INFO("Round Started!");
    for(long k = 0; k < number_of_grid_cells; k++) // line 2 of table 8.1
    {
      int rowk, colk, depthk;
      ind2sub(k, grid_width, grid_length, grid_depth, &rowk, &colk, &depthk);

      // ================ Prediction =================
      for(long i = 0; i < number_of_grid_cells; i++)
      {
        int rowi, coli, depthi;
        ind2sub(i, grid_width, grid_length, grid_depth, &rowi, &coli, &depthi);
      }
    }
    previous_transform = current_transform;

    ROS_INFO("One round completed!");
    ros::spinOnce();
    loop_rate.sleep();
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
