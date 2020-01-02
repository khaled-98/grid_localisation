#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "LinearMath/btMatrix3x3.h"

// Define global variables
nav_msgs::Odometry current_odom;
nav_msgs::Odometry previous_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom.pose.pose = msg->pose.pose;
}

int* ind2sub(int index, int N1, int N2, int N3)
{
  // Don't event ask
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  static int sub[3];
  int n1, n2, n3;
  n3 = index % N3;
  n2 = ((index-n3)/N3)%N2;
  n1 = (((index-n3)/N3)-n2)/N2;
  sub[0] = n1;
  sub[1] = n2;
  sub[2] = n3;
  return sub;
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

bool isNoMovement(nav_msgs::Odometry prev, nav_msgs::Odometry curr)
{
  double linear_tol=0.1, angular_tol=0.052; // 3 degrees in radians
  double x = prev.pose.pose.position.x;
  double y = prev.pose.pose.position.y;
  double theta = angle_from_orientation(prev.pose.pose.orientation);

  double x_prime = curr.pose.pose.position.x;
  double y_prime = curr.pose.pose.position.y;
  double theta_prime = curr.angle_from_orientation(curr.pose.pose.orientation);

  // Check if the distance travelled is less than the tolerance

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

  previous_odom = current_odom;

  ros::Rate loop_rate(10); // 10 Hz
  while(ros::ok())
  {
    if(isNoMovement(previous_odom, current_odom)) // Don't do anything if the robot didn't move
      continue;

    for(long k = 0; k < number_of_grid_cells; k++)
    {
      for(long i = 0; i < number_of_grid_cells; i++)
      {
        int* sub = ind2sub(k, grid_width, grid_length, grid_depth);
        int rowk = sub[0], colk = sub[1], depthk = sub[2];

      }
    }

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
