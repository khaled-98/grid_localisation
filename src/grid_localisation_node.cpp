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

double normalise(double z)
{
  return atan2(sin(z),cos(z));
}

double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalise(a);
  b = normalise(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

double prob(double a, double b)
{
  return (1/sqrt(2*M_PI*b))*exp(-0.5*(pow(a, 2)/b));
}

double motion_model(double* xt, tf::StampedTransform prev, tf::StampedTransform curr, double* xt_d1)
{
  float alpha1 = 0.2, alpha2 = 0.2, alpha3=0.8, alpha4=0.2, alpha5=0.1;
  double x_prime = xt[0];
  double y_prime = xt[1];
  double theta_prime = xt[2];

  double x_bar = prev.getOrigin().getX();
  double y_bar = prev.getOrigin().getY();
  double theta_bar = tf::getYaw(prev.getRotation());

  double x_bar_prime = curr.getOrigin().getX();
  double y_bar_prime = curr.getOrigin().getY();
  double theta_bar_prime = tf::getYaw(curr.getRotation());

  double x = xt_d1[0];
  double y = xt_d1[1];
  double theta = xt_d1[2];

  double delta_rot1 = angle_diff(atan2(y_bar_prime-y_bar, x_bar_prime-x_bar), theta_bar);
  double delta_trans = sqrt(pow((x_bar - x_bar_prime), 2) + pow((y_bar - y_bar_prime), 2));
  double delta_rot2 = angle_diff(theta_bar_prime, angle_diff(theta_bar, delta_rot1)); // NOTE: for some reason AMCL doesn't subtract the second theta

  double delta_rot1_hat = angle_diff(atan2(y_prime-y, x_prime-x), theta);
  double delta_trans_hat = sqrt(pow((x-x_prime), 2) + pow((y-y_prime), 2));
  double delta_rot2_hat = angle_diff(theta_prime, angle_diff(theta, delta_rot1_hat));

  double p1 = prob(angle_diff(delta_rot1, delta_rot1_hat), alpha1*(pow(delta_rot1_hat,2))+alpha2*(pow(delta_trans, 2)));
  double p2 = prob(delta_trans-delta_trans_hat, alpha3*(pow(delta_trans_hat, 2))+alpha4*(pow(delta_rot1_hat, 2))+alpha4*(pow(delta_rot2_hat, 2)));
  double p3 = prob(angle_diff(delta_rot2, delta_rot2_hat), alpha1*(pow(delta_rot2_hat, 2))+alpha2*(pow(delta_trans_hat, 2)));

  return p1*p2*p3;
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
      double xt[3];

      ind2sub(k, grid_width, grid_length, grid_depth, &rowk, &colk, &depthk);
      xt[0] = rowk*linear_resolution;
      xt[1] = colk*linear_resolution;
      xt[2] = depthk*angular_resolution;

      // TODO: BE CAREFUL ABOUT WHEN YOU TAKE THE TRANSFORM READINGS
      // ================ Prediction =================
      for(long i = 0; i < number_of_grid_cells; i++)
      {
        int rowi, coli, depthi;
        double xt_d1[3];

        ind2sub(i, grid_width, grid_length, grid_depth, &rowi, &coli, &depthi);
        xt_d1[0] = rowi*linear_resolution;
        xt_d1[1] = coli*linear_resolution;
        xt_d1[2] = depthi*angular_resolution;

        p_bar_kt[rowk][colk][depthk] += previous_dist[rowi][coli][depthi]*motion_model(xt, previous_transform, current_transform, xt_d1);
      }
    }
    previous_transform = current_transform;

    ROS_INFO("One round completed!");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin(); // do I need this?

  return 0;
}
