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
#include "sensor_msgs/LaserScan.h"

void ind2sub(long index, int N1, int N2, int N3, int* i, int* j, int* k)
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

double angle_diff(double a, double b)
{
  double angle = a - b;

  angle = fmod(a, 2.0*M_PI); // limit the angle from 0 to 2*Pi

  if(angle<=M_PI && angle>-M_PI)
    return angle;

  if(angle > M_PI)
  {
    // angle is between pi and 2*pi so it needs to wrap around
    return -M_PI + fmod(angle, M_PI);
  }

  return 0.0;
}

double prob(double a, double b)
{
  return (1/sqrt(2*M_PI*b*b))*exp(-0.5*((a*a)/(b*b)));
}

double motion_model(double* xt, double* ut, double* xt_d1)
{
  float alpha1 = 0.2, alpha2 = 0.2, alpha3=0.8, alpha4=0.2, alpha5=0.1;
  double x_prime = xt[0];
  double y_prime = xt[1];
  double theta_prime = xt[2];

  double x_bar = ut[0];
  double y_bar = ut[1];
  double theta_bar = ut[2];

  double x_bar_prime = ut[3];
  double y_bar_prime = ut[4];
  double theta_bar_prime = ut[5];

  double x = xt_d1[0];
  double y = xt_d1[1];
  double theta = xt_d1[2];

  double delta_rot1 = atan2(y_bar_prime-y_bar, x_bar_prime-x_bar) - theta_bar;
  double delta_trans =  sqrt((x_bar - x_bar_prime)*(x_bar - x_bar_prime) + (y_bar - y_bar_prime)*(y_bar - y_bar_prime));
  double delta_rot2 =  theta_bar_prime - theta_bar - delta_rot1; // NOTE: for some reason AMCL doesn't subtract the second theta

  double delta_rot1_hat =  atan2(y_prime-y, x_prime-x) - theta;
  double delta_trans_hat = sqrt((x-x_prime)*(x-x_prime) + (y-y_prime)*(y-y_prime));
  double delta_rot2_hat = theta_prime - theta - delta_rot1_hat;

  double a, b;
  a = angle_diff(delta_rot1, delta_rot1_hat);

  b = alpha1*(delta_rot1_hat*delta_rot1_hat)+alpha2*(delta_trans*delta_trans);
  double p1 = prob(a, b);

  a = delta_trans-delta_trans_hat;
  b = alpha3*(delta_trans_hat*delta_trans_hat)+alpha4*(delta_rot1_hat*delta_rot1_hat)+alpha4*(delta_rot2_hat*delta_rot2_hat);
  double p2 = prob(a, b);

  a = angle_diff(delta_rot2, delta_rot2_hat);

  b = alpha1*(delta_rot2_hat*delta_rot2_hat)+alpha2*(delta_trans_hat*delta_trans_hat);
  double p3 = prob(a, b);

  if(isnan(p1*p2*p3))
    return 0.0; // temporary fix

  return p1*p2*p3;
}

int map2ind (int x, int y, int map_height)
{
  return x*map_height + y;
}

double measurement_model(float min_angle, float angle_increment, float min_range, float max_range, std::vector<float> ranges, double* xt, double* sensor_pose, std::vector<float>& dist_map, int map_height)
{

  double q = 1.0;
  float z_hit = 0.95, z_random = 0.05, z_max = 0.05, sigma_hit = 0.2;
  float z_rand_max = z_random/z_max;
  for(int i = 0; i < 30; i++) // check 30 laser rays
  {
    if((ranges[i*12] == max_range) || (ranges[i*12] == min_range))
      continue;

    float x_zkt = floor(xt[0] + sensor_pose[0]*cos(xt[2]) - sensor_pose[1]*sin(xt[2]) + ranges[i*5]*cos(xt[2] + (min_angle+angle_increment*i*12))); // assume that the sensor is not mounted at angle
    float y_zkt = floor(xt[1] + sensor_pose[1]*cos(xt[2]) + sensor_pose[0]*sin(xt[2]) + ranges[i*5]*sin(xt[2] + (min_angle+angle_increment*i*12))); // assume that the sensor is not mounted at angle

    int index = map2ind(int(x_zkt), int(y_zkt), map_height);
    float dist = dist_map[index];

    q *= (z_hit*prob(dist, sigma_hit) + z_rand_max);
  }

  return q;
}

ros::Time latest_laser_scan_time;
std::vector<float> latest_laser_ranges;
float laser_angle_min;
float laser_angle_increment;
float laser_range_max;
float laser_range_min;


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  latest_laser_scan_time = msg->header.stamp;
  latest_laser_ranges = msg->ranges;
  laser_angle_min = msg->angle_min;
  laser_angle_increment = msg->angle_increment;
  laser_range_max = msg->range_max;
  laser_range_min = msg->range_min;
}

int* ind2map (int ind, int map_height)
{
  static int coord[2];
  coord[1] = ind % map_height;
  coord[0] = (ind - coord[1])/map_height;
  return coord;
}

float measure_distance(int index_a, int index_b, int map_width, int map_height, float resolution)
{
  float x, y, x_prime, y_prime;
  int* coord;
  coord = ind2map(index_a, map_height);
  x = coord[0]*resolution;
  y = coord[1]*resolution;

  coord = ind2map(index_b, map_height);
  x_prime = coord[0]*resolution;
  y_prime = coord[1]*resolution;

  return sqrt((x-x_prime)*(x-x_prime) + (y-y_prime)*(y-y_prime));
}

nav_msgs::OccupancyGrid likelihood_field;

std::vector<float>& compute_likelihood_field(const nav_msgs::OccupancyGrid& map)
{
  // TODO: find a more efficient implementation
  // TODO: calculate the probabilities in this function
  int map_width = map.info.width;
  int map_height = map.info.height;
  float resolution = map.info.resolution;

  static std::vector<float> dist_map(1000000, 0.0);
  for (int i = 0; i < map_width*map_height; i++)
  {
    if(map.data[i] == 100) // if there's an obstacle in this cells
    {
      dist_map[i] = 0;
      continue;
    }

    for (int j = 0; j < map_width*map_height; j++)
    {
      if(map.data[j] != 100) // if this is not an obstacle
        continue;

      float temp = measure_distance(i, j, map_width, map_height, resolution);
      if((dist_map[i] == 0) || (temp < dist_map[i]))
        dist_map[i] = temp;
    }
  }
  return dist_map;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laser_callback);
  ros::ServiceClient map_srv_client = n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap map_srv;

  ROS_INFO("Waiting for map...");
  if(map_srv_client.call(map_srv))
    ROS_INFO("Map received!");
  else
    ROS_INFO("Failed to call map service");

  ROS_INFO("Calculating likelihood field...");
  std::vector<float>& dist_map = compute_likelihood_field(map_srv.response.map);
  // TODO: provide the option to publish likelihood_field
  ROS_INFO("Completed likelihood field calculation");

  // Define grid resolution
  double map_width = map_srv.response.map.info.resolution*map_srv.response.map.info.width;
  double map_height = map_srv.response.map.info.resolution*map_srv.response.map.info.height;

  double linear_resolution = 0.50; // 50 cm
  double angular_resolution = 0.63; // 36 degress in radians

  int grid_width = std::floor(map_width/linear_resolution);
  int grid_length = std::floor(map_height/linear_resolution);
  int grid_depth = std::floor((2*M_PI)/angular_resolution);

  long number_of_grid_cells = grid_width*grid_length*grid_depth;

  // Initialise distribution uniformly
  ROS_INFO("Initialising distribution...");
  double previous_dist[50][50][73];
  double current_dist[50][50][73];
  double temp_prob = 1.0/number_of_grid_cells;
  double p_bar_kt[50][50][73] = {};
  for (int r = 0; r < grid_length; r++)
  {
    for (int c = 0; c < grid_width; c++)
    {
      for (int d = 0; d < grid_depth; d++)
      {
        previous_dist[r][c][d] = temp_prob;
        current_dist[r][c][d] = temp_prob;
      }
    }
  }
  ROS_INFO("Initialised...");

  tf::TransformListener tf_listener;
  tf::StampedTransform current_transform;
  tf::StampedTransform previous_transform;
  tf::StampedTransform laser_transform;

  tf_listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), previous_transform); // get initial transform

  tf_listener.waitForTransform("/base_footprint", "/base_laser_front_link", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/base_footprint", "/base_laser_front_link", ros::Time(0), laser_transform);
  double laser_pose[3] = {laser_transform.getOrigin().getX(), laser_transform.getOrigin().getY(), tf::getYaw(laser_transform.getRotation())};

  ros::Rate loop_rate(10); // 10 Hz
  while(ros::ok())
  {
    double sum_of_dist_values = 0.0;
    // Take a laser scan and get the transform at this scan.
    ros::spinOnce();
    ros::Time current_laser_scan_time = latest_laser_scan_time;
    std::vector<float> current_laser_ranges = latest_laser_ranges;
    try{
      tf_listener.waitForTransform("/base_footprint", "/odom", current_laser_scan_time, ros::Duration(3.0));
      tf_listener.lookupTransform("/base_footprint", "/odom", current_laser_scan_time, current_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    double ut[6] = {previous_transform.getOrigin().getX(), previous_transform.getOrigin().getY(), tf::getYaw(previous_transform.getRotation()),
                    current_transform.getOrigin().getX(), current_transform.getOrigin().getY(), tf::getYaw(current_transform.getRotation())};

    if(isNoMovement(previous_transform, current_transform)) // Don't do anything if the robot didn't move
      continue;

    ROS_INFO("Round Started!");
    for(long k = 0; k < number_of_grid_cells; k++) // line 2 of table 8.1
    {
      int rowk, colk, depthk;
      double xt[3];

      ind2sub(k, grid_length, grid_width, grid_depth, &rowk, &colk, &depthk);
      depthk = k % grid_depth;
      colk = ((k - depthk)/grid_depth) % grid_width;
      rowk = (((k - depthk)/grid_depth) - colk) / grid_width;

      xt[0] = rowk*linear_resolution;
      xt[1] = colk*linear_resolution;
      xt[2] = depthk*angular_resolution;

      // ================ Prediction =================
      for(long i = 0; i < number_of_grid_cells; i++)
      {
        int rowi, coli, depthi;
        double xt_d1[3];

        depthi = i % grid_depth;
        coli = ((i - depthi)/grid_depth) % grid_width;
        rowi = (((i - depthk)/grid_depth) - coli) / grid_width;
        xt_d1[0] = rowi*linear_resolution;
        xt_d1[1] = coli*linear_resolution;
        xt_d1[2] = depthi*angular_resolution;

        p_bar_kt[rowk][colk][depthk] += previous_dist[rowi][coli][depthi]*motion_model(xt, ut, xt_d1);
      }
      // =============================================
      // Calculate the unnormalised p_kt
      float temp_measure_model = measurement_model(laser_angle_min, laser_angle_increment, laser_range_min, laser_range_max, current_laser_ranges, xt, laser_pose, dist_map, map_srv.response.map.info.height);
      current_dist[rowk][colk][depthk] = p_bar_kt[rowk][colk][depthk]*temp_measure_model;
      sum_of_dist_values += current_dist[rowk][colk][depthk];
    }

    int max_r=0, max_c=0, max_d=0;
    double max_prob = 0;

    // normalise p_kt and publish the transform
    for(int row = 0; row < grid_length; row++)
    {
      for(int col = 0; col < grid_width; col++)
      {
        for(int dep = 0; dep < grid_depth; dep++)
        {
          current_dist[row][col][dep] /= sum_of_dist_values;
          previous_dist[row][col][dep] = current_dist[row][col][dep];
        }
      }
    }

    previous_transform = current_transform;

    ROS_INFO("One round completed!");
    loop_rate.sleep();
  }

  ros::spin(); // do I need this?

  return 0;
}
