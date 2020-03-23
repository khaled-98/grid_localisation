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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

void ind2sub(long index, int N1, int N2, int N3, int* i, int* j, int* k)
{
  // Don't even ask
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  int n1, n2, n3;
  n3 = index % N3;
  n2 = ((index-n3)/N3)%N2;
  n1 = (((index-n3)/N3)-n2)/N2;
  *i = n1;
  *j = n2;
  *k = n3;
}

long sub2ind(int n1, int n2, int n3, int N2, int N3)
{
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  return n3 + N3*(n2+N2*n1);
}

bool isNoMovement(tf::StampedTransform prev, tf::StampedTransform curr)
{
  double linear_tol=0.1; // 3 degrees in radians
  double x = prev.getOrigin().getX();
  double y = prev.getOrigin().getY();
  double theta = tf::getYaw(prev.getRotation());

  double x_prime = curr.getOrigin().getX();
  double y_prime = curr.getOrigin().getY();
  double theta_prime = tf::getYaw(curr.getRotation());

  // Check if the distance travelled is bigger than the tolerance
  if(sqrt((pow(x-x_prime, 2))+pow(y-y_prime, 2)) > linear_tol)
    return false;

  return true;
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
  return (1.0/(sqrt(2*M_PI)*b))*exp(-0.5*((a*a)/(b*b)));
}

float a1 = 0.1, a2 = 0.1, a3 = 0.1, a4 = 0.1;
double motion_model(double* xt, double* ut, double* xt_d1)
{
  float alpha1 = a1, alpha2 = a2, alpha3=a3, alpha4=a4;
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

  double a, b, p1, p2, p3;
  a = angle_diff(delta_rot1, delta_rot1_hat);
  b = sqrt(alpha1*delta_rot1_hat*delta_rot1_hat + alpha2*delta_trans_hat*delta_trans_hat);
  if(b == 0)
    p1 = 1.0;
  else
    p1 = prob(a, b);

  a = delta_trans-delta_trans_hat;
  b = sqrt(alpha3*delta_trans_hat*delta_trans_hat + alpha4*delta_rot1_hat*delta_rot1_hat + alpha4*delta_rot2_hat*delta_rot2_hat);
  if(b == 0)
    p2 = 1.0;
  else
    p2 = prob(a, b);
  
  a = angle_diff(delta_rot2, delta_rot2_hat);
  b = sqrt(alpha1*delta_rot2_hat*delta_rot2_hat + alpha2*delta_trans_hat*delta_trans_hat);
  if(b == 0)
    p3 = 1.0;
  else
    p3 = prob(a, b);

  return p1*p2*p3;
}

int map2ind (int x, int y, int map_width)
{
  return y*map_width + x;
}

std::vector<float> dist_map(1000000, 0.0);

double measurement_model(float min_angle, float angle_increment, float min_range, float max_range, std::vector<float> ranges, double* xt, double* sensor_pose, int map_width, int map_height, double map_origin_x, double map_origin_y, float map_resolution)
{

  double q = 1.0;
  float z_hit = 0.95, z_random = 0.05, z_max = max_range, sigma_hit = 0.2;
  float z_rand_max = z_random/z_max;
  double min_x = map_origin_x;
  double max_x = map_origin_x + map_width*map_resolution;
  double min_y = map_origin_y;
  double max_y = map_origin_y + map_height*map_resolution;

  for(int i = 0; i < 30; i++) // check 30 laser rays
  {
    // If the reading is above the maximum or below the minimum range of the LiDAR, discard it
    if((ranges[i*5] >= max_range) || (ranges[i*5] <= min_range))
      continue;

    // Calculate the angle of the beam and bound it to [-pi, pi]
    float beam_angle = min_angle+angle_increment*i*5;
    if(beam_angle < 0)
      beam_angle += 2*M_PI;
      
    // Project the beam end-point onto the map
    float x_zkt = xt[0] + sensor_pose[0]*cos(xt[2]) - sensor_pose[1]*sin(xt[2]) + ranges[i*5]*cos(xt[2] + beam_angle); // assume that the sensor is not mounted at angle
    float y_zkt = xt[1] + sensor_pose[1]*cos(xt[2]) + sensor_pose[0]*sin(xt[2]) + ranges[i*5]*sin(xt[2] + beam_angle); // assume that the sensor is not mounted at angle

    // temproary fix - discard readings that are out of bound
    if(x_zkt < min_x || x_zkt > max_x || y_zkt < min_y || y_zkt > max_y) // THESE VALUES NEED TO BE CHANGED!
      continue;

    
    // Project the points onto the likelihood field by subreacting the origin and dividing by the map resoluiton
    x_zkt -= map_origin_x;
    y_zkt -= map_origin_y;

    x_zkt /= map_resolution;
    y_zkt /= map_resolution;
    
    // Get the relevant point from the likelhood field
    int index = map2ind(int(x_zkt), int(y_zkt), map_width);
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

int* ind2map (int ind, int map_width)
{
  static int coord[2];
  coord[0] = ind % map_width;            // The x's are along the cols
  coord[1] = (ind - coord[0])/map_width; // The y's are along the rows
  return coord;
}

float measure_distance(int index_a, int index_b, int map_width, float resolution)
{
  float x, y, x_prime, y_prime;
  int* coord;
  coord = ind2map(index_a, map_width);
  x = coord[0]*resolution;
  y = coord[1]*resolution;

  coord = ind2map(index_b, map_width);
  x_prime = coord[0]*resolution;
  y_prime = coord[1]*resolution;

  return sqrt((x-x_prime)*(x-x_prime) + (y-y_prime)*(y-y_prime));
}

void compute_likelihood_field(const nav_msgs::OccupancyGrid& map)
{
  // TODO: find a more efficient implementation
  // TODO: calculate the probabilities in this function
  int map_width = map.info.width;
  int map_height = map.info.height;
  float resolution = map.info.resolution;

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

      float temp = measure_distance(i, j, map_width, resolution);
      if((dist_map[i] == 0) || (temp < dist_map[i]))
      {
        dist_map[i] = temp;
      }
    }
  }
}

geometry_msgs::PoseWithCovarianceStamped init_pose;
bool init_pose_recieved = false;
void init_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  init_pose.header = msg->header;
  init_pose.pose = msg->pose;
  init_pose_recieved = true;
  ROS_INFO("New pose receieved");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");
  private_n.getParam("a1", a1);
  private_n.getParam("a2", a2);
  private_n.getParam("a3", a3);
  private_n.getParam("a4", a4);
  ROS_INFO("Alpha values: %f, %f, %f, %f", a1, a2, a3, a4);

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("grid_pose", 1);
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laser_callback);
  ros::Subscriber init_pose_sub = n.subscribe("initialpose", 10, init_pose_callback);
  ros::ServiceClient map_srv_client = n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap map_srv;

  ROS_INFO("Waiting for map...");
  if(map_srv_client.call(map_srv))
    ROS_INFO("Map received!");
  else
    ROS_INFO("Failed to call map service");

  ROS_INFO("Calculating likelihood field...");
  compute_likelihood_field(map_srv.response.map);
  // TODO: provide the option to publish likelihood_field
  ROS_INFO("Completed likelihood field calculation");

  // Define grid resolution
  double map_width = map_srv.response.map.info.resolution*map_srv.response.map.info.width;
  double map_height = map_srv.response.map.info.resolution*map_srv.response.map.info.height;

  double linear_resolution = 0.15; // 50 cm
  double angular_resolution = 0.785; // 36 degress in radians

  int grid_width = std::floor(map_width/linear_resolution);
  int grid_height = std::floor(map_height/linear_resolution);
  int grid_depth = std::floor((2*M_PI)/angular_resolution);

  ROS_INFO("Grid width: %d", grid_width);
  ROS_INFO("Grid height: %d", grid_height);
  ROS_INFO("Grid depth: %d", grid_depth);
  long number_of_grid_cells = grid_width*grid_height*grid_depth;

  ROS_INFO("Number of grid cells: %d", number_of_grid_cells);

  // Get map origin
  // TODO: account for origin rotation
  double map_origin_x = map_srv.response.map.info.origin.position.x;
  double map_origin_y = map_srv.response.map.info.origin.position.y;

  // Initialise distribution uniformly
  ROS_INFO("Initialising distribution...");
  double previous_dist[50][50][73];
  double current_dist[50][50][73];
  double temp_prob = 0.5/(number_of_grid_cells-1); // equal distribuition at all the places where the robot isn't at
  for (int r = 0; r < grid_height; r++)
  {
    for (int c = 0; c < grid_width; c++)
    {
      for (int d = 0; d < grid_depth; d++)
      {
        previous_dist[r][c][d] = temp_prob;
        p_bar_kt[r][c][d] = 0.0;
      }
    }
  }
  ROS_INFO("Initialised...");

  tf::TransformListener tf_listener;
  tf::StampedTransform current_transform;
  tf::StampedTransform previous_transform;
  tf::StampedTransform laser_transform;

  tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), previous_transform); // get initial transform

  tf_listener.waitForTransform("/base_link", "/base_laser_front_link", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/base_link", "/base_laser_front_link", ros::Time(0), laser_transform);
  double laser_pose[3] = {laser_transform.getOrigin().getX(), laser_transform.getOrigin().getY(), tf::getYaw(laser_transform.getRotation())};

  ros::Rate loop_rate(0.5); // 10 Hz
  double max_prob = 0.0;

  geometry_msgs::PoseWithCovarianceStamped current_pose;
  int rolling_window_size = floor(2.0/linear_resolution);

  while(ros::ok())
  {
    if(init_pose_recieved)
    {
      ROS_INFO("Initial pose: %f, %f", init_pose.pose.pose.position.x, init_pose.pose.pose.position.y);
      int c = floor(float(init_pose.pose.pose.position.x - map_origin_x)/linear_resolution);
      int r = floor(float(init_pose.pose.pose.position.y - map_origin_y)/linear_resolution);
      // The orientation is given in the range [-pi, pi], so it is shifted to [0, 2pi]
      // for ease of conversion into grid coordinates
      float temp_d = tf::getYaw(init_pose.pose.pose.orientation);
      if(temp_d < 0)
        temp_d += 2*M_PI;
      temp_d /= angular_resolution;
      int d = floor(temp_d);
      ROS_INFO("Initial grid location: %d, %d, %d", c, r, d);
      previous_dist[r][c][d] = 0.5;
      //TODO: set all the others to zero!
      init_pose_recieved = false;

      current_pose.pose.pose.position.x = init_pose.pose.pose.position.x;
      current_pose.pose.pose.position.y = init_pose.pose.pose.position.y;
    }

    long double sum_of_dist_values = 0.0;
    // Take a laser scan and get the transform at this scan.
    ros::spinOnce();
    ros::Time current_laser_scan_time = latest_laser_scan_time;
    std::vector<float> current_laser_ranges = latest_laser_ranges;
    try{
      tf_listener.waitForTransform("/odom", "/base_link", current_laser_scan_time, ros::Duration(3.0));
      tf_listener.lookupTransform("/odom", "/base_link", current_laser_scan_time, current_transform);
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

    // rolling window calculations
    // 1) Where to start the window from
    double window_start_x = current_pose.pose.pose.position.x - 1;
    double window_start_y = current_pose.pose.pose.position.y - 1;
    // 2) Adjust if out of bound
    if(window_start_x < map_srv.response.map.info.origin.position.x)
      window_start_x = map_srv.response.map.info.origin.position.x;

    if(window_start_y < map_srv.response.map.info.origin.position.y)
      window_start_y = map_srv.response.map.info.origin.position.y;

    window_start_x -= map_srv.response.map.info.origin.position.x;
    window_start_x /= linear_resolution;

    window_start_y -= map_srv.response.map.info.origin.position.y;
    window_start_y /= linear_resolution;

    long window_start_index = sub2ind(int(window_start_y), int(window_start_x), 0, grid_width, grid_depth);

    // 3) Where to end the window
    double window_end_x = current_pose.pose.pose.position.x + 1;
    double window_end_y = current_pose.pose.pose.position.y + 1;

    window_end_x -= map_srv.response.map.info.origin.position.x;
    window_end_x /= linear_resolution;

    window_end_y -= map_srv.response.map.info.origin.position.y;
    window_end_y /= linear_resolution;

    long window_end_index = sub2ind(int(window_end_y), int(window_end_x), 0, grid_width, grid_depth);

    // 4) Adjust if out of bound
    if(window_end_index > number_of_grid_cells)
      window_end_index = number_of_grid_cells;

    long counterK = 1;
    for(long k = window_start_index; k < window_end_index; k++) // line 2 of table 8.1
    {
      // Skip cells outside the rolling window
      if(counterK == (rolling_window_size*grid_depth))
      {
        k += (grid_width - rolling_window_size)*grid_depth - 1;
        counterK = 1;
        continue;
      }
      counterK++;

      int rowk, colk, depthk;
      double xt[3];

      ind2sub(k, grid_height, grid_width, grid_depth, &rowk, &colk, &depthk);

      xt[0] = colk*linear_resolution + map_origin_x;
      xt[1] = rowk*linear_resolution + map_origin_y;
      xt[2] = depthk*angular_resolution;
      if(xt[2] > M_PI)
        xt[2] -= 2*M_PI;

      // ================ Prediction =================
      long counterI = 1;
      for(long i = window_start_index; i < window_end_index; i++)
      {
        if(i==k)
          continue;
        if(counterI == (rolling_window_size*grid_depth))
        {
          i += (grid_width - rolling_window_size)*grid_depth - 1;
          counterI = 1;
          continue;
        }
        counterI++;

        int rowi, coli, depthi;
        double xt_d1[3];

        ind2sub(i, grid_height, grid_width, grid_depth, &rowi, &coli, &depthi);
        xt_d1[0] = coli*linear_resolution + map_origin_x;
        xt_d1[1] = rowi*linear_resolution + map_origin_y;
        xt_d1[2] = depthi*angular_resolution;
        if(xt_d1[2] > M_PI)
          xt_d1[2] -= 2*M_PI;

        p_bar_kt[rowk][colk][depthk] += previous_dist[rowi][coli][depthi]*motion_model(xt, ut, xt_d1);
      }

      // =============================================
      // Calculate the unnormalised p_kt
      double temp_measure_model = measurement_model(laser_angle_min, laser_angle_increment, laser_range_min, laser_range_max, current_laser_ranges, xt, laser_pose, map_srv.response.map.info.width, map_srv.response.map.info.height, map_origin_x, map_origin_y, map_srv.response.map.info.resolution);
      current_dist[rowk][colk][depthk] = p_bar_kt[rowk][colk][depthk]*temp_measure_model;
      sum_of_dist_values += current_dist[rowk][colk][depthk];
    }

    int max_r=0, max_c=0, max_d=0;
    max_prob = 0.0;

    // normalise p_kt and publish the transform
    counterK = 1;
    for(long k=window_start_index; k<window_end_index; k++)
    {
      if(counterK == (rolling_window_size*grid_depth))
      {
        k += (grid_width - rolling_window_size)*grid_depth - 1;
        counterK = 1;
        continue;
      }
      counterK++;

      int row, col, dep;
      ind2sub(k, grid_height, grid_width, grid_depth, &row, &col, &dep);
      current_dist[row][col][dep] /= sum_of_dist_values;
      previous_dist[row][col][dep] = current_dist[row][col][dep];
      p_bar_kt[row][col][dep] = 0.0;
      if(current_dist[row][col][dep] > max_prob)
      {
        max_r = row;
        max_c = col;
        max_d = dep;
        max_prob = current_dist[row][col][dep];
      }

      }
    }  


    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "map";
    current_pose.pose.pose.position.x = max_c*linear_resolution+map_origin_x;
    current_pose.pose.pose.position.y = max_r*linear_resolution+map_origin_y;

    // TODO: use actual rotation
    current_pose.pose.pose.orientation.x = 0;
    current_pose.pose.pose.orientation.y = 0;
    current_pose.pose.pose.orientation.z = 0;
    current_pose.pose.pose.orientation.w = 1;

    pose_pub.publish(current_pose);

    previous_transform = current_transform;

    ROS_INFO("One round completed!");
    ros::spinOnce();
  }

  ros::spin(); // do I need this?

  return 0;
}
