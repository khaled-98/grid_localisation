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
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <thread>
#include <future>
#include <unordered_map>

void ind2sub(unsigned long long index, int N1, int N2, int N3, int* i, int* j, int* k)
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

unsigned long long sub2ind(int n1, int n2, int n3, int N2, int N3)
{
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  return n3 + N3*(n2+N2*n1);
}

double angle_diff(double a, double b)
{
  double angle = a - b;

  angle = fmod(angle, 2.0*M_PI); // limit the angle from 0 to 2*Pi

  if(angle <= M_PI && angle >= -M_PI) // angle within the desired limit
    return angle;

  else if(angle > M_PI)   
    return angle - 2.0*M_PI;
  
  else
    return angle + 2.0*M_PI;
}

double prob(double a, double b)
{
  return (1.0/(sqrt(2*M_PI)*b))*exp(-0.5*((a*a)/(b*b)));
}

int map2ind (int x, int y, int map_width)
{
  return y*map_width + x;
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

class GridLocalisationNode
{
public:
  GridLocalisationNode();
  ~GridLocalisationNode();

private:
  tf::TransformBroadcaster* tfb_;
  tf::TransformListener* tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;

  tf::StampedTransform curr_odom_to_base_tf_;
  tf::StampedTransform prev_odom_to_base_tf_;

  ros::Subscriber initial_pose_sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pose_pub_;

  std::string scan_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string laser_frame_id_;
  std::string global_frame_id_;

  double alpha1_; 
  double alpha2_; 
  double alpha3_; 
  double alpha4_;
  double alpha5_;

  double z_hit_;
  double z_random_;
  double sigma_hit_;
  bool laser_info_recieved_;
  double laser_max_range_;
  double laser_min_range_;
  double laser_min_angle_;
  double laser_angle_increment_;
  int number_of_beams_;
  int number_of_beams_to_use_;
  std::vector<float> latest_laser_ranges_;

  bool init_pose_recieved_;
  bool init_pose_set_;
  bool publish_tf_;

  double initial_pose_x_;
  double initial_pose_y_;

  double map_width_;
  double map_height_;
  uint32_t map_width_in_grids_;
  uint32_t map_height_in_grids_;
  double map_origin_x_;
  double map_origin_y_;
  float map_resolution_;
  std::vector<int8_t> map_data_;
  std::vector<double> likelihood_data_;

  double grid_linear_resolution_;
  double grid_angular_resolution_;
  int grid_width_;
  int grid_height_;
  int grid_depth_;
  unsigned long long number_of_grid_cells_;
  double rolling_window_length_;
  int rolling_window_grid_length_;

  double window_start_x_;
  double window_start_y_;
  double window_end_x_;
  double window_end_y_;
  unsigned long long window_start_index_;
  unsigned long long window_end_index_;
  std::vector<std::vector<int> > grid_locations_to_calculate_;

  std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double> > > p_kt_1_;
  std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double> > > p_bar_kt_;
  std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double> > > p_kt_;
  long double sum_of_dist_values_;

  std::vector<double> laser_pose_;
  std::vector<double> ut_;

  geometry_msgs::PoseWithCovarianceStamped curr_pose_;

  std::mutex vector_mutex_;
  int number_of_threads_;

  void laserRecived(const sensor_msgs::LaserScanConstPtr& laser_scan);
  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void requestMap();
  void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
  void computeLikelihoodField();
  void DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited);
  void initialiseDistributions();
  void getLaserPose();
  bool noMotion();
  void updateRollingWindow();
  void runGridLocalisation();
  double motionModel(double* xt, std::vector<double> &ut, double* xt_1);
  double laserModel(double* xt);
  void calculateGrid(std::promise<long double> *promObj, unsigned long long start_pose_location, unsigned long long end_pose_location);
};

std::shared_ptr<GridLocalisationNode> grid_localisation_node_ptr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;

  grid_localisation_node_ptr.reset(new GridLocalisationNode());
  ros::spin();

  return 0;
}

GridLocalisationNode::GridLocalisationNode() :
                    private_nh_("~"),
                    laser_info_recieved_(false),
                    init_pose_recieved_(false)
{
  private_nh_.param("initial_pose_set", init_pose_set_, true);
  private_nh_.param("initial_pose_x", initial_pose_x_, 1.0);
  private_nh_.param("initial_pose_y", initial_pose_y_, 1.0);

  private_nh_.param("tf_broadcast", publish_tf_, true);

  private_nh_.param("odom_alpha1", alpha1_, 0.01);
  private_nh_.param("odom_alpha2", alpha2_, 0.5);
  private_nh_.param("odom_alpha3", alpha3_, 0.5);
  private_nh_.param("odom_alpha4", alpha4_, 0.01);
  private_nh_.param("odom_alpha5", alpha4_, 0.01);

  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_random", z_random_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("number_of_beams_to_use", number_of_beams_to_use_, 150);

  private_nh_.param("grid_linear_resolution", grid_linear_resolution_, 0.1);
  private_nh_.param("grid_angular_resolution", grid_angular_resolution_, 0.125);
  private_nh_.param("rolling_window_length", rolling_window_length_, 0.8);

  private_nh_.param("scan_topic", scan_topic_, std::string("base_scan"));
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("laser_frame_id", laser_frame_id_, std::string("base_laser_front_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  private_nh_.param("number_of_threads", number_of_threads_, 4);

  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("grid_pose", 1);

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);
  laser_scan_filter_->registerCallback(boost::bind(&GridLocalisationNode::laserRecived, this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &GridLocalisationNode::initialPoseReceived, this);

  requestMap();
  computeLikelihoodField();

  grid_width_ = std::floor(map_width_/grid_linear_resolution_);
  grid_height_ = std::floor(map_height_/grid_linear_resolution_);
  grid_depth_ = std::floor((2*M_PI)/grid_angular_resolution_);
  number_of_grid_cells_ = grid_width_*grid_height_*grid_depth_;
  rolling_window_grid_length_ = std::floor(rolling_window_length_/grid_linear_resolution_);

  ROS_INFO("Grid width: %d", grid_width_);
  ROS_INFO("Grid height: %d", grid_height_);
  ROS_INFO("Grid depth: %d", grid_depth_);

  initialiseDistributions();

  getLaserPose();

  // Initilaise current pose
  curr_pose_.pose.pose.position.x = initial_pose_x_;
  curr_pose_.pose.pose.position.y = initial_pose_y_;
  curr_pose_.pose.pose.position.z = 0.0;
  curr_pose_.pose.pose.orientation.x = curr_pose_.pose.pose.orientation.y = curr_pose_.pose.pose.orientation.z = 0.0;
  curr_pose_.pose.pose.orientation.w = 1.0;
  curr_pose_.header.frame_id = global_frame_id_;

  if(init_pose_set_)
  {
    int col = floor(float(initial_pose_x_ - map_origin_x_)/grid_linear_resolution_);
    int row = floor(float(initial_pose_y_ - map_origin_y_)/grid_linear_resolution_);
    p_kt_1_[row][col][0] = 1.0;
  }

  while(ros::ok())
  {
    ros::spinOnce();

    // Wait for initial pose
    if(!init_pose_recieved_ && !init_pose_set_ || !laser_info_recieved_)
      continue;

    // ROS_INFO("ROUND STARTED!");
    updateRollingWindow();
    runGridLocalisation();
    pose_pub_.publish(curr_pose_);
    prev_odom_to_base_tf_ = curr_odom_to_base_tf_;
    // ROS_INFO("ROUND ENDED");
  }
}

GridLocalisationNode::~GridLocalisationNode()
{
  delete tfb_;
  delete tf_;
  delete laser_scan_sub_;
  delete laser_scan_filter_;
}

void GridLocalisationNode::laserRecived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  if(!laser_info_recieved_)
  {
    number_of_beams_ = laser_scan->ranges.size();
    laser_max_range_ = laser_scan->range_max;
    laser_min_range_ = laser_scan->range_min;
    laser_min_angle_ = laser_scan->angle_min;
    laser_angle_increment_ = laser_scan->angle_increment;

    tf_->waitForTransform(odom_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(10.0));
    tf_->lookupTransform(odom_frame_id_, base_frame_id_, ros::Time(0), prev_odom_to_base_tf_);

    laser_info_recieved_ = true;
  }

  latest_laser_ranges_ = laser_scan->ranges;

  try
  {
    tf_->lookupTransform(odom_frame_id_, base_frame_id_, laser_scan->header.stamp, curr_odom_to_base_tf_);
    tf::Quaternion curr_pose_quaternion_angle;
    tf::quaternionMsgToTF(curr_pose_.pose.pose.orientation, curr_pose_quaternion_angle);
    tf::Transform map_to_base_tf(curr_pose_quaternion_angle, tf::Vector3(curr_pose_.pose.pose.position.x, curr_pose_.pose.pose.position.y, 0.0));
    tf::Stamped<tf::Pose> base_to_map_pose_stamped(map_to_base_tf.inverse(),
                                                 laser_scan->header.stamp,
                                                 base_frame_id_);
    tf::Stamped<tf::Pose> odom_to_map_pose_stamped;
    tf_->transformPose(odom_frame_id_, base_to_map_pose_stamped, odom_to_map_pose_stamped);
    tf::Transform odom_to_map_tf = tf::Transform(tf::Quaternion(odom_to_map_pose_stamped.getRotation()),
                                                 tf::Point(odom_to_map_pose_stamped.getOrigin()));
    if(publish_tf_)
      tfb_->sendTransform(tf::StampedTransform(odom_to_map_tf.inverse(), laser_scan->header.stamp, global_frame_id_, odom_frame_id_));
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  ut_ = {prev_odom_to_base_tf_.getOrigin().getX(), prev_odom_to_base_tf_.getOrigin().getY(), tf::getYaw(prev_odom_to_base_tf_.getRotation()),
         curr_odom_to_base_tf_.getOrigin().getX(), curr_odom_to_base_tf_.getOrigin().getY(), tf::getYaw(curr_odom_to_base_tf_.getRotation())};
}

void GridLocalisationNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_INFO("Initial pose: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
  int col = floor(float(msg->pose.pose.position.x - map_origin_x_)/grid_linear_resolution_);
  int row = floor(float(msg->pose.pose.position.y - map_origin_y_)/grid_linear_resolution_);
  // The orientation is given in the range [-pi, pi], so it is shifted to [0, 2pi]
  // for ease of conversion into grid coordinates
  float temp_d = tf::getYaw(msg->pose.pose.orientation);
  if(temp_d < 0)
    temp_d += 2*M_PI;
  temp_d /= grid_angular_resolution_;
  int dep = floor(temp_d);
  p_kt_1_[row][col][dep] = 1.0;

  curr_pose_.pose.pose = msg->pose.pose;
  curr_pose_.header = msg->header;
  init_pose_recieved_ = true;
}


void GridLocalisationNode::requestMap()
{
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Waiting for map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  ROS_INFO("Map received!");
  
  handleMapMessage(resp.map);
}

void GridLocalisationNode::handleMapMessage(const nav_msgs::OccupancyGrid &msg)
{
  map_width_in_grids_ = msg.info.width;
  map_width_ = msg.info.resolution*map_width_in_grids_;
  map_height_in_grids_ = msg.info.height;
  map_height_ = msg.info.resolution*map_height_in_grids_;
  map_origin_x_ = msg.info.origin.position.x;
  map_origin_y_ = msg.info.origin.position.y;
  map_resolution_ = msg.info.resolution;
  map_data_ = msg.data;
  likelihood_data_.assign(map_data_.size(), 0.0);
}

void GridLocalisationNode::computeLikelihoodField()
{
  ROS_INFO("Calculating likelihood field...");

  std::vector<int> occupied_cells;

  for(unsigned long long i = 0; i < map_width_in_grids_*map_height_in_grids_; i++)
  {
    if(map_data_[i]==100)
    {
      likelihood_data_[i] = 0;
      occupied_cells.push_back(i);
    }
    else
      likelihood_data_[i] = 100;
  }

  for(auto i : occupied_cells)
  {
    std::vector<bool> visited(map_width_in_grids_*map_height_in_grids_, false);
    DFS(i, i, visited);
  }

  for(auto i=0; i < map_width_in_grids_*map_height_in_grids_; i++)
		likelihood_data_[i] = (1.0/(sqrt(2*M_PI)*sigma_hit_))*exp(-0.5*((likelihood_data_[i]*likelihood_data_[i])/(sigma_hit_*sigma_hit_)));

  ROS_INFO("Completed likelihood field calculation");
}

void GridLocalisationNode::DFS(const int &index_curr, const int &index_of_obstacle, std::vector<bool> &visited)
{
	visited[index_curr] = true;
  int* coord = ind2map(index_curr, map_width_in_grids_);
	std::pair<uint32_t, uint32_t> coord_curr;
  coord_curr.first = coord[0];
  coord_curr.second = coord[1];

	// This cell is NOT an obstacle
	if(likelihood_data_[index_curr]!=0.0)	
	{
		double distance_to_obstacle = measure_distance(index_curr, index_of_obstacle, map_width_in_grids_, map_resolution_);

		// Getting far from the obstacle
		if(distance_to_obstacle >= 2.0)
			return;

		// Found a closer obstacle
		if(distance_to_obstacle < likelihood_data_[index_curr])
			likelihood_data_[index_curr] = distance_to_obstacle;
	}

	// left
	if(coord_curr.first > 0)
	{
		int left_cell_index =  map2ind(coord_curr.first-1, coord_curr.second, map_width_in_grids_);
    if(!visited[left_cell_index])
			DFS(left_cell_index, index_of_obstacle, visited);
	}

	// right
	if(coord_curr.first < map_width_in_grids_-1)
	{
		int right_cell_index =  map2ind(coord_curr.first+1, coord_curr.second, map_width_in_grids_);
		if(!visited[right_cell_index])
			DFS(right_cell_index, index_of_obstacle, visited);
	}

	// up
	if(coord_curr.second > 0)
	{
		int up_cell_index =  map2ind(coord_curr.first, coord_curr.second-1, map_width_in_grids_);
		if(!visited[up_cell_index])
			DFS(up_cell_index, index_of_obstacle, visited);
	}

	// down
	if(coord_curr.second < map_height_in_grids_-1)
	{
		int down_cell_index =  map2ind(coord_curr.first, coord_curr.second+1, map_width_in_grids_);
		if(!visited[down_cell_index])
			DFS(down_cell_index, index_of_obstacle, visited);
	}
}


void GridLocalisationNode::initialiseDistributions()
{
  ROS_INFO("Initialising distributions...");
  ROS_INFO("Initialised!");
}

void GridLocalisationNode::getLaserPose()
{
  tf::StampedTransform base_to_laser_tf;
  tf_->waitForTransform(base_frame_id_, laser_frame_id_, ros::Time(0), ros::Duration(10.0));
  tf_->lookupTransform(base_frame_id_, laser_frame_id_, ros::Time(0), base_to_laser_tf);
  laser_pose_ = {base_to_laser_tf.getOrigin().getX(), base_to_laser_tf.getOrigin().getY(), tf::getYaw(base_to_laser_tf.getRotation())};
}

bool GridLocalisationNode::noMotion()
{
  double x = prev_odom_to_base_tf_.getOrigin().getX();
  double y = prev_odom_to_base_tf_.getOrigin().getY();
  double theta = tf::getYaw(prev_odom_to_base_tf_.getRotation());

  double x_prime = curr_odom_to_base_tf_.getOrigin().getX();
  double y_prime = curr_odom_to_base_tf_.getOrigin().getY();
  double theta_prime = tf::getYaw(curr_odom_to_base_tf_.getRotation());

  double linear_tolerance = grid_linear_resolution_;

  // moving diagonally
  if( ((theta_prime>0.087) && (theta_prime<1.484)) ||
      ((theta_prime>1.658) && (theta_prime<2.055)) ||
      ((theta_prime>-3.055) && (theta_prime<-1.658)) ||
      ((theta_prime>-1.484) && (theta_prime<-0.087)) )
      {
        linear_tolerance = sqrt(2*grid_linear_resolution_*grid_linear_resolution_);
      }
  
  if(sqrt((pow(x-x_prime, 2))+pow(y-y_prime, 2)) > linear_tolerance)
    return false;

  return true;
}

void GridLocalisationNode::updateRollingWindow()
{
  window_start_x_ = curr_pose_.pose.pose.position.x - (rolling_window_length_/2);
  window_start_y_ = curr_pose_.pose.pose.position.y - (rolling_window_length_/2);

  if(window_start_x_ < map_origin_x_)
    window_start_x_ = map_origin_x_;
  
  if(window_start_y_ < map_origin_y_)
    window_start_y_ = map_origin_y_;
  
  window_start_x_ -= map_origin_x_;
  window_start_x_ /= grid_linear_resolution_;

  window_start_y_ -= map_origin_y_;
  window_start_y_ /= grid_linear_resolution_;

  window_start_index_ = sub2ind(int(window_start_y_), int(window_start_x_), 0, grid_width_, grid_depth_);

  window_end_x_ = curr_pose_.pose.pose.position.x + (rolling_window_length_/2);
  window_end_y_ = curr_pose_.pose.pose.position.y + (rolling_window_length_/2);

  window_end_x_ -= map_origin_x_;
  window_end_x_ /= grid_linear_resolution_;

  window_end_y_ -= map_origin_y_;
  window_end_y_ /= grid_linear_resolution_; 

  window_end_index_ = sub2ind(int(window_end_y_), int(window_end_x_), 0, grid_width_, grid_depth_);
  if(window_end_index_ > number_of_grid_cells_)
    window_end_index_ = number_of_grid_cells_;

  unsigned long long counter = 0;
  for(unsigned long long i = window_start_index_; i < window_end_index_; i++)
  {
    if(counter == rolling_window_grid_length_*grid_depth_)
    {
      i += (grid_width_ - rolling_window_grid_length_)*grid_depth_ - 1;
      counter = 0;
      continue;
    }
    counter ++;

    int row, col, dep;
    ind2sub(i, grid_height_, grid_width_, grid_depth_, &row, &col, &dep);
    grid_locations_to_calculate_.push_back({col, row, dep});
  }
}

void GridLocalisationNode::runGridLocalisation()
{
  if(!laser_info_recieved_)
    return; 

  unsigned long long number_of_locations_per_thread = grid_locations_to_calculate_.size()/number_of_threads_;
  unsigned long long number_of_leftover_locations = grid_locations_to_calculate_.size()%number_of_threads_;

  std::vector<std::promise<long double>> promises(number_of_threads_);
  std::vector<std::future<long double>> futures;
  for(int i=0; i<number_of_threads_; i++)
    futures.push_back(promises[i].get_future()); 
  
  std::vector<std::thread> threads;
  for(int i=0; i<number_of_threads_-1; i++)
  {
    threads.emplace_back(&GridLocalisationNode::calculateGrid, this, &promises[i], i*number_of_locations_per_thread, (i+1)*number_of_locations_per_thread);
  }
  
  threads.emplace_back(&GridLocalisationNode::calculateGrid, this, &promises[number_of_threads_-1], (number_of_threads_-1)*number_of_locations_per_thread, number_of_threads_*number_of_locations_per_thread + number_of_leftover_locations);

  sum_of_dist_values_ = 0.0;
  for(int i=0; i<number_of_threads_; i++)
    sum_of_dist_values_ += futures[i].get();

  for(std::thread &t : threads)
    t.join();
  
  promises.clear();
  futures.clear();
  threads.clear();


  int max_row=0, max_col=0, max_dep=0;
  double max_prob = 0.0;
  for(unsigned long long i = 0; i < grid_locations_to_calculate_.size(); i++)
  {
    int col = grid_locations_to_calculate_[i][0];
    int row = grid_locations_to_calculate_[i][1];
    int dep = grid_locations_to_calculate_[i][2];

    p_kt_[row][col][dep] /= sum_of_dist_values_;
    p_kt_1_[row][col][dep] = p_kt_[row][col][dep];
    p_bar_kt_[row][col][dep] = 0.0;
    if(p_kt_[row][col][dep] > max_prob)
    {
      max_prob = p_kt_[row][col][dep];
      max_row = row;
      max_col = col;
      max_dep = dep;
    }
  }
  grid_locations_to_calculate_.clear();

  curr_pose_.header.stamp = ros::Time::now();
  curr_pose_.header.frame_id = global_frame_id_;
  curr_pose_.pose.pose.position.x = max_col*grid_linear_resolution_ + map_origin_x_;
  curr_pose_.pose.pose.position.y = max_row*grid_linear_resolution_ + map_origin_y_;

  double euler_angle = max_dep*grid_angular_resolution_;
  if(euler_angle > M_PI)
    euler_angle -= 2*M_PI;
  
  tf::Quaternion quaternion_angle;
  quaternion_angle.setEulerZYX(euler_angle, 0.0, 0.0);
  
  curr_pose_.pose.pose.orientation.x = quaternion_angle.getX();
  curr_pose_.pose.pose.orientation.y = quaternion_angle.getY();
  curr_pose_.pose.pose.orientation.z = quaternion_angle.getZ();
  curr_pose_.pose.pose.orientation.w = quaternion_angle.getW();
}

double GridLocalisationNode::motionModel(double* xt, std::vector<double> &ut, double* xt_1)
{
  double x_prime = xt[0];
  double y_prime = xt[1];
  double theta_prime = xt[2];

  double x_bar = ut[0];
  double y_bar = ut[1];
  double theta_bar = ut[2];

  double x_bar_prime = ut[3];
  double y_bar_prime = ut[4];
  double theta_bar_prime = ut[5];

  double x = xt_1[0];
  double y = xt_1[1];
  double theta = xt_1[2];

  double delta_trans =  sqrt((x_bar - x_bar_prime)*(x_bar - x_bar_prime) + (y_bar - y_bar_prime)*(y_bar - y_bar_prime));
  double delta_rot = theta_bar_prime - theta_bar;
  
  double delta_trans_hat = sqrt((x-x_prime)*(x-x_prime) + (y-y_prime)*(y-y_prime));
  double delta_rot_hat =  theta_prime - theta;

  double a, b, p1, p2, p3;
  a = angle_diff(delta_rot, delta_rot_hat);
  b = sqrt(alpha4_*delta_rot_hat*delta_rot_hat + alpha2_*delta_trans_hat*delta_trans_hat);
  p1 = prob(a, b);

  a = delta_trans-delta_trans_hat;
  b = sqrt(alpha3_*delta_trans_hat*delta_trans_hat + alpha1_*delta_rot_hat*delta_rot_hat);
  p2 = prob(a, b);
  
  a = 0.0;
  b = sqrt(alpha1_*delta_rot_hat*delta_rot_hat + alpha5_*delta_trans_hat*delta_trans_hat);
  p3 = prob(a, b);

  // std comes out to be zero when turning on the spot
  if(std::isnan(p1*p2*p3))
  {
    if(noMotion())  // if there has been no motion, then we are indeed turning on the spot
      return 1.0;
    else
      return 0.0;   // otherwise this hypothesis is not valid
  }

  return p1*p2*p3;  
}

double GridLocalisationNode::laserModel(double* xt)
{
  double q = 1.0;
  double z_max = laser_max_range_, z_rand_max = z_random_/z_max;
  double min_x = map_origin_x_;
  double max_x = map_origin_x_ + map_width_;
  double min_y = map_origin_y_;
  double max_y = map_origin_y_ + map_height_;

  int beam_increment = number_of_beams_/number_of_beams_to_use_;

  for(int i = 0; i < number_of_beams_to_use_; i++) // check 30 laser rays
  {
    // If the reading is above the maximum or below the minimum range of the LiDAR, discard it
    if((latest_laser_ranges_[i*beam_increment] >= laser_max_range_) || (latest_laser_ranges_[i*beam_increment] <= laser_min_range_))
      continue;

    // Calculate the angle of the beam and bound it to [-pi, pi]
    float beam_angle = laser_min_angle_ + laser_angle_increment_*i*beam_increment;
    if(beam_angle < 0)
      beam_angle += 2*M_PI;
      
    // Project the beam end-point onto the map
    float x_zkt = xt[0] + laser_pose_[0]*cos(xt[2]) - laser_pose_[1]*sin(xt[2]) + latest_laser_ranges_[i*beam_increment]*cos(xt[2] + beam_angle); // assume that the sensor is not mounted at angle
    float y_zkt = xt[1] + laser_pose_[1]*cos(xt[2]) + laser_pose_[0]*sin(xt[2]) + latest_laser_ranges_[i*beam_increment]*sin(xt[2] + beam_angle); // assume that the sensor is not mounted at angle

    // temproary fix - discard readings that are out of bound
    if(x_zkt < min_x || x_zkt > max_x || y_zkt < min_y || y_zkt > max_y) // THESE VALUES NEED TO BE CHANGED!
      continue;

    
    // Project the points onto the likelihood field by subreacting the origin and dividing by the map resoluiton
    x_zkt -= map_origin_x_;
    y_zkt -= map_origin_y_;

    x_zkt /= map_resolution_;
    y_zkt /= map_resolution_;
    
    // Get the relevant point from the likelhood field
    int index = map2ind(int(x_zkt), int(y_zkt), map_width_in_grids_);
    float dist = likelihood_data_[index];

    q *= (z_hit_*prob(dist, sigma_hit_) + z_rand_max);
  }

  return q;
}

void GridLocalisationNode::calculateGrid(std::promise<long double> *promObj, unsigned long long start_pose_location, unsigned long long end_pose_location)
{
  std::lock_guard<std::mutex> lock(vector_mutex_);

  long double sum_of_values = 0.0;
  for(unsigned long long k = start_pose_location; k < end_pose_location; k++)
  {
    double xt[3];
    int colk = grid_locations_to_calculate_[k][0];
    int rowk = grid_locations_to_calculate_[k][1];
    int depk = grid_locations_to_calculate_[k][2];

    xt[0] = colk*grid_linear_resolution_ + map_origin_x_;
    xt[1] = rowk*grid_linear_resolution_ + map_origin_y_;
    xt[2] = depk*grid_angular_resolution_;
    if(xt[2] > M_PI)
      xt[2] -= 2*M_PI;
    
    for(unsigned long long i = start_pose_location; i < end_pose_location; i++)
    {
      double xt_1[3];
      int coli = grid_locations_to_calculate_[i][0];
      int rowi = grid_locations_to_calculate_[i][1];
      int depi = grid_locations_to_calculate_[i][2];
      
      xt_1[0] = coli*grid_linear_resolution_ + map_origin_x_;
      xt_1[1] = rowi*grid_linear_resolution_ + map_origin_y_;
      xt_1[2] = depi*grid_angular_resolution_;
      if(xt_1[2] > M_PI)
        xt_1[2] -= 2*M_PI;

      p_bar_kt_[rowk][colk][depk] += p_kt_1_[rowi][coli][depi]*motionModel(xt, ut_, xt_1);
    }

    p_kt_[rowk][colk][depk] = p_bar_kt_[rowk][colk][depk];//*laserModel(xt);
    sum_of_values += p_kt_[rowk][colk][depk];
  }
  promObj->set_value(sum_of_values);
}