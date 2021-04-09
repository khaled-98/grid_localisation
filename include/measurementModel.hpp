#ifndef SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL
#define SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <unordered_map>

class MeasurementModel
{
public:
    MeasurementModel();
    double getScanProbability(const geometry_msgs::Pose &curr_pose, const sensor_msgs::LaserScanConstPtr &scan);
    void setMap(const nav_msgs::OccupancyGrid &map);
    void setLaserPose(const geometry_msgs::TransformStamped &laser_pose);

private:
    void computeLikelihoodField(const nav_msgs::OccupancyGrid &map);
    void DFS(const int & index_curr, const int & index_of_obstacle, std::vector<bool> & visited);

	geometry_msgs::TransformStamped laser_pose_;
    nav_msgs::OccupancyGrid map_;
    std::unordered_map<int, double> likelihood_field_dist_;
    int max_number_of_beams_;       // The maximum number of laser beams to use in the calculations 
	double max_likelihood_distance_; // The distance beyond which the likelihood is 0

    double z_hit_;
    double sigma_hit_;
    double z_rand_;
    double z_max_;

    ros::NodeHandle private_nh_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL */
