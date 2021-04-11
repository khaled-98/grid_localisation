#ifndef SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION
#define SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION

#include "ros/ros.h"
#include "../include/motionModel.hpp"
#include "../include/measurementModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <unordered_map>
#include <vector>
#include <utility>

class GridLocalisation
{
public:
    GridLocalisation(const std::shared_ptr<MotionModel> &motion_model,
                     const std::shared_ptr<MeasurementModel> &measurement_model);
    geometry_msgs::PoseWithCovarianceStamped localise(const sensor_msgs::LaserScanConstPtr &scan,
                                                      const geometry_msgs::TransformStamped &curr_odom);
    void setMap(const nav_msgs::OccupancyGrid &map);
private:
    double rolling_window_length_;
    std::vector<std::vector<int>> rolling_window_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;
    
    bool map_recieved_{false};
    nav_msgs::OccupancyGrid map_;

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_bar_k_t_;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_t_;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_t_1_;

    double grid_linear_resolution_;
    double grid_angular_resolution_;
    
    bool starting_point_set_;
    double starting_x_;
    double starting_y_;
    double starting_theta_;

    geometry_msgs::PoseWithCovarianceStamped curr_pose_;
    geometry_msgs::TransformStamped prev_odom_;

    bool visualisation_flag_;
    ros::Publisher prob_visual_pub_;
    ros::Publisher rolling_window_visual_pub_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION */
