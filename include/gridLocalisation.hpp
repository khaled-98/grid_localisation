#ifndef SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION
#define SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION

#include "ros/ros.h"
#include "../include/motionModel.hpp"
#include "../include/measurementModel.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <unordered_map>

class GridLocalisation
{
public:
    GridLocalisation(const ros::NodeHandle &nh,
                     const std::shared_ptr<MotionModel> &motion_model,
                     const std::shared_ptr<MeasurementModel> &measurement_model);
    geometry_msgs::PoseWithCovarianceStamped localise(const sensor_msgs::LaserScanConstPtr &scan);
    void set_map(const nav_msgs::OccupancyGrid &map);
private:
    ros::NodeHandle nh_;
    
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;
    
    bool map_recieved_{false};
    nav_msgs::OccupancyGrid map_;

    std::unordered_map<int, std::unordered_map<int, double>> p_bar_k_t_;
    std::unordered_map<int, std::unordered_map<int, double>> p_k_t_;
    std::unordered_map<int, std::unordered_map<int, double>> p_k_t_1_;

    double grid_linear_resolution_;
    double grid_angular_resolution_;
    
    bool starting_point_set_;
    double starting_x_;
    double starting_y_;
    double starting_theta_;

    geometry_msgs::PoseWithCovarianceStamped curr_pose_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION */
