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
    void set_map(const nav_msgs::OccupancyGrid &map);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;
    
    bool map_recieved_{false};
    double map_origin_x_;
    double map_origin_y_;
    double map_resolution_;

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_bar_k_t_;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_t_;
    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>> p_t_1_;

    double grid_linear_resolution_;
    double grid_angular_resolution_;

    int number_of_grid_rows_;
    int number_of_grid_cols_;
    int number_of_grid_layers_;
    
    bool starting_point_set_;
    double starting_x_;
    double starting_y_;
    double starting_theta_;

    geometry_msgs::PoseWithCovarianceStamped curr_pose_;
    geometry_msgs::TransformStamped prev_odom_;
    std::vector<std::pair<int, int>> map_cells_in_grid_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATION */
