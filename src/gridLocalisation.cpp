#include "../include/gridLocalisation.hpp"

GridLocalisation::GridLocalisation(const ros::NodeHandle &nh,
                                   const std::shared_ptr<MotionModel> &motion_model,
                                   const std::shared_ptr<MeasurementModel> &measurement_model)
: nh_(nh),
  motion_model_(motion_model), 
  measurement_model_(measurement_model)
{
    nh.param("grid_linear_resolution", grid_linear_resolution_, 0.1);
    nh.param("grid_angular_resolution", grid_angular_resolution_, 0.2);

    nh.param("starting_point_set", starting_point_set_, false);
    nh.param("starting_x", starting_x_, 0.0);
    nh.param("starting_y", starting_y_, 0.0);
    nh.param("starting_theta", starting_theta_, 0.0);
}

void GridLocalisation::set_map(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
    map_recieved_ = true;
    // measurement_model->compute_likelihood_field_prob(map)
}

geometry_msgs::PoseWithCovarianceStamped GridLocalisation::localise(const sensor_msgs::LaserScanConstPtr &scan)
{
    // Do not attempt to localise unless you have a map and you know where you are
    if(!map_recieved_ || !starting_point_set_)
        return curr_pose_;

    
}