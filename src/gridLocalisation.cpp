#include "../include/gridLocalisation.hpp"
#include "../include/utils.hpp"
#include <vector>

GridLocalisation::GridLocalisation(const std::shared_ptr<MotionModel> &motion_model,
                                   const std::shared_ptr<MeasurementModel> &measurement_model)
: private_nh_("~"),
  motion_model_(motion_model), 
  measurement_model_(measurement_model)
{
    private_nh_.param("grid_linear_resolution", grid_linear_resolution_, 0.1);
    private_nh_.param("grid_angular_resolution", grid_angular_resolution_, 0.2);

    private_nh_.param("starting_point_set", starting_point_set_, false);
    private_nh_.param("starting_x", starting_x_, 0.0);
    private_nh_.param("starting_y", starting_y_, 0.0);
    private_nh_.param("starting_theta", starting_theta_, 0.0);

    // if(starting_point_set_) // initialise distrubition
    // {

    // }
}

void GridLocalisation::set_map(const nav_msgs::OccupancyGrid &map)
{
    map_recieved_ = true;
    // measurement_model->compute_likelihood_field_prob(map)

    number_of_grid_rows_ = (map.info.height*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_cols_ = (map.info.width*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_layers_ = (2*M_PI)/grid_angular_resolution_;

    int rows_to_skip = map.info.height/number_of_grid_rows_;
    int cols_to_skip = map.info.width/number_of_grid_cols_;

    ROS_WARN("%d, %d", cols_to_skip, map.info.width);

    for(int row=0; row<map.info.height; row+=rows_to_skip)
    {
        for(int col=0; col<map.info.width; col+=cols_to_skip)
        {
            int index = Utils::map_to_index(row, col, map.info.width);
            index_of_map_cells_in_grid_.push_back(index);
        }
    }
}

geometry_msgs::PoseWithCovarianceStamped GridLocalisation::localise(const sensor_msgs::LaserScanConstPtr &scan,
                                                                    const geometry_msgs::TransformStamped &curr_odom)
{
    // Do not attempt to localise unless you have a map and you know where you are
    if(!map_recieved_ || !starting_point_set_)
        return curr_pose_;

    // We haven't gone anywhere!
    if(!Utils::has_moved(prev_odom_, curr_odom, grid_linear_resolution_, grid_angular_resolution_))
        return curr_pose_;

    // Run through the grid here
    
    prev_odom_ = curr_odom;
    return curr_pose_;
}