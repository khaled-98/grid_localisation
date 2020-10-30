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

}

void GridLocalisation::set_map(const nav_msgs::OccupancyGrid &map)
{
    map_recieved_ = true;
    map_origin_x_ = map.info.origin.position.x;
    map_origin_y_ = map.info.origin.position.y;
    map_resolution_ = map.info.resolution;
    // measurement_model->compute_likelihood_field_prob(map)

    number_of_grid_rows_ = (map.info.height*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_cols_ = (map.info.width*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_layers_ = (2*M_PI)/grid_angular_resolution_;

    int rows_to_skip = map.info.height/number_of_grid_rows_;
    int cols_to_skip = map.info.width/number_of_grid_cols_;

    // Initialise distribution
    if(starting_point_set_)
    {
        int initial_col = starting_x_/map_resolution_;
        int initial_row = starting_y_/map_resolution_;

        int smallest_difference_found = INT_MAX;
        int number_found;
        for(int i=1; cols_to_skip*(i-1) < initial_col; i++)
        {
            if(abs(cols_to_skip*i-initial_col) < smallest_difference_found)
            {
                smallest_difference_found = abs(cols_to_skip*i-initial_col);
                number_found = cols_to_skip*i;
            } 
        }
        initial_col = number_found;

        smallest_difference_found = INT_MAX;
        for(int i=1; rows_to_skip*(i-1) < initial_row; i++)
        {
            if(abs(rows_to_skip*i-initial_row) < smallest_difference_found)
            {
                smallest_difference_found = abs(rows_to_skip*i-initial_row);
                number_found = rows_to_skip*i;
            }
        }
        initial_row = number_found;

        ROS_INFO("Initial col: %d", initial_col);
        ROS_INFO("Initial row: %d", initial_row);

        p_t_1_[initial_row][initial_col][0.0] = 1.0;
        curr_pose_.pose.pose.position.x = starting_x_;
        curr_pose_.pose.pose.position.y = starting_y_;
        curr_pose_.pose.pose.orientation = Utils::getQuat(starting_theta_);
        prev_odom_.transform.translation.x = starting_x_;
        prev_odom_.transform.translation.y = starting_y_;
        prev_odom_.transform.rotation = Utils::getQuat(starting_theta_);
    }

    // Divide up the grid - this probably needs to be moved somewhere else
    for(int row=0; row<map.info.height; row+=rows_to_skip)
        for(int col=0; col<map.info.width; col+=cols_to_skip)
            map_cells_in_grid_.push_back(std::make_pair(row, col));
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

    ROS_WARN("START");

    double ut_1[3];
    ut_1[0] = prev_odom_.transform.translation.x - map_origin_x_;
    ut_1[1] = prev_odom_.transform.translation.y - map_origin_y_;
    ut_1[2] = Utils::get_angle_from_quat(prev_odom_.transform.rotation);
    // ROS_INFO("Previous angle: %f", ut_1[2]);
    if(ut_1[2]>M_PI)
        ut_1[2] -= 2*M_PI;

    double ut[3];
    ut[0] = curr_odom.transform.translation.x - map_origin_x_;
    ut[1] = curr_odom.transform.translation.y - map_origin_y_;
    ut[2] = Utils::get_angle_from_quat(curr_odom.transform.rotation);
    if(ut[2]>M_PI)
        ut[2] -= 2*M_PI;

    // Run through the grid here
    double sum_of_dist_values = 0.0;
    for(int grid_layer=0; grid_layer<number_of_grid_layers_; grid_layer++)
    {
        for(auto grid_cell : map_cells_in_grid_)
        {
            double xt[3];
            xt[0] = grid_cell.second*map_resolution_; // x is along the cols
            xt[1] = grid_cell.first*map_resolution_;  // y is along the rows
            xt[2] = grid_layer*grid_angular_resolution_; // from 0 to 2pi
            if(xt[2]>M_PI)
                xt[2] -= 2*M_PI;

            for(int grid_layer_2=0; grid_layer_2<number_of_grid_layers_; grid_layer_2++)
            {
                for(auto grid_cell_2 : map_cells_in_grid_)
                {
                    if(grid_layer==grid_layer_2 && grid_cell==grid_cell_2)
                        continue;
                        
                    double xt_1[3];
                    xt_1[0] = grid_cell_2.second*map_resolution_; // x is along the cols
                    xt_1[1] = grid_cell_2.first*map_resolution_;  // y is along the rows
                    xt_1[2] = grid_layer_2*grid_angular_resolution_; // from 0 to 2pi
                    if(xt_1[2]>M_PI)
                        xt_1[2] -= 2*M_PI;

                    p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer] +=
                        p_t_1_[grid_cell_2.first][grid_cell_2.second][grid_layer_2]*motion_model_->run(xt, xt_1, ut, ut_1);
                    sum_of_dist_values += p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer];
                }

            }
        }
    }

    ROS_WARN("Sum: %f", sum_of_dist_values);
    
    // Normalise the probablity and calculate the max
    int max_row = 0;
    int max_col = 0;
    int max_layer = 0;
    double max_prob = 0.0;

    for(int grid_layer=0; grid_layer<number_of_grid_layers_; grid_layer++)
    {
        for(auto grid_cell : map_cells_in_grid_)
        {
            p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer] /= sum_of_dist_values;
            p_t_1_[grid_cell.first][grid_cell.second][grid_layer] = p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer];
            if(p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer] > max_prob)
            {
                max_prob = p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer];
                max_row = grid_cell.first;
                max_col = grid_cell.second;
                max_layer = grid_layer;
                ROS_INFO("In here %f: %d, %d, %d", max_prob, max_col*map_resolution_, max_row*map_resolution_, max_layer*grid_angular_resolution_);
            }
             p_bar_k_t_[grid_cell.first][grid_cell.second][grid_layer] = 0.0;
        }
    }

    if(max_prob != 0.0)
    {
        curr_pose_.pose.pose.position.x = max_col*map_resolution_;
        curr_pose_.pose.pose.position.y = max_row*map_resolution_;
        curr_pose_.pose.pose.orientation = Utils::getQuat(max_layer*grid_angular_resolution_); // Is the domain of the angle correct?
    }

    ROS_WARN("FINISH: %f, %f", curr_pose_.pose.pose.position.x, curr_pose_.pose.pose.position.y);
    prev_odom_ = curr_odom;
    return curr_pose_;
}