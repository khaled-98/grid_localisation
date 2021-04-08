#include "../include/gridLocalisation.hpp"
#include "../include/utils.hpp"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <vector>
#include "tf2/utils.h"

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

    private_nh_.param("visualisation", visualisation_flag_, false);
    if(visualisation_flag_)
        visual_pub_ = nh_.advertise<visualization_msgs::Marker>("probability_visualisation", 0);
}

void GridLocalisation::set_map(const nav_msgs::OccupancyGrid &map)
{
    map_recieved_ = true;
    map_origin_x_ = map.info.origin.position.x;
    map_origin_y_ = map.info.origin.position.y;
    map_resolution_ = map.info.resolution;
    measurement_model_->setMap(map);

    number_of_grid_rows_ = (map.info.height*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_cols_ = (map.info.width*map.info.resolution)/grid_linear_resolution_;
    number_of_grid_layers_ = (2*M_PI)/grid_angular_resolution_;

    // Initialise distribution
    if(starting_point_set_)
    {
        int initial_row = starting_y_/grid_linear_resolution_;
        int initial_col = starting_x_ /grid_linear_resolution_;
        if(starting_theta_ < 0.0)
            starting_theta_ += 2*M_PI; // keep it in the 0 to 2pi range
        int initial_layer = starting_theta_/grid_angular_resolution_;

        ROS_INFO("Initial col: %d", initial_col);
        ROS_INFO("Initial row: %d", initial_row);

        p_t_1_[initial_row][initial_col][initial_layer] = 1.0;
        curr_pose_.pose.pose.position.x = starting_x_;
        curr_pose_.pose.pose.position.y = starting_y_;
        curr_pose_.pose.pose.orientation = Utils::getQuat(starting_theta_);
        prev_odom_.transform.translation.x = starting_x_;
        prev_odom_.transform.translation.y = starting_y_;
        prev_odom_.transform.rotation = Utils::getQuat(starting_theta_);
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

    double ut_1[3];
    ut_1[0] = prev_odom_.transform.translation.x - map_origin_x_;
    ut_1[1] = prev_odom_.transform.translation.y - map_origin_y_;
    ut_1[2] = Utils::get_angle_from_quat(prev_odom_.transform.rotation);

    double ut[3];
    ut[0] = curr_odom.transform.translation.x - map_origin_x_;
    ut[1] = curr_odom.transform.translation.y - map_origin_y_;
    ut[2] = Utils::get_angle_from_quat(curr_odom.transform.rotation);

    // Run through the grid here
    double sum_of_dist_values = 0.0;
    for(int grid_row=0; grid_row<number_of_grid_rows_; grid_row++)
    {
        for(int grid_col=0; grid_col<number_of_grid_cols_; grid_col++)
        {
            for(int grid_layer=0; grid_layer<number_of_grid_layers_; grid_layer++)
            {
                double xt[3];
                xt[0] = grid_col*grid_linear_resolution_;
                xt[1] = grid_row*grid_linear_resolution_;
                xt[2] = Utils::angle_diff(grid_layer*grid_angular_resolution_, 0);

                for(int grid_row_2=0; grid_row_2<number_of_grid_rows_; grid_row_2++)
                {
                    for(int grid_col_2=0; grid_col_2<number_of_grid_cols_; grid_col_2++)
                    {
                        for(int grid_layer_2=0; grid_layer_2<number_of_grid_layers_; grid_layer_2++)
                        {
                            if(grid_row==grid_row_2 && grid_col==grid_col_2 && grid_layer==grid_layer_2) // The algorthim only runs when there's movement so this is no point in this.
                                continue;

                            double xt_1[3];
                            xt_1[0] = grid_col_2*grid_linear_resolution_;
                            xt_1[1] = grid_row_2*grid_linear_resolution_;
                            xt_1[2] = Utils::angle_diff(grid_layer_2*grid_angular_resolution_, 0);

                            p_bar_k_t_[grid_row][grid_col][grid_layer] += p_t_1_[grid_row_2][grid_col_2][grid_layer_2]*motion_model_->run(xt, xt_1, ut, ut_1);;
                        }
                    }
                }

                geometry_msgs::TransformStamped curr_pose;
                curr_pose.transform.translation.x = xt[0];
                curr_pose.transform.translation.y = xt[1];

                tf2::Quaternion tf_quat;
                tf_quat.setRPY(0.0, 0.0, xt[2]);
                tf2::convert(tf_quat, curr_pose.transform.rotation);
                p_t_[grid_row][grid_col][grid_layer] = p_bar_k_t_[grid_row][grid_col][grid_layer] *
                                                       measurement_model_->getScanProbability(curr_pose, scan);
                sum_of_dist_values += p_t_[grid_row][grid_col][grid_layer];
            }
        }
    }

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;

    // Normalise the probability and calculate the max
    int max_row = 0;
    int max_col = 0;
    int max_layer = 0;
    double max_prob = 0.0;

    for(int grid_row=0; grid_row<number_of_grid_rows_; grid_row++)
    {
        for(int grid_col=0; grid_col<number_of_grid_cols_; grid_col++)
        {
            for(int grid_layer=0; grid_layer<number_of_grid_layers_; grid_layer++)
            {
                p_t_[grid_row][grid_col][grid_layer] /= sum_of_dist_values;
                if(p_t_[grid_row][grid_col][grid_layer] > max_prob)
                {
                    max_prob = p_t_[grid_row][grid_col][grid_layer];
                    max_row = grid_row;
                    max_col = grid_col;
                    max_layer = grid_layer;
                }
                p_t_1_[grid_row][grid_col][grid_layer] = p_t_[grid_row][grid_col][grid_layer];
                p_bar_k_t_[grid_row][grid_col][grid_layer] = 0.0;
                p_t_[grid_row][grid_col][grid_layer] = 0.0;

                if(visualisation_flag_)
                {
                    geometry_msgs::Point point;
                    point.x = grid_col*grid_linear_resolution_;
                    point.y = grid_row*grid_linear_resolution_;
                    point.z = grid_layer*grid_angular_resolution_*0.5;
                    points.push_back(point);

                    std_msgs::ColorRGBA color;
                    color.a = p_t_1_[grid_row][grid_col][grid_layer];
                    color.r = 1.0;
                    colors.push_back(color);
                }    
            }
        }
    }

    if(visualisation_flag_)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.points = points;
        marker.colors = colors;

        visual_pub_.publish(marker);
    }

    if(max_prob != 0.0)
    {
        curr_pose_.pose.pose.position.x = max_col*grid_linear_resolution_;
        curr_pose_.pose.pose.position.y = max_row*grid_linear_resolution_;
        curr_pose_.pose.pose.orientation = Utils::getQuat(max_layer*grid_angular_resolution_); // Is the domain of the angle correct?
    }

    prev_odom_ = curr_odom;
    return curr_pose_;
}