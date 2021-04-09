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

    private_nh_.param("rolling_window_length", rolling_window_length_, 2.0);

    private_nh_.param("visualisation", visualisation_flag_, false);
    if(visualisation_flag_)
        visual_pub_ = nh_.advertise<visualization_msgs::Marker>("probability_visualisation", 0);
}

void GridLocalisation::setMap(const nav_msgs::OccupancyGrid &map)
{
    map_recieved_ = true;
    map_ = map;
    measurement_model_->setMap(map);

    grid_height_ = (map.info.height*map.info.resolution)/grid_linear_resolution_;
    grid_width_ = (map.info.width*map.info.resolution)/grid_linear_resolution_;
    grid_depth_ = (2*M_PI)/grid_angular_resolution_;

    // Initialise distribution
    if(starting_point_set_)
    {
        if(starting_theta_ < 0.0)
            starting_theta_ += 2*M_PI; // keep it in the 0 to 2pi range

        p_t_1_[starting_x_][starting_y_][starting_theta_] = 1.0;
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
    if(!Utils::hasMoved(prev_odom_, curr_odom, grid_linear_resolution_, grid_angular_resolution_))
        return curr_pose_;

    double map_width = map_.info.width * map_.info.resolution;  // along the x axis
    double map_height = map_.info.height * map_.info.resolution; // along the y axis

    double sum_of_dist_values = 0.0;
    for(double x_k = 0.0; x_k < map_width; x_k += grid_linear_resolution_)
    {
        for(double y_k = 0.0; y_k < map_height; y_k += grid_linear_resolution_)
        {
            for(double theta_k = 0.0; theta_k < 2*M_PI; theta_k += grid_angular_resolution_)
            {
                geometry_msgs::Pose xt;
                xt.position.x = x_k + map_.info.origin.position.x;
                xt.position.y = y_k + map_.info.origin.position.y;
                xt.orientation = Utils::getQuat(theta_k);

                for(double x_i = 0.0; x_i < map_width; x_i += grid_linear_resolution_)
                {
                    for(double y_i = 0.0; y_i < map_height; y_i += grid_linear_resolution_)
                    {
                        for(double theta_i = 0.0; theta_i < 2*M_PI; theta_i += grid_angular_resolution_)
                        {
                            geometry_msgs::Pose xt_1;
                            xt_1.position.x = x_i + map_.info.origin.position.x;
                            xt_1.position.y = y_i + map_.info.origin.position.y;
                            xt_1.orientation = Utils::getQuat(theta_i);

                            if(xt == xt_1)
                                continue;

                            p_bar_k_t_[x_k][y_k][theta_k] += p_t_1_[x_i][y_i][theta_i]*motion_model_->getTransitionProbability(xt, xt_1, curr_odom, prev_odom_);
                        }
                    }
                }
                
                p_t_[x_k][y_k][theta_k] = p_bar_k_t_[x_k][y_k][theta_k] * measurement_model_->getScanProbability(xt, scan);
                sum_of_dist_values += p_t_[x_k][y_k][theta_k];
            }
        }
    }

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;

    double max_x = 0.0;
    double max_y = 0.0;
    double max_theta = 0.0;
    double max_prob = 0.0;
    for(double x = 0.0; x < map_width; x += grid_linear_resolution_)
    {
        for(double y = 0.0; y < map_height; y += grid_linear_resolution_)
        {
            for(double theta = 0.0; theta < 2*M_PI; theta += grid_angular_resolution_)
            {
                geometry_msgs::Pose xt;
                xt.position.x = x;
                xt.position.y = y;
                xt.orientation = Utils::getQuat(theta);

                p_t_[x][y][theta] /= sum_of_dist_values;
                if(p_t_[x][y][theta] > max_prob)
                {
                    max_prob = p_t_[x][y][theta];
                    max_x = x;
                    max_y = y;
                    max_theta = theta;
                }
                p_t_1_[x][y][theta] = p_t_[x][y][theta];
                p_t_[x][y][theta] = 0.0;
                p_bar_k_t_[x][y][theta] = 0.0;

                if(visualisation_flag_)
                {
                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = theta*0.5;
                    points.push_back(point);

                    std_msgs::ColorRGBA color;
                    color.a = p_t_1_[x][y][theta];
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
        curr_pose_.pose.pose.position.x = max_x + map_.info.origin.position.x;
        curr_pose_.pose.pose.position.y = max_y + map_.info.origin.position.y;
        curr_pose_.pose.pose.orientation = Utils::getQuat(max_theta);
    }

    prev_odom_ = curr_odom;
    return curr_pose_;
}