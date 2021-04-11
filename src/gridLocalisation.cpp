#include "../include/gridLocalisation.hpp"
#include "../include/utils.hpp"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <vector>
#include "tf2/utils.h"
#include <cmath>

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
    {
        prob_visual_pub_ = nh_.advertise<visualization_msgs::Marker>("probability_visualisation", 0);
        rolling_window_visual_pub_ = nh_.advertise<visualization_msgs::Marker>("rolling_window", 0);
    }
}

void GridLocalisation::setMap(const nav_msgs::OccupancyGrid &map)
{
    map_recieved_ = true;
    map_ = map;
    measurement_model_->setMap(map);

    // Initialise distribution
    if(starting_point_set_)
    {
        if(starting_theta_ < 0.0)
            starting_theta_ += 2*M_PI; // keep it in the 0 to 2pi range

        int x = starting_x_/grid_linear_resolution_;
        int y = starting_y_/grid_linear_resolution_;
        int theta = starting_theta_/grid_angular_resolution_;

        p_t_1_[x][y][theta] = 1.0;
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

    // update the rolling window
    double start_x = curr_pose_.pose.pose.position.x - (rolling_window_length_/2);
    if(start_x < map_.info.origin.position.x)
        start_x = map_.info.origin.position.x;
    start_x /= grid_linear_resolution_;
    
    int window_start_x = round(start_x);

    double start_y = curr_pose_.pose.pose.position.y - (rolling_window_length_/2);
    if(start_y < map_.info.origin.position.y)
        start_y = map_.info.origin.position.y;
    start_y /= grid_linear_resolution_;
    
    int window_start_y = round(start_y);

    double end_x = curr_pose_.pose.pose.position.x + (rolling_window_length_/2);
    if(end_x > (map_.info.origin.position.x + map_width))
        end_x = map_.info.origin.position.x + map_width;
    end_x /= grid_linear_resolution_;

    int window_end_x = round(end_x);

    double end_y = curr_pose_.pose.pose.position.y + (rolling_window_length_/2);
    if(end_y > (map_.info.origin.position.y + map_height))
        end_y = map_.info.origin.position.y + map_height;
    end_y /= grid_linear_resolution_;
    
    int window_end_y = round(end_y);
 
    if(visualisation_flag_)
    {
        visualization_msgs::Marker rolling_window_marker;
        rolling_window_marker.header.frame_id = "map";
        rolling_window_marker.header.stamp = ros::Time();
        rolling_window_marker.type = visualization_msgs::Marker::LINE_STRIP;
        rolling_window_marker.scale.x = 0.01;
        rolling_window_marker.color.a = 1.0;

        std::vector<geometry_msgs::Point> rolling_window_corners;
        geometry_msgs::Point p1;
        p1.x = window_start_x*grid_linear_resolution_;
        p1.y = window_start_y*grid_linear_resolution_;
        
        geometry_msgs::Point p2;
        p2.x = window_end_x*grid_linear_resolution_;
        p2.y = window_start_y*grid_linear_resolution_;
        
        geometry_msgs::Point p3;
        p3.x = window_end_x*grid_linear_resolution_;
        p3.y = window_end_y*grid_linear_resolution_;
        
        geometry_msgs::Point p4;
        p4.x = window_start_x*grid_linear_resolution_;
        p4.y = window_end_y*grid_linear_resolution_;
        rolling_window_corners.push_back(p1);
        rolling_window_corners.push_back(p2);
        rolling_window_corners.push_back(p2);
        rolling_window_corners.push_back(p3);
        rolling_window_corners.push_back(p3);
        rolling_window_corners.push_back(p4);
        rolling_window_corners.push_back(p4);
        rolling_window_corners.push_back(p1);
        
        rolling_window_marker.points = rolling_window_corners;
        rolling_window_visual_pub_.publish(rolling_window_marker);
    }

    double sum_of_dist_values = 0.0;
    for(int x_k = window_start_x; x_k < window_end_x; x_k++)
    {
        for(int y_k = window_start_y; y_k < window_end_y; y_k++)
        {
            for(int theta_k = 0; theta_k < (2*M_PI/grid_angular_resolution_); theta_k += grid_angular_resolution_)
            {
                geometry_msgs::Pose xt;
                xt.position.x = x_k*grid_linear_resolution_;
                xt.position.y = y_k*grid_linear_resolution_;
                xt.orientation = Utils::getQuat(theta_k*grid_angular_resolution_);

                for(int x_i = window_start_x; x_i < window_end_x; x_i++)
                {
                    for(int y_i = window_start_y; y_i < window_end_y; y_i++)
                    {
                        for(int theta_i = 0; theta_i < (2*M_PI/grid_angular_resolution_); theta_i++)
                        {
                            geometry_msgs::Pose xt_1;
                            xt_1.position.x = x_i*grid_linear_resolution_;
                            xt_1.position.y = y_i*grid_linear_resolution_;
                            xt_1.orientation = Utils::getQuat(theta_i*grid_angular_resolution_);

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

    int max_x = 0;
    int max_y = 0;
    int max_theta = 0;
    double max_prob = 0.0;
    for(int x = window_start_x; x < window_end_x; x++)
    {
        for(int y = window_start_y; y < window_end_y; y++)
        {
            for(int theta = 0.0; theta < (2*M_PI/grid_angular_resolution_); theta++)
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
                    point.x = x*grid_linear_resolution_;
                    point.y = y*grid_linear_resolution_;
                    point.z = theta*grid_angular_resolution_*0.5;
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

        prob_visual_pub_.publish(marker);
    }

    if(max_prob != 0.0)
    {
        curr_pose_.pose.pose.position.x = max_x*grid_linear_resolution_;
        curr_pose_.pose.pose.position.y = max_y*grid_linear_resolution_;
        curr_pose_.pose.pose.orientation = Utils::getQuat(max_theta*grid_angular_resolution_);
    }

    prev_odom_ = curr_odom;
    return curr_pose_;
}