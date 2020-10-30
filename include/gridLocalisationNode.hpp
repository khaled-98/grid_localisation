#ifndef SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATIONNODE
#define SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATIONNODE

#include "ros/ros.h"
#include "../include/motionModel.hpp"
#include "../include/measurementModel.hpp"
#include "../include/gridLocalisation.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/message_filter.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class GridLocalisationNode
{
public:
    GridLocalisationNode();
    nav_msgs::OccupancyGrid request_map();
    void scan_callback(const sensor_msgs::LaserScanConstPtr &scan);
    void run_through_algorithm();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<GridLocalisation> grid_localisation_;
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string laser_topic_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;

    geometry_msgs::TransformStamped curr_odom_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_GRIDLOCALISATIONNODE */
