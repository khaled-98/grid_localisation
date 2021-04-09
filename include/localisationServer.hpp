#ifndef SRC_GRID_LOCALISATION_INCLUDE_LOCALISATIONSERVER
#define SRC_GRID_LOCALISATION_INCLUDE_LOCALISATIONSERVER

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

class LocalisationServer
{
public:
    LocalisationServer();
    nav_msgs::OccupancyGrid requestMap();
    void scanCallback(const sensor_msgs::LaserScanConstPtr &scan);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<GridLocalisation> grid_localisation_;
    std::shared_ptr<MotionModel> motion_model_;
    std::shared_ptr<MeasurementModel> measurement_model_;

    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;
    std::string laser_topic_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> laser_scan_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_scan_sub_;

    ros::Publisher curr_location_pub_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_LOCALISATIONSERVER */
