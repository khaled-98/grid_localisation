#include "../include/gridLocalisationNode.hpp"
#include "nav_msgs/GetMap.h"

GridLocalisationNode::GridLocalisationNode() : private_nh_("~")
{
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
    private_nh_.param("laser_topic", laser_topic_, std::string("base_scan"));

    curr_location_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("current_location", 100);

    motion_model_ = std::make_shared<MotionModel>();
    measurement_model_ = std::make_shared<MeasurementModel>();
    grid_localisation_ = std::make_shared<GridLocalisation>(motion_model_, measurement_model_);

    grid_localisation_->set_map(request_map());

    laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, laser_topic_, 100);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    laser_scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*laser_scan_sub_,
                                                                                          *tf_buffer_,
                                                                                          odom_frame_id_,
                                                                                          100,
                                                                                          nh_);
    laser_scan_filter_->registerCallback(std::bind(&GridLocalisationNode::scan_callback, this, std::placeholders::_1));
}

nav_msgs::OccupancyGrid GridLocalisationNode::request_map()
{
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response resp;

    ROS_INFO("Waiting for map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    ROS_INFO("Map received!");
    
    return resp.map;
}

void GridLocalisationNode::scan_callback(const sensor_msgs::LaserScanConstPtr &scan)
{
    try
    {
        curr_odom_ = tf_buffer_->lookupTransform(odom_frame_id_, base_frame_id_, scan->header.stamp, ros::Duration(1.0));
        geometry_msgs::PoseWithCovarianceStamped curr_location = grid_localisation_->localise(scan, curr_odom_);
        curr_location.header.stamp = ros::Time::now();
        curr_location.header.frame_id = global_frame_id_;
        curr_location_pub_.publish(curr_location);
        // ROS_ERROR("`%f, %f", curr_location.pose.pose.position.x, curr_location.pose.pose.position.y);
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("Failure %s\n", ex.what());
    }
}