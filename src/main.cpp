#include "ros/ros.h"
#include "../include/localisationServer.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  LocalisationServer grid_localisation_node;
  ros::spin();
  return 0;
}