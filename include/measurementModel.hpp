#ifndef SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL
#define SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

class MeasurementModel
{
public:
    MeasurementModel();
    double run(const int* xt, sensor_msgs::LaserScanConstPtr &scan);
    void compute_likelihood_field_prob(nav_msgs::OccupancyGridConstPtr &map);
private:
    double z_hit_;
    double sigma_hit_;
    double z_rand_;
    double z_max_;
    std::vector<double> likelihood_field_prob_;

    ros::NodeHandle private_nh_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_MEASUREMENTMODEL */
