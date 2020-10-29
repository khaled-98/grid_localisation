#include "ros/ros.h"
#include "../include/gridLocalisation.hpp"
#include "../include/motionModel.hpp"
#include "../include/measurementModel.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_localisation");
    ros::NodeHandle nh;
    
    std::shared_ptr<MotionModel> motion_model = std::make_shared<MotionModel>(nh);
    std::shared_ptr<MeasurementModel> measurement_model = std::make_shared<MeasurementModel>(nh);
    
    GridLocalisation grid_localisation(nh, motion_model, measurement_model);

    while (ros::ok())
    {

    }
    return 0;
}