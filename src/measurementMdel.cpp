#include "../include/measurementModel.hpp"

MeasurementModel::MeasurementModel(const ros::NodeHandle &nh)
{
    nh.param("z_hit", z_hit_, 0.95);
    nh.param("sigma_hit", sigma_hit_, 0.2);
    nh.param("z_rand", z_rand_, 0.05);
    nh.param("z_max", z_max_, 0.05);
}