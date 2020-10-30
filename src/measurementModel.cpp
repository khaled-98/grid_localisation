#include "../include/measurementModel.hpp"

MeasurementModel::MeasurementModel() : private_nh_("~")
{
    private_nh_.param("z_hit", z_hit_, 0.95);
    private_nh_.param("sigma_hit", sigma_hit_, 0.2);
    private_nh_.param("z_rand", z_rand_, 0.05);
    private_nh_.param("z_max", z_max_, 0.05);
}