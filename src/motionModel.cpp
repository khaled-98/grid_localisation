#include "../include/motionModel.hpp"

MotionModel::MotionModel(const ros::NodeHandle &nh)
{
    nh.param("alpha1", alpha1_, 0.2);
    nh.param("alpha2", alpha2_, 0.2);
    nh.param("alpha3", alpha3_, 0.2);
    nh.param("alpha4", alpha4_, 0.2);
}