#ifndef SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL
#define SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL

#include "ros/ros.h"

class MotionModel
{
public:
    MotionModel();
    double run(const int* xt, const int* xt_1, const int* ut, const int* ut_1);
private:
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;

    ros::NodeHandle private_nh_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL */
