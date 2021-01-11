#ifndef SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL
#define SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL

#include "ros/ros.h"

class MotionModel
{
public:
    MotionModel();
    double run(const double* xt, const double* xt_1, const double* ut, const double* ut_1);
private:
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;

    ros::NodeHandle private_nh_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL */
