#ifndef SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL
#define SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"

class MotionModel
{
public:
    MotionModel();
    double getTransitionProbability(const geometry_msgs::Pose &xt, const geometry_msgs::Pose &xt_1,
                                    const geometry_msgs::TransformStamped &ut, const geometry_msgs::TransformStamped &ut_1);
private:
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;

    ros::NodeHandle private_nh_;
};

#endif /* SRC_GRID_LOCALISATION_INCLUDE_MOTIONMODEL */
