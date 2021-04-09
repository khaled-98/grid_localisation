#include "../include/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/utils.h"

namespace Utils
{
int map_to_index(const int &row, const int &col, const int &map_width)
{
    return row*map_width + col;
}

double getAngle(geometry_msgs::Quaternion quat_geo)
{
    tf2::Quaternion quat_tf;
    tf2::fromMsg(quat_geo, quat_tf);
    return tf2::impl::getYaw(quat_tf);
}

geometry_msgs::Quaternion getQuat(double yaw)
{
    tf2::Quaternion tf_quat;
    geometry_msgs::Quaternion geo_quat;

    tf_quat.setRPY(0.0, 0.0, yaw);
    tf2::convert(tf_quat, geo_quat);

    return geo_quat;
}

bool hasMoved(const geometry_msgs::TransformStamped &start,
               const geometry_msgs::TransformStamped &end,
               const double &trans_tol,
               const double &rot_tol)
{
    double theta = getAngle(start.transform.rotation);
    double x = start.transform.translation.x;
    double y = start.transform.translation.y;

    double theta_prime = getAngle(end.transform.rotation);
    double x_prime = end.transform.translation.x;
    double y_prime = end.transform.translation.y;
    
    double translation = sqrt(pow(x_prime-x, 2) + pow(y_prime-y, 2));
    double rotation = abs(angleDiff(theta_prime, theta));

    if(translation > trans_tol || rotation > rot_tol)
        return true;
    else
        return false;
}

double angleDiff(double a, double b)
{
    double angle = a - b;

    angle = fmod(angle, 2.0*M_PI); // limit the angle from 0 to 2*Pi

    if(angle <= M_PI && angle >= -M_PI) // angle within the desired limit
        return angle;

    else if(angle > M_PI)   
        return angle - 2.0*M_PI;
    
    else
        return angle + 2.0*M_PI;
}

double prob(double a, double b)
{
    return (1.0/(sqrt(2*M_PI)*b))*exp(-0.5*((a*a)/(b*b)));
}

unsigned long long subToIndex(int n1, int n2, int n3, int N2, int N3)
{
    // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
    return n3 + N3*(n2+N2*n1);
}


} // namespace Utils
