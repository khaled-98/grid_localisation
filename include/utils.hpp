#ifndef SRC_GRID_LOCALISATION_INCLUDE_UTILS
#define SRC_GRID_LOCALISATION_INCLUDE_UTILS

#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace Utils
{
int map_to_index(const int &row, const int &col, const int &map_width)
{
    return row*map_width + col;
}

bool has_moved(const geometry_msgs::TransformStamped &start,
               const geometry_msgs::TransformStamped &end,
               const double &trans_tol,
               const double &rot_tol)
{
    tf2::Quaternion quat_tf;
    
    tf2::convert(start.transform.rotation, quat_tf);
    double theta = quat_tf.getAngle();
    double x = start.transform.translation.x;
    double y = start.transform.translation.y;

    tf2::convert(end.transform.rotation, quat_tf);
    double theta_prime = quat_tf.getAngle();
    double x_prime = end.transform.translation.x;
    double y_prime = end.transform.translation.y;
    
    double translation = sqrt(pow(x_prime-x, 2) + pow(y_prime-y, 2));
    double rotation = abs(theta_prime-theta);

    if(translation > trans_tol || rotation > rot_tol)
        return true;
    else
        return false;
}

}

#endif /* SRC_GRID_LOCALISATION_INCLUDE_UTILS */
