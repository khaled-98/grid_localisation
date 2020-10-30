#ifndef SRC_GRID_LOCALISATION_INCLUDE_UTILS
#define SRC_GRID_LOCALISATION_INCLUDE_UTILS

#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

namespace Utils
{
int map_to_index(const int &row, const int &col, const int &map_width);
double get_angle_from_quat(geometry_msgs::Quaternion quat_geo);
geometry_msgs::Quaternion getQuat(double yaw);
bool has_moved(const geometry_msgs::TransformStamped &start,
               const geometry_msgs::TransformStamped &end,
               const double &trans_tol,
               const double &rot_tol);
double angle_diff(double a, double b);
double prob(double a, double b);
}

#endif /* SRC_GRID_LOCALISATION_INCLUDE_UTILS */
