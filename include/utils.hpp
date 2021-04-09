#ifndef SRC_GRID_LOCALISATION_INCLUDE_UTILS
#define SRC_GRID_LOCALISATION_INCLUDE_UTILS

#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

namespace Utils
{
int map_to_index(const int &row, const int &col, const int &map_width);
double getAngle(geometry_msgs::Quaternion quat_geo);
geometry_msgs::Quaternion getQuat(double yaw);
bool hasMoved(const geometry_msgs::TransformStamped &start,
              const geometry_msgs::TransformStamped &end,
              const double &trans_tol,
              const double &rot_tol);
double angleDiff(double a, double b);
double prob(double a, double b);
unsigned long long subToIndex(int n1, int n2, int n3, int N2, int N3);
}

#endif /* SRC_GRID_LOCALISATION_INCLUDE_UTILS */
