#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Transform.h"
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

float atan2_approximation(float y, float x)
{
  //https://gist.github.com/volkansalma/2972237#file-atan2_approximation-c-L15

    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA

  const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );
}

void ind2sub(int index, int N1, int N2, int N3, int* i, int* j, int* k)
{
  // Don't event ask
  // https://eli.thegreenplace.net/2015/memory-layout-of-multi-dimensional-arrays/
  int n1, n2, n3;
  n3 = index % N3;
  n2 = ((index-n3)/N3)%N2;
  n1 = (((index-n3)/N3)-n2)/N2;
  *i = n1;
  *j = n2;
  *k = n3;
}

double angle_from_orientation(geometry_msgs::Quaternion orientation)
{
  // orientation (geometry_msgs::Quaternion) is transformed to a tf::Quaterion
  tf::Quaternion quat;
  tf::quaternionMsgToTF(orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  return yaw;
}

bool isNoMovement(tf::StampedTransform prev, tf::StampedTransform curr)
{
  double linear_tol=0.1, angular_tol=0.052; // 3 degrees in radians
  double x = prev.getOrigin().getX();
  double y = prev.getOrigin().getY();
  double theta = tf::getYaw(prev.getRotation());

  double x_prime = curr.getOrigin().getX();
  double y_prime = curr.getOrigin().getY();
  double theta_prime = tf::getYaw(curr.getRotation());

  // Check if the distance travelled is bigger than the tolerance
  if(sqrt((pow(x-x_prime, 2))+pow(y-y_prime, 2)) < linear_tol)
    return true;

  // Check if the angle travelled is bigger than the tolerance
  if(fabs(theta - theta_prime) < angular_tol)
    return true;

  return false;
}

double normalise(double z)
{
  // TODO: check maths: https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360
  const double width = 2*M_PI;   //
  const double offsetValue = z - (-M_PI) ;   // value relative to 0

  return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + (-M_PI) ;
}

double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalise(a);
  b = normalise(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

float approx_sqrt( float number )
{
  // https://www.geeksforgeeks.org/fast-inverse-square-root/
  // https://www.beyond3d.com/content/articles/8/
  const float threehalfs = 1.5F;

  float x2 = number * 0.5F;
  float y = number;

  // evil floating point bit level hacking
  long i = * ( long * ) &y;

  // value is pre-assumed
  i = 0x5f3759df - ( i >> 1 );
  y = * ( float * ) &i;

  // 1st iteration
  y = y * ( threehalfs - ( x2 * y * y ) );

  // 2nd iteration, this can be removed
  y = y * ( threehalfs - ( x2 * y * y ) );

  return 1/y;
}

double approx_exp(double x)
{
  // https://codingforspeed.com/using-faster-exponential-approximation/
  x = 1.0 + x / 256.0;
  x *= x; x *= x; x *= x; x *= x;
  x *= x; x *= x; x *= x; x *= x;
  return x;
}

double prob(double a, double b)
{
  return (1/approx_sqrt(2*M_PI*b))*approx_exp(-0.5*((a*a)/b));
}

double motion_model(double* xt, double* ut, double* xt_d1)
{
  float alpha1 = 0.2, alpha2 = 0.2, alpha3=0.8, alpha4=0.2, alpha5=0.1;
  double x_prime = xt[0];
  double y_prime = xt[1];
  double theta_prime = xt[2];

  double x_bar = ut[0];
  double y_bar = ut[1];
  double theta_bar = ut[2];

  double x_bar_prime = ut[3];
  double y_bar_prime = ut[4];
  double theta_bar_prime = ut[5];

  double x = xt_d1[0];
  double y = xt_d1[1];
  double theta = xt_d1[2];

  double delta_rot1 = angle_diff(atan2_approximation(y_bar_prime-y_bar, x_bar_prime-x_bar), theta_bar);
  double delta_trans =  approx_sqrt((x_bar - x_bar_prime)*(x_bar - x_bar_prime) + (y_bar - y_bar_prime)*(y_bar - y_bar_prime));
  double delta_rot2 =  angle_diff(theta_bar_prime, angle_diff(theta_bar, delta_rot1)); // NOTE: for some reason AMCL doesn't subtract the second theta

  double delta_rot1_hat =  angle_diff(atan2_approximation(y_prime-y, x_prime-x), theta);
  double delta_trans_hat = approx_sqrt((x-x_prime)*(x-x_prime) + (y-y_prime)*(y-y_prime));
  double delta_rot2_hat = angle_diff(theta_prime, angle_diff(theta, delta_rot1_hat));

  double p1 = prob(angle_diff(delta_rot1, delta_rot1_hat), alpha1*(delta_rot1_hat*delta_rot1_hat)+alpha2*(delta_trans*delta_trans));
  double p2 = prob(delta_trans-delta_trans_hat, alpha3*(delta_trans_hat*delta_trans_hat)+alpha4*(delta_rot1_hat*delta_rot1_hat)+alpha4*(delta_rot2_hat*delta_rot2_hat));
  double p3 = prob(angle_diff(delta_rot2, delta_rot2_hat), alpha1*(delta_rot2_hat*delta_rot2_hat)+alpha2*(delta_trans_hat*delta_trans_hat));

  return p1*p2*p3;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_localisation");
  ros::NodeHandle n;

  ros::ServiceClient map_srv_client = n.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap map_srv;

  ROS_INFO("Waiting for map...");
  if(map_srv_client.call(map_srv))
    ROS_INFO("Map received!");
  else
    ROS_INFO("Failed to call map service");

  // Define grid resolution
  double map_width = map_srv.response.map.info.resolution*map_srv.response.map.info.width;
  double map_height = map_srv.response.map.info.resolution*map_srv.response.map.info.height;

  double linear_resolution = 0.15; // 15 cm
  double angular_resolution = 0.0873; // 5 degress in radians

  int grid_width = std::floor(map_width/linear_resolution);
  int grid_length = std::floor(map_height/linear_resolution);
  int grid_depth = std::floor((2*M_PI)/angular_resolution);

  long number_of_grid_cells = grid_width*grid_length*grid_depth;

  // Initialise distribution uniformly
  ROS_INFO("Initialising distribution...");
  double previous_dist[100][100][73];
  double current_dist[100][100][73];
  double temp_prob = 1/number_of_grid_cells;
  for (int r = 0; r < 100; r++)
  {
    for (int c = 0; c < 100; c++)
    {
      for (int d = 0; d < 73; d++)
      {
        previous_dist[r][c][d] = temp_prob;
        current_dist[r][c][d] = temp_prob;
      }
    }
  }
  ROS_INFO("Initialised...");

  // Initliase with zeros
  double p_bar_kt[100][100][73];

  tf::TransformListener tf_listener;
  tf::StampedTransform current_transform;
  tf::StampedTransform previous_transform;

  tf_listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), previous_transform); // get initial transform

  ros::Rate loop_rate(10); // 10 Hz
  double test[100][100][73];
  while(ros::ok())
  {
    try{
      tf_listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), current_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    double ut[6] = {previous_transform.getOrigin().getX(), previous_transform.getOrigin().getY(), tf::getYaw(previous_transform.getRotation()),
                    current_transform.getOrigin().getX(), current_transform.getOrigin().getY(), tf::getYaw(current_transform.getRotation())};

    if(isNoMovement(previous_transform, current_transform)) // Don't do anything if the robot didn't move
      continue;

    ROS_INFO("Round Started!");
    for(long k = 0; k < number_of_grid_cells; k++) // line 2 of table 8.1
    {
      int rowk, colk, depthk;
      double xt[3];

      ind2sub(k, grid_width, grid_length, grid_depth, &rowk, &colk, &depthk);
      xt[0] = rowk*linear_resolution;
      xt[1] = colk*linear_resolution;
      xt[2] = depthk*angular_resolution;

      // TODO: BE CAREFUL ABOUT WHEN YOU TAKE THE TRANSFORM READINGS
      // ================ Prediction =================
      for(long i = 0; i < number_of_grid_cells; i++)
      {
        int rowi, coli, depthi;
        double xt_d1[3];

        ind2sub(i, grid_width, grid_length, grid_depth, &rowi, &coli, &depthi);
        xt_d1[0] = rowi*linear_resolution;
        xt_d1[1] = coli*linear_resolution;
        xt_d1[2] = depthi*angular_resolution;

        p_bar_kt[rowk][colk][depthk] += previous_dist[rowi][coli][depthi]*motion_model(xt, ut, xt_d1);
      }
    }
    previous_transform = current_transform;

    ROS_INFO("One round completed!");
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin(); // do I need this?

  return 0;
}
