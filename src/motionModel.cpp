#include "../include/motionModel.hpp"
#include "../include/utils.hpp"
#include <cmath>

MotionModel::MotionModel() : private_nh_("~")
{
    private_nh_.param("alpha1", alpha1_, 0.2);
    private_nh_.param("alpha2", alpha2_, 0.2);
    private_nh_.param("alpha3", alpha3_, 0.2);
    private_nh_.param("alpha4", alpha4_, 0.2);
}

double MotionModel::run(const double* xt, const double* xt_1, const double* ut, const double* ut_1)
{
    double x = xt_1[0];
    double y = xt_1[1];
    double theta = xt_1[2];
 
    double x_prime = xt[0];
    double y_prime = xt[1];
    double theta_prime = xt[2];

    double x_bar = ut_1[0];
    double y_bar = ut_1[1];
    double theta_bar = ut_1[2];

    double x_bar_prime = ut[0];
    double y_bar_prime = ut[1];
    double theta_bar_prime = ut[2];

    double delta_trans = sqrt(pow(x_bar-x_bar_prime, 2) + pow(y_bar-y_bar_prime, 2));
    double delta_rot_1 = Utils::angle_diff(atan2(y_bar_prime-y_bar, x_bar_prime-x_bar), theta_bar);
    if(delta_rot_1 < -M_PI_2)
    {
        delta_rot_1 += M_PI;
        delta_trans *= -1;   
    }
    else if(delta_rot_1 > M_PI_2)
    {
        delta_rot_1 -= M_PI;
        delta_trans *= -1;
    }
    double delta_rot_2 = Utils::angle_diff(Utils::angle_diff(theta_bar_prime, theta_bar), delta_rot_1);

    double delta_trans_hat = sqrt(pow(x-x_prime, 2) + pow(y-y_prime, 2));
    double delta_rot_1_hat = Utils::angle_diff(atan2(y_prime-y, x_prime-x), theta);
    if(delta_rot_1_hat < -M_PI_2)
    {
        delta_rot_1_hat += M_PI;
        delta_trans_hat *= -1;
    }
    else if(delta_rot_1_hat > M_PI_2)
    {
        delta_rot_1_hat -= M_PI;
        delta_trans_hat *= -1;
    }
    double delta_rot_2_hat = Utils::angle_diff(Utils::angle_diff(theta_prime, theta), delta_rot_1_hat);

    double a, b, p1, p2, p3;
    a = Utils::angle_diff(delta_rot_1, delta_rot_1_hat);
    b = sqrt(alpha1_*pow(delta_rot_1_hat, 2) + alpha2_*pow(delta_trans_hat, 2));
    p1 = Utils::prob(a, b);
    if(std::isnan(p1))
    {
        // the variance is zero if delta_rot_1 and delta_trans are 0
        if(delta_rot_1==0 && delta_trans==0)
            p1 = 1.0;
        else
            p1 = 0.0;
    }

    a = delta_trans - delta_trans_hat;
    b = sqrt(alpha3_*pow(delta_trans_hat, 2) + alpha4_*(pow(delta_rot_1_hat, 2) + pow(delta_rot_2_hat, 2)));
    p2 = Utils::prob(a, b);
    if(std::isnan(p2))
    {
        // Assuming ut and ut_1 are not the same, a variance of 0 is invalid
        p2 = 0.0;
    }

    a = Utils::angle_diff(delta_rot_2, delta_rot_2_hat);
    b = sqrt(alpha1_*(pow(delta_rot_1_hat, 2) + pow(delta_rot_2_hat, 2)) + alpha2_*pow(delta_trans_hat, 2));
    p3 = Utils::prob(a, b);
    if(std::isnan(p3))
    {
        // the variance is zero if delta_rot_2 and delta_trans are 0
        if(delta_rot_2==0 && delta_trans==0)
            p3 = 1.0;
        else
            p3 = 0.0;
    }

    return p1*p2*p3;
}
