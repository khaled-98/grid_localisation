import math
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

def normalise(a):
    return math.atan2(math.sin(a), math.cos(a))

def angle_diff(a, b):
    # Ensures that the difference between the angles lies between -pi and pi.
    a = normalise(a)
    b = normalise(b)
    d1 = a-b
    d2 = 2*math.pi - abs(d1)
    if(d1 > 0):
        d2 *= -1
    if(abs(d1) < abs(d2)):
        return d1
    else:
        return d2

def prob(a, b):
    return (1/math.sqrt(2*math.pi*b))*math.exp(-0.5*((a**2)/b))

def motion_model(x_t, u_t, x_t_1):
    delta_rot1 = angle_diff(math.atan2(u_t[1].y-u_t[0].y, u_t[1].x-u_t[0].x), u[0].theta)
    delta_trans = math.sqrt((u_t[0].x-u_t[1].x)**2 + (u_t[0].y-u_t[1].y)**2)
    delta_rot2 = angle_diff(u_t[1].theta, angle_diff(u_t[0].theta, delta_rot1)) # NOTE: for some reason AMCL doesn't subtract the second theta

    delta_rot1_hat = angle_diff(math.atan2(x_t.y-x_t_1.y, x_t.x-x_t_1.x), x_t_1.theta)
    delta_trans_hat = math.sqrt((x_t_1.x-x_t.x)**2 + (x_t_1.y-x_t.y)**2)
    delta_rot2_hat = angle_diff(x_t_1.theta, angle_diff(x_t_1.theta, delta_rot1))

    p1 = prob(angle_diff(delta_rot1, delta_rot1_hat), alpha1*(delta_rot1_hat**2)+alpha2*(delta_trans**2))
    p2 = prob(delta_trans-delta_trans_hat, alpha3*(delta_trans_hat**2)+alpha4*(delta_rot1_hat**2)+alpha4*(delta_rot2_hat**2))
    p3 = prob(angle_diff(delta_rot2, delta_rot2_hat), alpha1*(delta_rot2_hat**2)+alpha2*(delta_trans_hat**2))

    return p1*p2*p3

def observation_model(z, x, m):
    q = 1

    k = size(z.ranges)
    for i in range(k):
        # map the laser end-points onto the global coordinates
        x_z = x.x + x_sensor*math.cos(x.theta) - y_sensor*math.sin(x.theta) + z.ranges[i]*math.cos(x.theta + theta_sensor)
        y_z = x.y + y_sensor*math.cos(x.theta) - x_sensor*math.sin(x.theta) + z.ranges[i]*math.sin(x.theta + theta_sensor)

        # find the distance to the nearest object
        dist = find_dist(x_z, y_z, m)

        q = q*(z_hit*prob(dist,sigma_hit**2) + (z_random/z_max))

    return q

previous_pose = Odometry()
current_pose = Odometry()

previous_odata = Odometry()
current_odata = Odometry()

def odom_callback(data):
    previous_pose = current_pose
    current_pose = data.pose.pose

def init():
    rospy.init_node('grid_localisation')
    # rospy.Subscriber("odom", Odometry, odom_callback)

    rospy.wait_for_service('static_map') # Hold until map is available
    rospy.loginfo("Waiting for map ...")
    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        map = get_map()
        rospy.loginfo("Map Recieved!")
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return


    # previous_dist =
    while not rospy.is_shutdown():
        prior = motion_model(current_pose, [previous_odata, current_odata], previous_pose)


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
