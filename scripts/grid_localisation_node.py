import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetMap
from tf.transformations import euler_from_quaternion

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

def angle_from_orientation(orientation):
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

def motion_model(x_t, u_t, x_t_1):
    alpha1 = 0.2
    alpha2 = 0.2
    alpha3 = 0.8
    alpha4 = 0.2
    alpha5 = 0.1

    x_prime = x_t[0]
    y_prime = x_t[1]
    theta_prime = x_t[2]

    x_bar = u_t[0].position.x
    y_bar = u_t[0].position.y
    theta_bar = angle_from_orientation(u_t[0].orientation)

    x_bar_prime = u_t[1].position.x
    y_bar_prime = u_t[1].position.y
    theta_bar_prime = angle_from_orientation(u_t[1].orientation)

    x = x_t_1[0]
    y = x_t_1[1]
    theta = x_t_1[2]

    delta_rot1 = angle_diff(math.atan2(y_bar_prime-y_bar, x_bar_prime-x_bar), theta_bar)
    delta_trans = math.sqrt((x_bar - x_bar_prime)**2 + (y_bar - y_bar_prime)**2)
    delta_rot2 = angle_diff(theta_bar_prime, angle_diff(theta_bar, delta_rot1)) # NOTE: for some reason AMCL doesn't subtract the second theta

    delta_rot1_hat = angle_diff(math.atan2(y_prime-y, x_prime-x), theta)
    delta_trans_hat = math.sqrt((x-x_prime)**2 + (y-y_prime)**2)
    delta_rot2_hat = angle_diff(theta_prime, angle_diff(theta, delta_rot1_hat))

    try:    # NOTE: this is only a temporary fix. When the starting and finishing pose are the same, the code divides by zero. I assume that this is not possible because this code is only run when the robot is in motion
        p1 = prob(angle_diff(delta_rot1, delta_rot1_hat), alpha1*(delta_rot1_hat**2)+alpha2*(delta_trans**2))
        p2 = prob(delta_trans-delta_trans_hat, alpha3*(delta_trans_hat**2)+alpha4*(delta_rot1_hat**2)+alpha4*(delta_rot2_hat**2))
        p3 = prob(angle_diff(delta_rot2, delta_rot2_hat), alpha1*(delta_rot2_hat**2)+alpha2*(delta_trans_hat**2))
    except ZeroDivisionError:
        return 0

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

previous_odata = Odometry().pose.pose
current_odata = Odometry().pose.pose

def odom_callback(data):
    global current_odata
    current_odata = data.pose.pose

def init():
    rospy.init_node('grid_localisation')
    rospy.Subscriber("odom", Odometry, odom_callback)

    rospy.wait_for_service('static_map') # Hold until map is available
    rospy.loginfo("Waiting for map ...")
    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        map = get_map().map
        rospy.loginfo("Map Recieved!")
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return

    # Define grid resolution
    map_width = map.info.resolution*map.info.width
    map_height = map.info.resolution*map.info.height

    linear_resolution = 1 # 15 cm
    angular_resolution = math.radians(5) # degrees to radians

    grid_width = int(math.floor(map_width/linear_resolution))
    grid_length = int(math.floor(map_height/linear_resolution))
    grid_depth = int(math.floor((2*math.pi)/angular_resolution))
    number_of_cells_in_grid = grid_width * grid_length * grid_depth

    # Initialise distributions uniformly
    previous_dist = np.full([grid_width, grid_length, grid_depth], 1/number_of_cells_in_grid)
    current_dist = previous_dist

    pbarkt = np.zeros([grid_width, grid_length, grid_depth])

    previous_odata = current_odata # Not sure if this is a valid thing to do

    r = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        ut = [previous_odata, current_odata]
        if(ut[0] == ut[1]): # Don't do anything if the robot didn't move
            continue

        for k in range(number_of_cells_in_grid): # line 2 of table 8.1
            [rowk, colk, depthk] = np.unravel_index(k,[grid_width, grid_length, grid_depth])
            xt = [rowk, colk, depthk*angular_resolution]

            # =============== Prediction ================
            for i in range(number_of_cells_in_grid):
                [rowi, coli, depthi] = np.unravel_index(i, [grid_width, grid_length, grid_depth])
                xt_d1 = [rowi, coli, depthi*angular_resolution]

                pbarkt[rowk, colk, depthk] += previous_dist[rowi, coli, depthi]*motion_model(xt, ut, xt_d1)
            # ===========================================
        previous_odata = ut[1] # previous = current
        r.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
