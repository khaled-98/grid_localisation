import math

def normalise(a):
    reutrn math.atan2(math.sin(a), math.cos(a))

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
