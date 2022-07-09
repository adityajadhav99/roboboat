#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Float32

"""

/wamv/thrusters/left_front_thrust_cmd
/wamv/thrusters/left_rear_thrust_cmd
/wamv/thrusters/right_front_thrust_cmd
/wamv/thrusters/right_rear_thrust_cmd

"""

def inverse_glf_map(T):

    k = T
    # print(f[0][0])

    for i in range(np.shape(T)[0]):

        if T[i] >= 250:
            T[i] = 250
        elif T[i] < -100:
            T[i] = -100
        if 0.06 <= T[i] < 1.2:
            T[i] = 0.06
        
        if T[i] >= 1.2:
            A = 0.01
            K = 59.82
            B = 5.0
            nu = 0.38
            C = 0.56
            M = 0.

        if T[i] <= 0.06:
            A = -199.13
            K = -0.09
            B = 8.84
            nu = 5.34
            C = 0.99
            M = -0.57

        k[i] = M - (1/B)*math.log((((K-A)/(T[i]-A))**nu)-C)
    
    return k

def control():
    rospy.init_node('control')

    fl_cmd = rospy.Publisher('/wamv/thrusters/left_front_thrust_cmd', Float32, queue_size=1)
    fr_cmd = rospy.Publisher('/wamv/thrusters/right_front_thrust_cmd', Float32, queue_size=1)
    rl_cmd = rospy.Publisher('/wamv/thrusters/left_rear_thrust_cmd', Float32, queue_size=1)
    rr_cmd = rospy.Publisher('/wamv/thrusters/right_rear_thrust_cmd', Float32, queue_size=1)

    # left_cmd = +1.0
    # right_cmd = -1.0

    lx1 = 1.6;       ly1 = 0.7;         a1 = -0.785
    lx2 = -2.373776; ly2 = 1.027135;    a2 = 0.785
    lx3 = -2.373776; ly3 = -1.027135;   a3 = -0.785
    lx4 = 1.6;       ly4 = -0.7;        a4 = 0.785

    tau = np.array([[0],[2000],[0]])

    T = np.array([[np.cos(a1),np.cos(a2),np.cos(a3),np.cos(a4)],
                  [np.sin(a1),np.sin(a2),np.sin(a3),np.sin(a4)],
                  [lx1*np.sin(a1) - ly1*np.cos(a1), lx2*np.sin(a2) - ly2*np.cos(a2), lx3*np.sin(a3) - ly3*np.cos(a3), lx4*np.sin(a4) - ly4*np.cos(a4)]])

    # T[:,1] = np.zeros(4) 
    # T[:,2] = np.zeros(4)

    T_pinv = np.linalg.pinv(T)
    f = np.dot(T_pinv,tau)
    print(f)
    cmd = inverse_glf_map(f)

    # rospy.loginfo(f"f = {f}")

    # rospy.loginfo(f"cmd = {cmd}")
    print(cmd)

    while not rospy.is_shutdown():

        fl_cmd.publish(cmd[0])
        rl_cmd.publish(cmd[1])
        rr_cmd.publish(cmd[2])
        fr_cmd.publish(cmd[3])
        rospy.Rate(10).sleep

if __name__ == '__main__':
    control()
