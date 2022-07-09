#!/usr/bin/env python3

import rospy
import tf
import pyproj
import math
import numpy as np
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry


"""
/vrx/station_keeping/goal
/vrx/station_keeping/mean_pose_error
/vrx/station_keeping/pose_error

/wamv/robot_localization/odometry/filtered
"""

class StationKeep():
    def __init__(self):

        self.current_pose_sub = rospy.Subscriber('/wamv/robot_localization/odometry/filtered', Odometry, self.current_pose_callback)

        self.goal_sub = rospy.Subscriber('/vrx/station_keeping/goal', GeoPoseStamped, self.goal_callback)
        
        self.goal_lat = None
        self.goal_long = None
        self.goal_heading = None

        self.current_state = np.array([[0],[0],[0]])

        self.Kp = np.array([[1000,0,0],[0,1000,0],[0,0,1000]])
        self.Kd = np.array([[100,0,0],[0,100,0],[0,0,100]])

        self.fl_cmd = rospy.Publisher('/wamv/thrusters/left_front_thrust_cmd', Float32, queue_size=1)
        self.fr_cmd = rospy.Publisher('/wamv/thrusters/right_front_thrust_cmd', Float32, queue_size=1)
        self.rl_cmd = rospy.Publisher('/wamv/thrusters/left_rear_thrust_cmd', Float32, queue_size=1)
        self.rr_cmd = rospy.Publisher('/wamv/thrusters/right_rear_thrust_cmd', Float32, queue_size=1)

        lx1 = 1.6;       ly1 = 0.7;         a1 = -0.785
        lx2 = -2.373776; ly2 = 1.027135;    a2 = 0.785
        lx3 = -2.373776; ly3 = -1.027135;   a3 = -0.785
        lx4 = 1.6;       ly4 = -0.7;        a4 = 0.785

        T = np.array([[np.cos(a1),np.cos(a2),np.cos(a3),np.cos(a4)],
                  [np.sin(a1),np.sin(a2),np.sin(a3),np.sin(a4)],
                  [lx1*np.sin(a1) - ly1*np.cos(a1), lx2*np.sin(a2) - ly2*np.cos(a2), lx3*np.sin(a3) - ly3*np.cos(a3), lx4*np.sin(a4) - ly4*np.cos(a4)]])
        
        self.T_pinv = np.linalg.pinv(T) # 4*3

    def goal_callback(self, msg):
        self.goal_lat = msg.pose.position.latitude
        self.goal_long = msg.pose.position.longitude

        q = (msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(q)

        (_, _, self.goal_heading) = euler
    
    def latlong2xy(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        y = math.cos(azimuth) * distance
        x = math.sin(azimuth) * distance

        return x, y

    def rotZ3D(self, angle):
        return np.array([[math.cos(angle), -math.sin(angle), 0],
                        [math.sin(angle), math.cos(angle), 0],
                        [0 ,0 ,1]])

    def inverse_glf_map(self, T):

        k = T

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


    def current_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.u = msg.twist.twist.linear.x
        self.v = msg.twist.twist.linear.y
        self.r = msg.twist.twist.angular.z

        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(q)

        (_, _, psi) = euler

        self.current_state[0] = x 
        self.current_state[1] = y
        self.current_state[2] = psi
        
        return 

    def execute(self):

        rospy.sleep(10)

        datum = [-33.73, 150.67]
        (self.goal_x, self.goal_y) = self.latlong2xy(datum[0], datum[1], self.goal_lat, self.goal_long)
        self.goal_state = np.array([[self.goal_x],[self.goal_y],[self.goal_heading]])

        rospy.loginfo(self.goal_state)

        while not rospy.is_shutdown():
            
            self.error = self.goal_state - self.current_state  
            self.error_dot = np.dot(self.rotZ3D(self.current_state[2]),-np.array([[self.u],[self.v],[self.r]]))

            tau_global = np.dot(self.Kp, self.error) + np.dot(self.Kd, self.error_dot) # tau in global frame
            tau_local = np.dot(np.transpose(self.rotZ3D(self.current_state[2])), tau_global)

            f = np.dot(self.T_pinv, tau_local)

            cmd = self.inverse_glf_map(f)
            self.fl_cmd.publish(cmd[0])
            self.rl_cmd.publish(cmd[1])
            self.rr_cmd.publish(cmd[2])
            self.fr_cmd.publish(cmd[3])

            rospy.Rate(10).sleep


if __name__ == '__main__':
    try:
        rospy.init_node('station_keep')
        control = StationKeep()
        control.execute()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
        

        

