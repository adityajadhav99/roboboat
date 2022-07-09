from re import X

from yaml import safe_dump_all
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
        self.goal_state = np.zeros(3)
        self.goal_state = self.goal_state.reshape(3,1)
        self.error = self.current_state = self.goal_state

    def goal_callback(self, msg):
        goal_lat = msg.pose.position.latitude
        goal_long = msg.pose.position.longitude

        q = (msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(q)

        (_, _, goal_yaw) = euler

        self.goal_state = np.array([[goal_lat],[goal_long],[goal_yaw]])

        rospy.loginfo(f"Goal state = {self.goal_state}")
    
    def latlong2xy(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        y = math.cos(azimuth) * distance
        x = math.sin(azimuth) * distance

        self.goal_state[0] = x
        self.goal_state[1] = y
        return 

    def current_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

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

        datum = [-33.73, 150.67]
        rospy.wait_for_message('/wamv/robot_localization/odometry/filtered', Odometry)
        self.latlong2xy(datum[0], datum[1], self.goal_state[0], self.goal_state[1])

        while not rospy.is_shutdown():
            
            self.error = self.current_state - self.goal_state
            # rospy.loginfo(f"Current state: {self.current_state}\tGoal State: {self.goal_state}")
            rospy.loginfo(f"goal state: {self.error}")

if __name__ == '__main__':
    try:
        rospy.init_node('station_keep')
        control = StationKeep()
        control.execute()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
        

        

