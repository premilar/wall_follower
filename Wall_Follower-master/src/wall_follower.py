#!/usr/bin/env python2

import numpy as np

import rospy
import math
from scipy import stats
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    Kp = 2.3 
    Kd = 1 
    Ki = 0
  

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        self.lidar_data_subscriber = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.lidar_data_subscriber_callback, queue_size=10)
        self.wall_marker = rospy.Publisher('wall_marker_topic', Marker, queue_size=10)
        self.rate = rospy.Rate(20) # 20hz
        self.time_seconds = rospy.get_time()
        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.error = 0
        self.lidar_data_ranges = []
        self.lidar_data_angles = []

    def lidar_data_subscriber_callback(self,lidar_data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", lidar_data)
        ack_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        
        lidar_data.scan_time = abs(self.time_seconds- rospy.get_time())
        self.time_seconds = rospy.get_time() # stores current time of scan

        
        
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        angle_min = lidar_data.angle_min
        angle_max = lidar_data.angle_max
        angle_increment = lidar_data.angle_increment
        min_dist = min(lidar_data.ranges)
        max_dist = max(lidar_data.ranges)
        print(min_dist, max_dist)

        np_lidar_data_distances = np.array(lidar_data.ranges)
        
        np_lidar_data_angles = np.linspace(angle_min, angle_max, len(np_lidar_data_distances))
      
        
        print('these should be the same length', len(np_lidar_data_distances), len(np_lidar_data_angles))


        if self.SIDE == 1:
            # only look at left half of data
           
            wall_distances = np_lidar_data_distances[40*len(np_lidar_data_distances)//100:60*len(np_lidar_data_distances)//100]
            wall_angles = np_lidar_data_angles[40*len(np_lidar_data_distances)//100:60*len(np_lidar_data_distances)//100]
            print(len(wall_angles), len(wall_distances))
           
        else:
            # only look at right half of data
             wall_distances = np_lidar_data_distances[31*len(np_lidar_data_distances)//100:57*len(np_lidar_data_distances)//100]
            wall_angles = np_lidar_data_angles[31*len(np_lidar_data_distances)//100:57*len(np_lidar_data_distances)//100]
           
            print(len(wall_angles), len(wall_distances))
            
        wall_distances_filtered = []
        wall_angles_filtered = []
        for i, dist in enumerate(wall_distances):
            if dist < 6:
                wall_distances_filtered.append(dist)
                wall_angles_filtered.append(wall_angles[i])
        wall_distances_filtered = np.array(wall_distances_filtered)
        wall_angles_filtered = np.array(wall_angles_filtered)
    
        x_vals = wall_distances*(np.cos(wall_angles))
        y_vals = wall_distances*(np.sin(wall_angles))
        print(x_vals, y_vals)
        slope, intercept, r_value, p_value, std_err = stats.linregress(x_vals, y_vals)

        
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = x_vals[0]
        first_line_point.y = slope*first_line_point.x + intercept
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = x_vals[len(x_vals)-1] #  [len(x_vals)-1]
        second_line_point.y = slope*second_line_point.x + intercept
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # Publish the Marker
        self.wall_marker.publish(marker)

        distance_to_wall = abs(intercept)/math.sqrt(slope**2+1)
        corresponding_angle_to_wall = np.tan(slope)


        # pid controller
        self.error = distance_to_wall - self.DESIRED_DISTANCE
        # integral = integral + error * lidar_data.scan_time
        derivative = (corresponding_angle_to_wall - (self.SIDE*math.pi/2))
        angle_to_wall = (self.SIDE*self.Kp*self.error + self.Kd*self.derivative)
       

        ack_msg.drive.speed = self.VELOCITY
        ack_msg.drive.steering_angle = angle_to_wall

        rospy.loginfo(ack_msg)
        ack_pub.publish(ack_msg)

  
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
