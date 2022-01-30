#!/usr/bin/env python
import numpy as np
import threading
import math
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

# Useful methods
# TODO: Create library for project-wide methods such as these
class Util:
    # Adds two angles, keeping them within [-pi, pi)
    def add_headings(h1, h2):
        return math.fmod(math.pi+h1+h2, math.pi*2)-math.pi

# Useful constants.
# TODO: Create library for project-wide constants such as these
class Constants:
  l = 1 #wheel base length (m)
  w = 1 #lateral wheel seperation (m)
  radius = 1 #radius of the wheel (m)

# Stores physical position and velocity of robot
class State:
    def __init__(self):
        self.speed = 0.0
        self.steerAngle = 0.0 
        self.heading = 0.0 # In range [-pi, pi)
        self.x = 0.0
        self.y = 0.0
        self.timestamp = rospy.Time(0) # Time zero means we don't yet know time
        self.mutex=threading.Lock()

    # Takes speed of wheels in rad/s and turning angle in radians, updating state. Thread-safe.
    def update(self, wheelSpeed, steerAngle, newTimestamp):
        self.mutex.acquire()
        # Update speed and generate trapezium approximation for average speed over time period
        new_speed = Constants.radius*wheelSpeed
        interp_speed = (self.speed+new_speed)/2
        self.speed=new_speed

        # Code that requires a time interval (don't run the first time)
        if(self.timestamp!=rospy.Time(0)): 
            dt = (newTimestamp - self.timestamp).to_sec()

            # Update steer angle
            dHeadingdt = (interp_speed/Constants.l)*np.tan(np.deg2rad(steerAngle))
            new_heading = Util.add_headings(self.heading, dHeadingdt*dt)

            # Trapezium approximating average heading over time period
            interp_heading = (self.heading+new_heading)/2

            self.heading = new_heading

            self.x += interp_speed*math.cos(interp_heading)*dt
            self.x += interp_speed*math.sin(interp_heading)*dt

            self.timestamp = newTimestamp

        self.mutex.release()
    
    # Returns Odometry object to be returned via ROS, or None if Odometry has no data. Thread-safe.
    def get_odometry(self):
        self.mutex.acquire()

        # Return None if no data has been sent yet
        if(self.timestamp == rospy.Time(0)):
            self.mutex.release()
            return None

        odom = Odometry()

        # Set timestamp
        odom.header.stamp = self.timestamp
        odom.header.frame_id = "odom"

        # set position
        quat = tf.transformations.quaternion_from_euler(0, 0, self.heading)
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*quat))

        # set velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        self.mutex.release()
        return odom
    
# Keeps track of odometry state and interfaces with ros
class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry', anonymous=False)

        self.state=State()

        self.publisher = rospy.Publisher('/localization/dead_reckon/odom', Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.subscriber_callback, queue_size=2)

    # Asynchronously updates with new data
    # TODO: call self.state.update() with relevant parameters from data
    def subscriber_callback(self, data):
        self.state.update(0, 0, rospy.Time.now())

    # Publish odometry information to rostopic
    def publish_odometry(self):
        if self.publisher.get_num_connections() == 0:
            return
        odom = self.state.get_odometry()
        if(odom):
            self.publisher.publish(odom)   

if __name__ == '__main__':
    try:
        odometry = OdometryNode()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            odometry.publish_odometry()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass