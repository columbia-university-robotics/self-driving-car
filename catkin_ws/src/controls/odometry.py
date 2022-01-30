#!/usr/bin/env python
import copy
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
    @staticmethod
    def add_headings(h1, h2):
        return math.fmod(math.pi+h1+h2, math.pi*2)-math.pi
    @staticmethod
    def avg_of_angles(a, b):
        diff = math.fmod(a - b + 180 + 360, 360) - 180
        return math.fmod(360 + b + diff/2, 360)

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
        self.vheading = 0.0
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.timestamp = rospy.Time(0) # Time zero means we don't yet know time

    # Takes speed of wheels in rad/s and turning angle in radians, updating state. Pass a mutex to make thread-safe.
    def update(self, wheelSpeed, steerAngle, timestamp, mutex=None):
        
        # Reading variables must be done synchronously to avoid race conditions
        if(mutex): mutex.acquire()
        old=copy.deepcopy(self)
        if(mutex): mutex.release()

        # Create new state
        new=copy.deepcopy(old)
        new.timestamp = timestamp
        new.steerAngle = steerAngle

        # Find new speed and generate trapezium approximation for average speed over time period
        new.speed = Constants.radius*wheelSpeed
        interp_speed = (old.speed+new.speed)/2
        
        # Code that requires a time interval (don't run the first time)
        if(old.timestamp!=rospy.Time(0)): 
            dt = (new.timestamp - old.timestamp).to_sec()

            interp_steerAngle = Util.avg_of_angles(old.steerAngle, steerAngle)

            interp_vheading = (interp_speed/Constants.l)*np.tan(np.deg2rad(interp_steerAngle))

            new.heading = Util.add_headings(old.heading, interp_vheading*dt)

            new.vheading = (new.speed/Constants.l)*np.tan(np.deg2rad(new.steerAngle))

            interp_heading = Util.avg_of_angles(old.heading, new.heading)

            new.x = old.x + interp_speed*math.cos(interp_heading)*dt
            new.y = old.y + interp_speed*math.sin(interp_heading)*dt

            new.vx = new.speed*math.cos(new.heading)
            new.vy = new.speed*math.sin(new.heading)
        
        if(mutex): mutex.acquire()
        if(new.timestamp > self.timestamp):
            # Replace self with new
            self.__dict__.update(new.__dict__)
        if(mutex): mutex.release()

    # Returns Odometry object to be broadcast to rostopic. Pass a mutex to make thread-safe.
    def get_odometry(self, mutex=None):
        if(mutex): mutex.acquire()
        state = copy.deepcopy(self)
        if(mutex): mutex.release()

        odom = Odometry()

        # Set timestamp
        odom.header.stamp = state.timestamp
        odom.header.frame_id = "odom"

        # set position
        quat = tf.transformations.quaternion_from_euler(0, 0, state.heading)
        odom.pose.pose = Pose(Point(state.x, state.y, 0.), Quaternion(*quat))

        # set velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(state.vx, state.vy, 0), Vector3(0, 0, state.vheading))
        
        return odom
    
# Keeps track of odometry state and interfaces with ros
class OdometryNode:
    def __init__(self):
        rospy.init_node('odometry', anonymous=False)

        self.state=State()
        self.mutex=threading.Lock()

        self.publisher = rospy.Publisher('/localization/dead_reckon/odom', Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.subscriber_callback, queue_size=2)

    # Asynchronously updates with new data
    # TODO: call self.state.update() with relevant parameters from data
    def subscriber_callback(self, data):
        self.state.update(0, 0, rospy.Time.now(), mutex=self.mutex)

    # Publish odometry information to rostopic
    def publish_odometry(self):
        if self.publisher.get_num_connections() == 0:
            return
        odom = self.state.get_odometry(mutex=self.mutex)
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