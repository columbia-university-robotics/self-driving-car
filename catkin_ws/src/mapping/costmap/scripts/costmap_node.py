#!/usr/bin/env python
import copy
import numpy as np
import threading
import math
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import time
import rospy

# Useful constants.
# TODO: Create library for project-wide constants such as these
class Constants:
    l = 1  # wheel base length (m)
    w = 1  # lateral wheel seperation (m)
    radius = 1  # radius of the wheel (m)


# Stores physical position and velocity of robot
class Costmap:
    def __init__(self):
        self.timestamp = rospy.Time(0)  # Time zero means we don't yet know time

    # Takes speed of wheels in rad/s and turning angle in radians, updating costmap. Pass a mutex to make thread-safe.
    def update(self, timestamp, mutex=None):

        # Reading variables must be done synchronously to avoid race conditions
        if (mutex): mutex.acquire()
        old = copy.deepcopy(self)
        if (mutex): mutex.release()

        # Create new costmap


    # Returns Odometry object to be broadcast to rostopic. Pass a mutex to make thread-safe.
    def get_costmap(self, mutex=None):
        if (mutex): mutex.acquire()
        state = copy.deepcopy(self)
        if (mutex): mutex.release()


# Keeps track of odometry costmap and interfaces with ros
class CostmapNode:
    def __init__(self):
        rospy.init_node('costmap_node', anonymous=False)

        self.costmap = Costmap()
        self.mutex = threading.Lock()

        self.subscriber = rospy.Subscriber('/grid_map', OccupancyGrid, self.grid_map_callback, queue_size=1)

    # Asynchronously updates with new data
    # TODO: call self.costmap.update() with relevant parameters from data
    def grid_map_callback(self, data):
        pass

    # Publish odometry information to rostopic
    def publish_map(self):
        pass


if __name__ == '__main__':
    try:
        costmap = CostmapNode()
        rate = rospy.Rate(25)

    except rospy.ROSInterruptException:
        pass