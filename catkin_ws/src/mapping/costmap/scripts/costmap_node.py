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
    sigma = 0
    kernel_size = 0

# Wrapper to synchronize access to OccupancyGrid costmap
class State:
    def __init__(self):
        self.costmap = None # OccupancyGrid

    # Takes an occupancy grid from gmapping, updating costmap. Pass a mutex to make thread-safe.
    def update(self, newGridmap, mutex=None):
        # Do calculations on newGridmap here


        # Reading variables must be done synchronously to avoid race conditions
        if (mutex): mutex.acquire()
        # Update costmap here


        if (mutex): mutex.release()

    # Returns costmap object to be broadcast to rostopic. Pass a mutex to make thread-safe.
    def get_costmap(self, mutex=None):
        if (mutex): mutex.acquire()
        c = copy.deepcopy(costmap)
        if (mutex): mutex.release()

        return c


# Keeps track of odometry costmap and interfaces with ros
class CostmapNode:
    def __init__(self):
        rospy.init_node('costmap_node', anonymous=False)

        self.state = State()
        self.mutex = threading.Lock()

        self.publisher = rospy.Publisher('/localization/dead_reckon/odom', Odometry, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber('/map', OccupancyGrid, self.grid_map_callback, queue_size=1)
        

    # Asynchronously updates with new data
    # TODO: call self.costmap.update() with relevant parameters from data
    def grid_map_callback(self, data):
        self.state.update(data, mutex=self.mutex)
        self.publish_map()


    # Publish odometry information to rostopic
    def publish_map(self):
        costmap = self.state.get_costmap(mutex=self.mutex)
        if(costmap):
            self.publisher.publish(costmap)   


if __name__ == '__main__':
    try:
        costmap = CostmapNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass