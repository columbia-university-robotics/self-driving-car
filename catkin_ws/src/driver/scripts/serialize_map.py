#!/usr/bin/env python
import rospy

import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class MapSerializer():
    
    def map_callback(self, data):
        grid = np.array(data.data).reshape((data.info.height, data.info.width))
        with open("/occupancy_grid.npy", "wb") as f:
            np.save(f, grid)

        metadata = np.array([
            data.info.resolution,
            data.info.width,
            data.info.height,
        ])
        with open("/map_metadata.npy", "wb") as f:
            np.save(f, metadata)

        map_origin = np.array([
            data.info.origin.position.x,
            data.info.origin.position.y,
            data.info.origin.position.z,
            data.info.origin.orientation.x,
            data.info.origin.orientation.y,
            data.info.origin.orientation.z,
            data.info.origin.orientation.w,
        ])
        with open("/map_origin.npy", "wb") as f:
            np.save(f, map_origin)


    def pose_callback(self, data):
        pose = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ])
        with open("/pose.npy", "wb") as f:
            np.save(f, pose)

    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=10)
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.pose_callback, queue_size=10)

        rospy.init_node("map_serializer_node")

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    MapSerializer()

