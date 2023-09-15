#!/usr/bin/env python
import rospy

import numpy as np

from geometry_msgs.msg import PoseStamped


class PoseAppender:
    def pose_callback(self, data):
        """Get pose from data, append to array, write array to file."""
        new_pose = np.array(
            [
                [
                    data.pose.position.x,
                    data.pose.position.y,
                    data.pose.position.z,
                    data.pose.orientation.x,
                    data.pose.orientation.y,
                    data.pose.orientation.z,
                    data.pose.orientation.w,
                ]
            ]
        )

        if len(self.poses):
            self.poses = np.append(self.poses, new_pose, axis=0)
        else:
            self.poses = new_pose

        np.save(self.filename, self.poses)

    def __init__(self, filename):
        self.filename = filename
        self.poses = []

        rospy.Subscriber(
            "/slam_out_pose", PoseStamped, self.pose_callback, queue_size=10
        )
        rospy.init_node("waypoint_pose_appender_node")

        rate = rospy.Rate(5)  # 5 Hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    PoseAppender("waypoint_poses.npy")
