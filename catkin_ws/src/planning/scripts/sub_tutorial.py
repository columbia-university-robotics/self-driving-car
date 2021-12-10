#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid 

"""
def run_a_star(data, width, height):
    np_grid = np.array(data)
    np_grd = np.reshape(height, width)

    ******
    *****
"""

def callback(data):
    rospy.loginfo(data.info)
    rospy.loginfo(data.info.resolution)

    # run_a_star(data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('global_planner', anonymous=True)

    rospy.Subscriber("/map", OccupancyGrid, callback)
    
    print("I am subscribed to the topic now")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()













