#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

def callback(data):
    speed = data.axes[1] # Left stick vertical axis
    angle = data.axes[0] # Left stick horizontal axis

    speed_to_rad = ((speed)*(1000))
    speed_to_rad = min(1000, speed_to_rad)
    
    angle = (angle+1)/2

    speed_topic.publish(speed_to_rad)
    steer_angle_topic.publish(angle)

def start():
    global speed_topic
    global steer_angle_topic

    speed_topic = rospy.Publisher("systems/output/speed", Float64)
    steer_angle_topic = rospy.Publisher("systems/output/steer_angle", Float64)
    
    rospy.Subscriber("systems/input/joystick", Joy, callback)

    rospy.init_node("teleop_node")

    rospy.spin()

if __name__ == '__main__':
    start()
