#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class TeleOpNode():

    def callback(self, data):
        speed = data.axes[1] # Left stick vertical axis
        angle = data.axes[0] # Left stick horizontal axis

        speed_to_rad = ((speed)*(300))
        speed_to_rad = min(300, speed_to_rad)
        
        angle = (-1 * angle+1)/2
        
        self.speed_to_pub = speed_to_rad
        self.angle_to_pub = angle

    def __init__(self):
    
        self.speed_to_pub = 0
        self.angle_to_pub = 0.5

        self.speed_topic = rospy.Publisher("systems/output/speed", Float64, queue_size = 40)
        self.steer_angle_topic = rospy.Publisher("systems/output/steer_angle", Float64, queue_size = 40)
        
        rospy.Subscriber("/systems/input/joystick", Joy, self.callback)
        
        rospy.init_node("teleop_node")
    
        rate = rospy.Rate(50)
    
        while not rospy.is_shutdown():
            print("in while loop")
            print((self.speed_to_pub, self.angle_to_pub))
            self.speed_topic.publish(self.speed_to_pub)
            self.steer_angle_topic.publish(self.angle_to_pub)
            print("speed: " + str(self.speed_to_pub))
            rate.sleep()

if __name__ == '__main__':
    TeleOpNode()

