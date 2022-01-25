#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


class SimpleRoverController:

    def __init__(self):

        # self.name_space = rospy.get_param("name_space", "/csi_rover")

        lf_steering_pub = rospy.Publisher("/csi_rover/left_front_steering_controller/command", Float64, queue_size=5)
        rf_steering_pub = rospy.Publisher("/csi_rover/right_front_steering_controller/command", Float64, queue_size=5)
        lr_steering_pub = rospy.Publisher("/csi_rover/left_rear_steering_controller/command", Float64, queue_size=5)
        rr_steering_pub = rospy.Publisher("/csi_rover/right_rear_steering_controller/command", Float64, queue_size=5)

        lf_axle_pub = rospy.Publisher("/csi_rover/left_front_axle_controller/command", Float64, queue_size=5)
        rf_axle_pub = rospy.Publisher("/csi_rover/right_front_axle_controller/command", Float64, queue_size=5)
        lr_axle_pub = rospy.Publisher("/csi_rover/left_rear_axle_controller/command", Float64, queue_size=5)
        rr_axle_pub = rospy.Publisher("/csi_rover/right_rear_axle_controller/command", Float64, queue_size=5)

        rospy.Subscriber("/csi_rover/commands/synchronized_steering", Float64, callback=self.steering_joint_angle_callback)
        rospy.Subscriber("/csi_rover/commands/synchronized_driving", Float64, callback=self.axle_joint_vel_callback)

        self.steering_cmd = 0
        self.linear_vel = 0

        rospy.init_node('simple_rover_controller', anonymous=True)
        rate = rospy.Rate(30)  # 10hz

        while not rospy.is_shutdown():

            lf_steering_pub.publish(self.steering_cmd)
            rf_steering_pub.publish(self.steering_cmd)
            lr_steering_pub.publish(self.steering_cmd)
            rr_steering_pub.publish(self.steering_cmd)

            lf_axle_pub.publish(self.linear_vel)
            lr_axle_pub.publish(self.linear_vel)
            rf_axle_pub.publish(self.linear_vel)
            rr_axle_pub.publish(self.linear_vel)

            rate.sleep()

    def steering_joint_angle_callback(self, data):
        self.steering_cmd = data.data

    def axle_joint_vel_callback(self, data):
        self.linear_vel = data.data


if __name__ == '__main__':
    try:
        SimpleRoverController()
    except rospy.ROSInterruptExoception:
        pass
