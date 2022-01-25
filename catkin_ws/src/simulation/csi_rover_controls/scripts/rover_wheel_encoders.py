#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RoverWheelEncoders:

    def __init__(self):

        # user can change the naming of the axles
        self.fl_axle_name = rospy.get_param("left_front_axle_name", "left_front_axle")
        self.fr_axle_name = rospy.get_param("right_front_axle_name", "right_front_axle")
        self.rl_axle_name = rospy.get_param("left_rear_axle_name", "left_rear_axle")
        self.rr_axle_name = rospy.get_param("right_rear_axle_name", "right_rear_axle")

        # publishers
        fr_wheel_pub = rospy.Publisher("/csi_rover/fr_wheel/encoder_value", Float64, queue_size=5)
        fl_wheel_pub = rospy.Publisher("/csi_rover/fl_wheel/encoder_value", Float64, queue_size=5)
        rr_wheel_pub = rospy.Publisher("/csi_rover/rr_wheel/encoder_value", Float64, queue_size=5)
        rl_wheel_pub = rospy.Publisher("/csi_rover/rl_wheel/encoder_value", Float64, queue_size=5)

        # create and initialize the class variables.
        self.fr_wheel_pos = 0
        self.fl_wheel_pos = 0
        self.rr_wheel_pos = 0
        self.rl_wheel_pos = 0

        # subscribe to the joint state topic.
        rospy.Subscriber("/csi_rover/joint_states", JointState, callback=self.joint_states_callback)

        rospy.init_node('rover_wheel_encoders', anonymous=True)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            fr_wheel_pub.publish(self.fr_wheel_pos)
            fl_wheel_pub.publish(self.fl_wheel_pos)
            rr_wheel_pub.publish(self.rr_wheel_pos)
            rl_wheel_pub.publish(self.rl_wheel_pos)

            rate.sleep()

    def joint_states_callback(self, msg):

        names = msg.name
        position = msg.position

        for i in range(0, len(names)):

            # pick out all of the axle position and
            # assign them to the class variables.

            if names[i] == self.fl_axle_name:
                self.fl_wheel_pos = position[i]
            elif names[i] == self.rl_axle_name:
                self.rl_wheel_pos = position[i]
            elif names[i] == self.fr_axle_name:
                self.fr_wheel_pos = position[i]
            elif names[i] == self.rr_axle_name:
                self.rr_wheel_pos = position[i]


if __name__ == "__main__":
    try:
        RoverWheelEncoders()
    except rospy.ROSInterruptException:
        pass
