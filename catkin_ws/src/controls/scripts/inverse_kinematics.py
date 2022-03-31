#!/usr/bin/env python
import math
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import constants

def calc_rpm(lin_vel, wheel_radius):
  lin_vel_magnitude = np.linalg.norm(lin_vel)
  rpm = lin_vel_magnitude * 60 / (2 * math.pi * wheel_radius)
  return rpm


def calc_steer_angle(lin_vel, ang_vel, wheel_base_length):
  lin_vel_magnitude = np.linalg.norm(lin_vel)
  ang_vel_z = ang_vel[2]
  n = (lin_vel_magnitude * wheel_base_length) / ang_vel_z
  steering_angle = math.atan(n)
  steering_angle = (steering_angle + (math.pi/2))/math.pi
  return steering_angle

class InverseKinematicsNode():

  def cmd_callback(self, data):
    self.lin_vel_cmd = np.array(data.linear)
    
  def __init__(self):
    self.cmd_subscriber = rospy.Subscriber('cmd', Twist, self.cmd_callback, queue_size=2)
    self.rpm_publisher = rospy.Publisher("systems/output/speed", Float64)
    self.steer_angle_publisher = rospy.Publisher("systems/output/steer_angle", Float64)

    rospy.init_node("inverse_kinematics_node")
    
    rate = rospy.Rate(50)

    self.lin_vel_cmd = None
    self.ang_vel_cmd = None
    
    while not rospy.is_shutdown():
      print("in while loop")
      if self.lin_vel_cmd == None or self.ang_vel_cmd == None:
        continue
      self.rpm_publisher.publish(calc_rpm(self.lin_vel_cmd, constants.radius))
      self.steer_angle_publisher.publish(calc_steer_angle(self.lin_vel_cmd, self.ang_vel_cmd, constants.l))
      rate.sleep()
