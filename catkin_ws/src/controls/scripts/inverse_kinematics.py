#!/usr/bin/env python
import math
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from constants import Constants as constants

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def calc_rpm(lin_vel, wheel_radius):
  lin_vel_magnitude = np.linalg.norm(lin_vel)
  rpm = lin_vel_magnitude * 60 / (2 * math.pi * wheel_radius)
  return rpm


def calc_steer_angle(lin_vel, ang_vel, wheel_base_length):
  lin_vel_magnitude = np.linalg.norm(lin_vel)
  ang_vel_z = ang_vel[2]
  if np.isclose(ang_vel_z, 0, atol=0.001):
      steering_angle = 0
  else:
      n = (lin_vel_magnitude * wheel_base_length) / ang_vel_z
      steering_angle = math.atan(n)
  steering_angle = valmap(steering_angle, -math.pi/6, math.pi/6, 0, 1)
  print("steering angle:", steering_angle)
  return steering_angle


class InverseKinematicsNode():

  def cmd_callback(self, data):
    self.lin_vel_cmd = np.array([data.linear.x, data.linear.y, data.linear.z])
    self.ang_vel_cmd = np.array([data.angular.x, data.angular.y, data.angular.z])
    
  def __init__(self):

    rospy.init_node("inverse_kinematics_node")
    self.cmd_subscriber = rospy.Subscriber('cmd', Twist, self.cmd_callback, queue_size=2)
    self.rpm_publisher = rospy.Publisher("systems/output/speed", Float64)
    self.steer_angle_publisher = rospy.Publisher("systems/output/steer_angle", Float64)
 
    rate = rospy.Rate(30)

    self.lin_vel_cmd = np.zeros((3, ))
    self.ang_vel_cmd = np.zeros((3, ))
    
    while not rospy.is_shutdown():
      # if self.lin_vel_cmd.any() or self.ang_vel_cmd.any():
        # continue
      self.rpm_publisher.publish(calc_rpm(self.lin_vel_cmd, constants.radius))
      self.steer_angle_publisher.publish(calc_steer_angle(self.lin_vel_cmd, self.ang_vel_cmd, constants.l))
      rate.sleep()

if __name__ == "__main__":
    InverseKinematicsNode()