Odometry:
Reference frame is with respect to the robot, based on the starting orientation/position of the robot; starting coordinates/speed are all
initialized to 0

Given wheel speed (RPM) and turning angle (radians) updates position by outputting odom object containing Twist and Pose messages

Inverse Kinematics: 
Reference frame is with respect to the robot, with the angular velocity based on how the robot is first oriented.

Given desired linear velocity (m/s) and angular velocity (rads/s), publishes the corresponding RPM output and steering angle position 
directly to the VESC. 
