# Controls Package

## TeleOp

### Purpose

This node, named `teleop_node`, defined in the `teleop-pub-sub.py` script, subscribes to the `systems/input/joystick` topic, converts this input into speed and steer angle, and publishes to the `systems/output/speed` and `systems/output/steer_angle` topics.

### Launch

0. Run the Docker container (see the main README for details).

1. Launch the car nodes from the systems package.

```$ roslaunch systems car.launch```

2. Launch the teleop node.

```$ roslaunch controls teleop.launch```

3. Press the home buttom on the gamepad controller. You should see the blue light turn on; the car is now ready to drive. 

For questions, please contact Neil.

## Inverse Kinematics

### Purpose

This node, named `inverse_kinematics_node`, defined in the `inverse_kinematics.py` script, subscribes to the `cmd` topic, a Twist message, converts this to RPMs and steer angle, and publishers to the `systems/output/speed` and `systems/output/steer_angle` topics.

This node is used with the planning nodes when running autonomously.

### Launch

The launch file is a TODO. For now, use the `rosrun`

## Odometry

This node is not currently used.
