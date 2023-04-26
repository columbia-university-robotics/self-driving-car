# Systems Package

## Car

### Purpose

The car launch file remaps several topics for the VESC nodes and launches the `vesc_driver_node` via the `vesc_driver_node.launch` file.

Input topics of `vesc_driver_node` (any value accepted, but cutoffs hardware-specific unless otherwise stated):
 - systems/output/speed (std_msgs/Float64.msg): Motor command speed in rad/s, controlled by firmware using PID.
 - systems/output/current (std_msgs/Float64.msg): Motor current in amps.
 - systems/output/brake (std_msgs/Float64.msg): Motor braking current in amps.
 - systems/output/steer_angle (std_msgs/Float64.msg): Servo command position, ranging from 0 to 1. Cutoffs enforced automatically.

Output topics of `vesc_driver_node`:
 - systems/vesc/sensors/servo_position (std_msgs/Float64.msg): Gives most recent servo position command (value between 0 to 1). Not an encoder.
 - systems/vesc/sensors/core (vesc_msgs/VescStateStamped.msg): Gives time-stamped vesc-specific data

### Launch

0. Run the Docker container (see the main README for details).

1. Launch the car node.

```$ roslaunch systems car.launch```

## Joystick

### Purpose

The joystick launch file remaps the joy input (`sensor_msgs/Joy`) and launches the `joy_node`. The `joy_node` listens for input from a game controller or joystick and publishes a `sensor_msgs/Joy` message. To drive the car, one must also run the TeleOp package to relay the commands from the joystick to the VESC driver node.

For more information of the `joy_node` so the [ROS wiki](http://wiki.ros.org/joy).

### Launch

The most common way to launch this node is in the control package with the TeleOp launch file.
