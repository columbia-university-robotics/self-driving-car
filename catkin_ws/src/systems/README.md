car-driver.launch:
    Output topics (any value accepted, but cutoffs hardware-specific unless otherwise stated):
        systems/output/speed (std_msgs/Float64.msg): Motor command speed in rad/s, controlled by firmware using PID.
        systems/output/current (std_msgs/Float64.msg): Motor current in amps.
        systems/output/brake (std_msgs/Float64.msg): Motor braking current in amps.
        systems/output/steer_angle (std_msgs/Float64.msg): Servo command position, ranging from 0 to 1. Cutoffs enforced automatically.

    Input topics:
        systems/input/servo_position (std_msgs/Float64.msg): Gives most recent servo position command (value between 0 to 1). Not an encoder.
        systems/input/vesc_sensors (vesc_msgs/VescStateStamped.msg): Gives time-stamped vesc-specific data
        systems/input/lidar_pointcloud (sensor_msgs/PointCloud2): Gives lidar point cloud
        systems/input/zed_pointcloud (sensor_msgs/PointCloud2): Gives zed color point cloud

joystick.launch:
    Input topics:
        systems/input/joy (sensor_msgs/Joy): Gives joystick state.
