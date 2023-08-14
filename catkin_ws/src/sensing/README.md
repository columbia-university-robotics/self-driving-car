# Sensing Package

## Sensing

### Purpose

The sensing node, `urg_node`, connects to the LiDAR sensor and publishes a `sensor_msgs/LaserScan` to the `/scan` topic. This information is used downstream for mapping and motion planning.

For more details about `urg_node`, see the [Github repo](https://github.com/ros-drivers/urg_node) and the [ROS wiki](http://wiki.ros.org/urg_node).

### Launch

0. Run the Docker container (see the main README for details).

1. Launch the sensing node.

```$ roslaunch sensing sensing.launch```
