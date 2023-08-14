# Mapping Package

## Laser Mapping

### Purpose

The laser mapping node, specifically `hector_mapping`, subscribes to the `/scan` topic, and uses SLAM to construct an occupancy grid modeling the world and the robot's pose within it. The occupancy grid is in the form of a `nav_msgs/OccupancyGrid` and is published to the `/map` topic; the robot's pose is in the form of a `geometry_msgs/PoseStamped` and is published to the `/slam_out_pose` topic.

When the node is started, the origin of the map is the current location of the robot.

For more details about `hector_mapping`, see the [ROS wiki](http://wiki.ros.org/hector_mapping).

### Launch

0. Run the Docker container (see the main README for details).

1. Launch the laser scan node.

```$ roslaunch sensing sensing.launch```

2. Launch the mapping node.

```$ roslaunch laser_mapping mapping.launch```

## Costmap

This node is not currently used.