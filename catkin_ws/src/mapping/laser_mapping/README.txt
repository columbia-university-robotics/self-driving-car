This file contains the API definition for the mapping I/O related to the costmap node. 

For all components that interact with these data streams, please read the topic definitions carefully.

laser_mapping.launch: 
	Input topics:
		/cloud_in (sensor_msgs/PointCloud2): the input point cloud

	Output topics:
		/scan (sensor_msgs/LaserScan): the output laser scan 
