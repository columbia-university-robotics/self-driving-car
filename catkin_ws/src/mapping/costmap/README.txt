This file contains the API definition for the mapping I/O related to the costmap node. 

For all components that interact with these data streams, please read the topic definitions carefully.

Costmap_node.py: 
	Output topics:
		/mapping/costmap(nav_msgs/Odometry.msg): outputs costmap odometry data

	Gmapping --> Input topics:
		/maps(nav_msgs/OccupancyGrid.msg): takes in occupancygrid data from Gmapping
