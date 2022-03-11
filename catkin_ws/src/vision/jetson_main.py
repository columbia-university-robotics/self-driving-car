import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2


def callback(data):

    # Get XYZ points from stream
    points = np.array([
        point
        for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True)
    ])
    points = points[:, :3] # skip intensity for now

    # Instantiate point cloud object
    pcd = o3d.geometry.PointCloud(
        points=o3d.cpu.pybind.utility.Vector3dVector(
            np.asarray(
                points
            )
        )
    )

    # o3d.io.write_point_cloud('./input_cloud.ply', pcd)

    # Use RANSAC to segment PC into ground plane
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.05,
        ransac_n=3,
        num_iterations=300
    )

    # Create new PCDs (one for ground and one for not-ground)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # Write to PLY files
    # TODO: generate some kind of output
   
 
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ground_finder', anonymous=True)

    rospy.Subscriber("/rslidar_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

