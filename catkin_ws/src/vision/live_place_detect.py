import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points
from ros_numpy.point_cloud2 import pointcloud2_to_array


pub = rospy.Publisher("ground_plane_points", PointCloud2, queue_size=2)


def callback(data):

    # Get XYZ points from stream
    points = np.array([point for point in read_points(data, skip_nans=True)]).astype(
        "float32"
    )
    pc2_array = pointcloud2_to_array(data)
    print(pc2_array.shape, points.shape)
    points = points[:, :3]  # skip intensity for now

    # Instantiate point cloud object
    pcd = o3d.geometry.PointCloud(points=o3d.cpu.pybind.utility.Vector3dVector(points))

    # o3d.io.write_point_cloud('./input_cloud.ply', pcd)

    # Use RANSAC to segment PC into ground plane
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.05, ransac_n=3, num_iterations=100
    )

    # Create new PCDs (one for ground and one for not-ground)
    inlier_cloud = pcd.select_by_index(inliers)
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    points = np.asarray(inlier_cloud.points)
    header = data.header
    cloud_msg = create_cloud_xyz32(header, points)

    # Publish
    pub.publish(cloud_msg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("ground_finder", anonymous=True)

    rospy.Subscriber("/rslidar_points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()
