import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2

pub = rospy.Publisher("ground_plane_points", PointCloud2, queue_size=2)

type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]

nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)


def dtype_to_fields(dtype):
    """
    Convert a numpy record datatype into a list of PointFields.
    """
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields


def callback(data):

    # Get XYZ points from stream
    points = np.array(
        [point for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True)]
    )
    points = points[:, :3]  # skip intensity for now

    # Instantiate point cloud object
    pcd = o3d.geometry.PointCloud(
        points=o3d.cpu.pybind.utility.Vector3dVector(np.asarray(points))
    )

    # o3d.io.write_point_cloud('./input_cloud.ply', pcd)

    # Use RANSAC to segment PC into ground plane
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.05, ransac_n=3, num_iterations=300
    )

    # Create new PCDs (one for ground and one for not-ground)
    inlier_cloud = pcd.select_by_index(inliers)
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # Convert plane points to message (use ros to Point_Cloud)
    point_cloud_msg = PointCloud2()
    point_cloud_msg.header.stamp = rospy.Time.now()
    point_cloud_msg.header.frame_id = data.header.frame_id
    point_cloud_msg.height = inlier_cloud.points.shape[0]
    point_cloud_msg.width = inlier_cloud.points.shape[1]
    point_cloud_msg.fields = dtype_to_fields(inlier_cloud.points.dtype)
    point_cloud_msg.is_bigendian = False  # assumption
    point_cloud_msg.point_step = inlier_cloud.points.dtype.itemsize
    point_cloud_msg.row_step = point_cloud_msg.point_step * inlier_cloud.points.shape[1]
    point_cloud_msg.is_dense = all(
        [
            np.isfinite(inlier_cloud.points[fname]).all()
            for fname in inlier_cloud.points.dtype.names
        ]
    )
    point_cloud_msg.data = inlier_cloud.points.tostring()

    # Publish
    pub.publish(point_cloud_msg)


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
