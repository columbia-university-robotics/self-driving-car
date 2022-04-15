import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points

pub = rospy.Publisher("mapping/pointcloud_xyz", PointCloud2, queue_size=2)

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
        [point for point in read_points(data, skip_nans=True)]
    )
    points = points[:, :3].astype("float32")  # skip intensity for now

    # Convert plane points to message (use ros to Point_Cloud)
    point_cloud_msg = create_cloud_xyz32(data.header, points)

    # Publish
    pub.publish(point_cloud_msg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("lidar_proccessor", anonymous=False)

    rospy.Subscriber("systems/input/lidar_pointcloud", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()

