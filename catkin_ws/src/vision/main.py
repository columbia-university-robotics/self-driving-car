import time

import open3d as o3d


if __name__ == "__main__":

    # Source: http://www.cvlibs.net/datasets/kitti-360/index.php
    filename = "./input_cloud.ply"

    # Read PLY (Point Cloud)
    pcd = o3d.io.read_point_cloud(filename)

    # # Down sample if necessary
    # pcd = pcd.voxel_down_sample(voxel_size=0.5)

    # # Routine to create open3d point cloud from points
    # pcd = o3d.geometry.PointCloud(
    #     points=o3d.cpu.pybind.utility.Vector3dVector(
    #         np.asarray(
    #             pcd.points
    #         )
    #     )
    # )

    # Use RANSAC to segment PC into ground plane
    start = time.perf_counter()
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.05,
        ransac_n=3,
        num_iterations=300
    )
    end = time.perf_counter()
    print("Time to find ground plane: {} secs".format(end - start))

    # Create new PCDs (one for ground and one for not-ground)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # Write to PLY files
    try:
        o3d.io.write_point_cloud("./output_clouds/inlier_cloud_1.ply", inlier_cloud)
        o3d.io.write_point_cloud("./output_clouds/outlier_cloud_1.ply", outlier_cloud)
    except Exception as e:
        print("Something went wrong with writing PLY output files.")
        raise e
